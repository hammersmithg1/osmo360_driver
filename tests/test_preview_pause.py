"""Retained preview pause barriers: fake sockets/codecs, never camera I/O."""

import queue
import struct
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from osmo360_camera.vendor import datalink
from osmo360_camera.vendor.duml import FLAG_REPLY, FLAG_REQUEST, Message, parse_all
from osmo360_camera.video import VideoStream


class FakeSocket:
    def __init__(self):
        self.packets = []
        self.closed = False
        self.block_command = None
        self.entered = threading.Event()
        self.release = threading.Event()
        self.acks = None

    def setsockopt(self, *args):
        pass

    def sendto(self, packet, address):
        self.packets.append(packet)
        if packet[6] == datalink.Packet.DATA:
            message = parse_all(packet[20:])[0]
            if message.command == self.block_command:
                self.block_command = None
                self.entered.set()
                if not self.release.wait(2):
                    raise AssertionError("Blocked fake send was not released")
            if self.acks is not None:
                self.acks.put(struct.unpack_from("<H", packet, 4)[0])
        return len(packet)

    def close(self):
        self.closed = True


def messages(sock):
    return [
        parse_all(packet[20:])[0]
        for packet in sock.packets
        if packet[6] == datalink.Packet.DATA
    ]


class DatalinkPauseTests(unittest.TestCase):
    def setUp(self):
        self.socket = FakeSocket()
        with (
            patch.object(datalink.socket, "socket", return_value=self.socket),
            patch.object(datalink, "bind_socket"),
        ):
            self.link = datalink.Datalink(liveview="action")
        self.link.ready.set()
        self.link._data_seq = 80
        self.workers = []
        self.ack_queue = None

    def tearDown(self):
        self.socket.release.set()
        self.link.stop()
        if self.ack_queue is not None:
            self.ack_queue.put(None)
        for worker in self.workers:
            worker.join(2)
            self.assertFalse(worker.is_alive())

    def start(self, function):
        thread = threading.Thread(target=function)
        thread.start()
        self.workers.append(thread)
        return thread

    def auto_ack(self):
        self.ack_queue = self.socket.acks = queue.Queue()

        def acknowledge():
            while True:
                seq = self.ack_queue.get()
                if seq is None:
                    return
                self.link._acknowledged(seq)

        self.start(acknowledge)

    def test_pause_waits_for_whole_inflight_heartbeat_batch_then_retains_socket(self):
        self.auto_ack()
        self.socket.block_command = (0, 0x81)  # Tick10: before the paired encoder00/82.
        with patch.object(datalink, "HEARTBEAT_INTERVAL", 0.001):
            self.start(self.link._heartbeat_loop)
            self.assertTrue(self.socket.entered.wait(1))
            result, finished = [], threading.Event()

            def pause():
                result.append(self.link.pause(1))
                finished.set()

            self.start(pause)
            self.assertFalse(finished.wait(0.02))
            self.assertFalse(any(m.command == (0, 0x82) for m in messages(self.socket)))
            self.socket.release.set()
            self.assertTrue(finished.wait(1))
            self.assertEqual(result, [True])
            sent = messages(self.socket)
            self.assertEqual([m.command for m in sent[-2:]], [(0, 0x81), (0, 0x82)])
            count = len(sent)
            time.sleep(0.015)
            self.assertEqual(
                len(messages(self.socket)), count
            )  # No more heartbeat/start batches.
        self.assertFalse(self.socket.closed)
        self.assertIs(self.link._sock, self.socket)
        seqs = [
            struct.unpack_from("<H", p, 4)[0]
            for p in self.socket.packets
            if p[6] == datalink.Packet.DATA
        ]
        self.assertEqual(seqs, list(range(80, 80 + 8 * len(seqs), 8)))
        self.assertEqual(self.link.stats["transmit_state"], "paused")
        self.assertTrue(self.link.stats["pause_drained"])

    def test_failed_drain_never_clears_pending_data_or_consumes_a_sequence(self):
        self.link.send(0, 0x82, b"\0")
        pending, seq, message_seq = (
            list(self.link._unacked),
            self.link._data_seq,
            self.link._message_seq,
        )
        self.assertFalse(self.link.pause(0.01))
        self.assertEqual(self.link.stats["transmit_state"], "quiescing")
        self.assertEqual(self.link._unacked, pending)
        with self.assertRaisesRegex(RuntimeError, "quiesced"):
            self.link.send(0, 0x82, b"\0")
        self.assertEqual(
            (self.link._data_seq, self.link._message_seq), (seq, message_seq)
        )
        self.assertTrue(self.link.resume(trigger_encoder=False))
        self.assertEqual(self.link._unacked, pending)
        self.link.send(0, 0x4F)
        self.assertEqual(self.link._unacked[-1][0], seq)

    def test_blocked_inflight_send_returns_false_on_pause_deadline_without_interrupting_the_batch(
        self,
    ):
        self.socket.block_command = (0, 0x82)
        sender = self.start(lambda: self.link.send(0, 0x82, b"\0"))
        self.assertTrue(self.socket.entered.wait(1))
        self.assertFalse(
            self.link.pause(0.01)
        )  # No barrier; native Sleep is forbidden.
        self.assertEqual(self.link.stats["transmit_state"], "active")
        self.socket.release.set()
        sender.join(1)
        self.assertFalse(sender.is_alive())
        self.assertEqual(self.link._data_seq, 88)
        self.link._acknowledged(80)
        self.assertTrue(self.link.pause(0.2))

    def test_passive_reply_during_drain_can_be_received_and_acked_without_deadlock(
        self,
    ):
        self.link.send(0, 0x82, b"\0")
        result, done = [], threading.Event()

        def pause():
            result.append(self.link.pause(1))
            done.set()

        self.start(pause)
        deadline = time.monotonic() + 1
        while self.link.stats["transmit_state"] != "quiescing":
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.001)
        # The real receiver does these on the same thread: a reply before ACK.
        self.link._answer(Message(1, 2, 999, FLAG_REQUEST, 0, 0x81, b""))
        self.link._acknowledged(self.link._unacked[-1][0])
        self.assertTrue(done.wait(1))
        self.assertEqual(result, [True])

    def test_paused_link_keeps_ack_and_passive_reply_but_never_replays_start(self):
        self.link.send(0, 0x82, b"\0")
        self.link._acknowledged(self.link._unacked[-1][0])
        self.assertTrue(self.link.pause(0.2))
        original = len(self.socket.packets)
        session, cursors = self.link._session, list(self.link._cursors)
        self.link._send_link(datalink.Packet.ACK, datalink.ack_body(*cursors))
        self.link._answer(Message(1, 2, 1000, FLAG_REQUEST, 0, 0x81, b""))
        self.link._acks_seen = True
        with (
            patch.object(datalink.time, "monotonic", return_value=time.monotonic() + 1),
            patch.object(datalink, "log"),
        ):
            self.link._resend()
        new = self.socket.packets[original:]
        self.assertEqual(new[0][6], datalink.Packet.ACK)
        replies = [parse_all(p[20:])[0] for p in new if p[6] == datalink.Packet.DATA]
        self.assertEqual(len(replies), 2)
        self.assertTrue(
            all(m.flags & FLAG_REPLY and m.command == (0, 0x81) for m in replies)
        )
        self.assertEqual((self.link._session, self.link._cursors), (session, cursors))

    def test_strict_pause_blocks_answers_and_resume_triggers_once_without_reinitialization(
        self,
    ):
        self.assertTrue(self.link.pause(0.2, keep_answer_requests=False))
        seq = self.link._data_seq
        self.link._answer(Message(1, 2, 1000, FLAG_REQUEST, 0, 0x81, b""))
        self.assertEqual(self.link._data_seq, seq)
        self.assertEqual(messages(self.socket), [])
        with (
            patch.object(self.link, "_handshake") as handshake,
            patch.object(datalink, "poke") as poke,
        ):
            self.assertTrue(self.link.resume())
            self.assertFalse(self.link.resume())
        handshake.assert_not_called()
        poke.assert_not_called()
        self.assertEqual(
            [m.command for m in messages(self.socket)], [(0, 0x81), (0, 0x82)]
        )
        self.assertFalse(self.socket.closed)

    def test_paused_reply_policy_cannot_silently_change_on_an_idempotent_call(self):
        self.assertTrue(self.link.pause(0.2))
        with self.assertRaisesRegex(ValueError, "passive reply policy"):
            self.link.pause(0.2, keep_answer_requests=False)
        self.assertTrue(self.link._keep_answer_requests)

    def test_partial_resume_trigger_error_quiesces_active_commands(self):
        self.assertTrue(self.link.pause(0.2))
        original = self.socket.sendto

        def fail_second(packet, address):
            message = parse_all(packet[20:])[0]
            if message.command == (0, 0x82):
                raise OSError("Encoder trigger failed")
            return original(packet, address)

        self.socket.sendto = fail_second
        with self.assertRaisesRegex(OSError, "Encoder trigger failed"):
            self.link.resume()
        self.assertEqual(self.link.stats["transmit_state"], "quiescing")
        self.assertFalse(self.link.stats["pause_drained"])
        with self.assertRaisesRegex(RuntimeError, "quiesced"):
            self.link.send(0, 0x82, b"\0")
        self.assertEqual([m.command for m in messages(self.socket)], [(0, 0x81)])


class FakeLink:
    def __init__(self, *, on_video, **kwargs):
        self.on_video = on_video
        self.stats = {"transmit_state": "active"}
        self.running, self.release = threading.Event(), threading.Event()
        self.pause_result = True
        self.resume_calls = []

    def run(self):
        self.running.set()
        self.release.wait(5)

    def stop(self):
        self.release.set()

    def pause(self, timeout, **kwargs):
        self.stats["transmit_state"] = "paused" if self.pause_result else "quiescing"
        return self.pause_result

    def resume(self, **kwargs):
        self.resume_calls.append(kwargs)
        self.stats["transmit_state"] = "active"
        return True


class FakePacket:
    def __init__(self, data):
        self.data = data


class FakeImage:
    def __init__(self, packet):
        self.pts, self.time_base = packet.pts, packet.time_base


class Codec:
    def __init__(self, delayed=False):
        self.inputs, self.pending = [], None
        self.delayed = delayed
        self.thread_count = None

    def decode(self, packet):
        self.inputs.append(packet)
        if not self.delayed:
            return [FakeImage(packet)]
        previous, self.pending = self.pending, packet
        return [FakeImage(previous)] if previous is not None else []


class VideoPauseTests(unittest.TestCase):
    def setUp(self):
        self.links = []

        def factory(**kwargs):
            value = FakeLink(**kwargs)
            self.links.append(value)
            return value

        self.link_patch = patch("osmo360_camera.video.Datalink", side_effect=factory)
        self.poke_patch = patch("osmo360_camera.video.poke")
        self.meta_patch = patch(
            "osmo360_camera.video.records_from_video",
            side_effect=lambda raw: [{"frame_clock_ns_raw": int(raw)}],
        )
        self.link_patch.start()
        self.poke = self.poke_patch.start()
        self.meta_patch.start()
        self.streams = []

    def tearDown(self):
        for stream in self.streams:
            stream.stop()
        self.meta_patch.stop()
        self.poke_patch.stop()
        self.link_patch.stop()

    def stream(self, codec):
        fake_av = SimpleNamespace(
            CodecContext=SimpleNamespace(create=Mock(return_value=codec)),
            Packet=FakePacket,
        )
        with patch.dict(sys.modules, {"av": fake_av}):
            stream = VideoStream(queue_size=2).start()
        self.streams.append(stream)
        self.assertTrue(self.links[-1].running.wait(1))
        return stream, self.links[-1], fake_av

    def test_pause_suppresses_only_publication_and_retains_decoder_and_link(self):
        codec = Codec()
        stream, link, fake_av = self.stream(codec)
        link.on_video(b"1", 0.0)
        self.assertEqual(stream.stats["queued_frames"], 1)
        self.assertTrue(stream.pause(0.2))
        self.assertEqual(stream.stats["queued_frames"], 0)
        link.on_video(b"2", 0.0)
        self.assertEqual(stream.stats["decoded_frames"], 2)
        self.assertEqual(stream.stats["suppressed_frames"], 1)
        with self.assertRaises(TimeoutError):
            stream.read(0)
        self.assertTrue(stream.resume(trigger_encoder=False))
        link.on_video(b"3", 0.0)
        self.assertEqual(stream.read(0).timestamp_ns, 3)
        self.assertEqual(len(codec.inputs), 3)
        self.assertIs(stream._decoder, codec)
        self.assertIs(stream._link, link)
        fake_av.CodecContext.create.assert_called_once()
        self.poke.assert_called_once()
        self.assertEqual(stream.stats["paused_queue_discards"], 1)

    def test_delayed_decoder_outputs_from_before_or_during_pause_stay_suppressed_after_resume(
        self,
    ):
        codec = Codec(delayed=True)
        stream, link, _ = self.stream(codec)
        link.on_video(b"1", 0.0)  # Decoder retains the pre-pause AU.
        self.assertTrue(stream.pause(0.2))
        stream.resume(trigger_encoder=False)
        link.on_video(b"2", 0.0)  # Old1 must not appear as the first resumed frame.
        with self.assertRaises(TimeoutError):
            stream.read(0)
        self.assertTrue(stream.pause(0.2))
        link.on_video(b"3", 0.0)
        stream.resume(trigger_encoder=False)
        link.on_video(b"4", 0.0)  # Paused3 remains suppressed even after resume.
        with self.assertRaises(TimeoutError):
            stream.read(0)
        link.on_video(b"5", 0.0)
        self.assertEqual(stream.read(0).timestamp_ns, 4)
        self.assertEqual(len(codec.inputs), 5)
        self.assertEqual(stream.stats["suppressed_frames"], 3)

    def test_failed_drain_keeps_publication_paused_until_explicit_recovery(self):
        stream, link, _ = self.stream(Codec())
        link.pause_result = False
        self.assertFalse(stream.pause(0.2))
        link.on_video(b"1", 0.0)
        self.assertTrue(stream.stats["paused"])
        self.assertEqual(stream.stats["published_frames"], 0)
        stream.resume(trigger_encoder=False)
        link.on_video(b"2", 0.0)
        self.assertEqual(stream.read(0).timestamp_ns, 2)
        self.assertEqual(link.resume_calls, [{"trigger_encoder": False}])


if __name__ == "__main__":
    unittest.main()
