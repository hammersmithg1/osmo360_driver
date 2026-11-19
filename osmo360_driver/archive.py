"""Decode archived Annex-B access units with their original mapped timestamps.

No ROS, camera handles or robot paths. Records are (bytes, unix_ns). The
archive must start at a decoder boundary; corruption is reported, not hidden.
PyAV is imported only when the derived images are requested.
"""

from fractions import Fraction


def decode_h264_records(records, *, codec_headers=b''):
    import av
    from osmo360_camera.vendor.nal import nal_starts

    decoder = av.CodecContext.create('h264', 'r')
    decoder.thread_count = 1
    if codec_headers:
        decoder.extradata = codec_headers
    clock = Fraction(1, 1_000_000_000)
    pending = set()

    def frames(decoded):
        for frame in decoded:
            if frame.pts is None:
                raise ValueError('Archived H.264 decoder returned a frame without its timestamp')
            # PyAV flush frames may omit time_base; packet PTS still use the
            # same nanosecond units supplied to this decoder for every AU.
            stamp = round(frame.pts * (frame.time_base or clock) * 1_000_000_000)
            if stamp not in pending:
                raise ValueError('Archived H.264 decoder returned an unmatched timestamp')
            pending.remove(stamp)
            if frame.is_corrupt:
                raise ValueError('Archived H.264 contains a corrupt decoded frame')
            yield frame, stamp

    for raw, stamp in records:
        if type(stamp) is not int or not 0 <= stamp < 1 << 63:
            raise ValueError('Archived H.264 timestamp must be a nonnegative int64')
        if stamp in pending:
            raise ValueError('Archived H.264 repeats a pending timestamp')
        packet = av.Packet(raw)
        packet.pts, packet.time_base = stamp, clock
        # Parameter sets/SEI carry no picture and owe no decoded image.
        if any(1 <= raw[at] & 31 <= 5 for at in nal_starts(raw)):
            pending.add(stamp)
        yield from frames(decoder.decode(packet))
        if len(pending) > 64:
            raise ValueError('Archived H.264 decoder did not produce frames for 64 access units')
    yield from frames(decoder.decode(None))
    if pending:
        raise ValueError('Archived H.264 ended with undecoded access units')
