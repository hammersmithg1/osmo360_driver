"""Lossy bounded H.264 preview decoding, isolated from raw AU admission.

When overloaded, an interdependent H.264 stream cannot discard arbitrary
pictures and keep decoding. Resume only at the next IDR using cached codec
headers. This queue and its failures never discard an archival access unit.
"""
from collections import deque
import threading
import time


class PreviewDecoder:
    def __init__(self, stream, limit=8):
        self.stream, self.limit = stream, limit
        self.queue = deque()
        self.generation = 0
        self.applied_generation = 0
        self.waiting_idr = self.closed = False
        self.drops = self.resets = self.errors = 0
        self.error = ''
        self.request_pending = False
        self.keyframe_requests = 0
        self.keyframe_error = ''
        self.last_keyframe_request = -float('inf')
        self.thread = threading.Thread(target=self.run, name='osmo-preview-decode', daemon=True)

    def start(self):
        self.thread.start()

    def invalidate(self):
        owner = self.stream
        self.drops += len(self.queue)
        self.queue.clear()
        self.generation += 1
        self.waiting_idr = True
        self.request_pending = True  # One request per invalidation, never a poll loop.
        # Disallow output of an already decoding old AU after this boundary.
        owner._publication_epoch += 1

    def submit(self, metadata):
        owner = self.stream
        with owner._condition:
            if self.closed:
                return
            if len(self.queue) >= self.limit:
                self.invalidate()
            if self.waiting_idr:
                if not metadata[3].get('h264_keyframe'):
                    self.drops += 1
                    return
                self.waiting_idr = False
                self.request_pending = False  # An actual IDR already resolved this episode.
                metadata = (*metadata[:4], -1 if owner._publication_paused else owner._publication_epoch)
            self.queue.append((self.generation, metadata))
            owner._condition.notify_all()

    def run(self):
        owner = self.stream
        while True:
            request = False
            with owner._condition:
                owner._condition.wait_for(lambda: self.queue or self.closed or self.request_pending)
                if self.closed:
                    return
                if self.request_pending:
                    delay = 5 - (time.monotonic() - self.last_keyframe_request)
                    if delay <= 0:
                        request = True
                        self.request_pending = False
                        self.last_keyframe_request = time.monotonic()
                        self.keyframe_requests += 1
                    elif not self.queue:
                        owner._condition.wait(delay)
                        continue
                if not request:
                    generation, metadata = self.queue.popleft()
            if request:
                try:
                    owner.request_keyframe()
                except Exception as error:
                    # An unanswered request is diagnostic, not permission for a
                    # periodic encoder-restart loop. A later actual IDR resumes.
                    with owner._condition:
                        self.keyframe_error = f'{type(error).__name__}: {error}'[:500]
                continue
            try:
                if generation != self.applied_generation:
                    owner._decoder = owner._av.CodecContext.create('h264', 'r')
                    owner._decoder.thread_count = 1
                    owner._pending.clear()
                    owner._unstamped.clear()
                    self.applied_generation = generation
                    self.resets += 1
                    prefix = bytes.fromhex(metadata[3].get('h264_codec_headers_hex', ''))
                else:
                    prefix = b''
                owner._decode_access_unit(metadata, prefix=prefix)
            except Exception as error:
                with owner._condition:
                    self.errors += 1
                    self.error = f'{type(error).__name__}: {error}'[:500]
                    self.invalidate()
                    owner._condition.notify_all()

    def status(self):
        with self.stream._condition:
            return dict(queued_units=len(self.queue), queue_limit=self.limit,
                        dropped_units=self.drops, decoder_resets=self.resets,
                        errors=self.errors, error=self.error, waiting_idr=self.waiting_idr,
                        keyframe_requests=self.keyframe_requests, keyframe_error=self.keyframe_error)

    def stop(self, timeout):
        with self.stream._condition:
            self.closed = True
            self.queue.clear()
            self.stream._condition.notify_all()
        if self.thread.ident is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout)
            if self.thread.is_alive():
                raise TimeoutError('Preview decoder did not stop before timeout')
