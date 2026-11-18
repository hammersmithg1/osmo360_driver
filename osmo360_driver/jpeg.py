"""Small live previews and demand-driven, unchanged archival JPEGs.

One PreviewEncoder belongs to the video reader, so its PyAV scaler can be
reused. Each FrameJpegs belongs to one decoded frame only. The full-resolution
BGR conversion and JPEG92 remain exactly the legacy path when requested.
"""

PREVIEW_WIDTH = 1024
PREVIEW_HEIGHT = 512
PREVIEW_QUALITY = 75
ARCHIVE_QUALITY = 92


class PreviewEncoder:
    def __init__(self):
        import cv2
        self.cv2 = cv2
        self.reformatter = None

    def for_frame(self, frame):
        return FrameJpegs(self, frame)

    def small_bgr(self, image):
        if self.reformatter is None:
            from av.video.reformatter import VideoReformatter
            self.reformatter = VideoReformatter()
        return self.reformatter.reformat(image, width=PREVIEW_WIDTH,
            height=PREVIEW_HEIGHT, format='bgr24').to_ndarray()

    def resize(self, bgr):
        return self.cv2.resize(bgr, (PREVIEW_WIDTH, PREVIEW_HEIGHT),
                               interpolation=self.cv2.INTER_AREA)

    def encode(self, bgr, quality):
        ok, encoded = self.cv2.imencode('.jpg', bgr,
                                       [self.cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError('Не удалось подготовить JPEG Osmo')
        return encoded.tobytes()


class FrameJpegs:
    def __init__(self, encoder, frame):
        self.encoder, self.frame = encoder, frame
        self._full_bgr = self._full_jpeg = self._preview_jpeg = None

    def full(self):
        """Encode JPEG92 only once, and only when an archival consumer asks."""
        if self._full_jpeg is None:
            self._full_bgr = self.frame.bgr
            self._full_jpeg = self.encoder.encode(self._full_bgr, ARCHIVE_QUALITY)
        return self._full_jpeg

    def preview(self):
        if self._preview_jpeg is None:
            # A recording/ROS consumer runs first. Reuse its full BGR if needed;
            # otherwise downscale native YUV directly, avoiding the full BGR.
            bgr = self._full_bgr
            if bgr is None:
                bgr = getattr(self.frame, '_bgr', None)
            if bgr is not None:
                small = self.encoder.resize(bgr)
            elif getattr(self.frame, 'image', None) is not None:
                small = self.encoder.small_bgr(self.frame.image)
            else:
                # BGR-only frame adapters retain their existing compatibility.
                self._full_bgr = self.frame.bgr
                small = self.encoder.resize(self._full_bgr)
            self._preview_jpeg = self.encoder.encode(small, PREVIEW_QUALITY)
        return self._preview_jpeg
