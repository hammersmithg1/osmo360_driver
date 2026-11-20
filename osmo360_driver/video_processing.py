"""Facts about the SDK's video path, separate from camera ISP assumptions.

"Original H.264" means the received encoder bytes are retained unchanged. It
does not mean sensor RAW, lossless transport, an unstabilized camera image, or
calibrated geometry suitable for SLAM. The camera's liveview encoder is upstream
of this SDK; its internal ISP processing has not been verified.

Keep these facts independent of opaque SEI flags, fusion quaternions and raw
calibration blobs. None currently proves a camera-side stabilization setting or
a fixed geometric camera model. This module performs no I/O or camera commands.
"""


# Tuples and scalar values are immutable. Each caller gets its own dictionary,
# so a UI/manifest consumer cannot change the contract for another session.
_FACTS = (
    ('encoded_source', 'camera_liveview_encoder'),
    ('original_encoded_h264_preserved', True),
    ('sensor_raw', False),
    ('host_stabilization_applied', False),
    ('onboard_stabilization', 'unknown'),
    ('onboard_stabilization_disabled_verified', False),
    ('fixed_geometry_verified', False),
    ('slam_intrinsics_verified', False),
)


def video_processing_contract():
    """Return a fresh, JSON-ready copy of verified processing facts."""
    return dict(_FACTS)
