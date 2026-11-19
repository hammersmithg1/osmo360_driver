"""Verified panoramic-video selection before opening a fresh capture owner.

Lens switching can select a photo profile in native firmware. Select/verify
both settings, with Record0 guards, rather than inferring video readiness from
USB connectivity. Retained Sleep/Awake ownership must be resolved elsewhere;
this function is never a reason to write settings under an active stream.
"""
from .controls import LensMode, ModeProfile


def ensure_video_mode(camera, *, guard=None):
    if guard is not None and not callable(guard):
        raise TypeError("Video mode guard must be callable or None")

    def check():
        if guard is not None:
            guard()

    def query(method):
        check()
        value = method()
        check()
        return value

    stream = getattr(camera, '_usb_stream', None)
    if (camera._usb_preview is not None or camera._imu is not None
            or stream is not None and stream.owns_camera_resources):
        raise RuntimeError('Video mode selection requires closed capture owners')

    def idle():
        if query(camera.state).recording_state != 0:
            raise RuntimeError('Video mode selection requires idle Record0; recording was not stopped')

    idle()
    lens = query(camera.lens_mode)
    profile = query(camera.mode_profile)
    before = dict(lens=int(lens), profile=int(profile.profile))
    changed = []
    if lens != LensMode.PANORAMIC:
        idle()
        check()
        camera.set_lens_mode(LensMode.PANORAMIC, **({"guard": check} if guard is not None else {}))
        changed.append('lens')
        # Native firmware may choose PHOTO after changing the lens. Its prior
        # video profile is not evidence of the current profile.
        profile = query(camera.mode_profile)
    if profile.profile != int(ModeProfile.VIDEO_PANO):
        idle()
        check()
        camera.set_mode_profile(ModeProfile.VIDEO_PANO, **({"guard": check} if guard is not None else {}))
        changed.append('profile')
    lens, profile = query(camera.lens_mode), query(camera.mode_profile)
    idle()
    if lens != LensMode.PANORAMIC or profile.profile != int(ModeProfile.VIDEO_PANO):
        raise RuntimeError('Camera did not confirm panoramic video mode; capture was not opened')
    return dict(ready=True, before=before, lens=int(lens), profile=int(profile.profile),
                changed=changed, recording_state=0)
