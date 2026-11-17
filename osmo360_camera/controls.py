"""Pure camera-control codecs; this module never opens a USB device.

Public ``decode_*`` reply functions include the DUML status byte (00=success).
``encode_*`` returns request data only, without a status byte. Integer enum
values unknown to these headers are retained so a read/restore is lossless.
The explicit LensMode codec is strict and rejects unknown mode values.
Encoding validates wire ranges, not support in the camera's current mode.

Sources: extracted duml_cmdset.h and duml_cmd_camera.h (packed structures),
retail libdcam_cs_duss_proxy.so GET handlers, and EB6001 captured on this camera.
"""

from dataclasses import dataclass
from enum import IntEnum
from fractions import Fraction
import struct
from typing import TypeVar


CAMERA_COMMAND_SET = 2


class CameraCommand(IntEnum):
    SET_VIDEO_FORMAT = 0x18
    GET_VIDEO_FORMAT = 0x19
    SET_EXPOSURE_MODE = 0x1E
    GET_EXPOSURE_MODE = 0x1F
    SET_SHUTTER = 0x28
    GET_SHUTTER = 0x29
    SET_ISO = 0x2A
    GET_ISO = 0x2B
    SET_WHITE_BALANCE = 0x2C
    GET_WHITE_BALANCE = 0x2D
    SET_COLOR = 0x42
    GET_COLOR = 0x43
    PARAM_OPTION = 0x8E
    SET_MODE_PROFILE = 0xE1
    GET_CAMERA_INFO = 0xEB


class ControlDecodeError(ValueError):
    """A successful response has a malformed or unsupported wire layout."""


class CameraReplyError(RuntimeError):
    def __init__(self, status: int, payload: bytes):
        self.status = status
        self.payload = payload
        super().__init__(f"Camera rejected command: status 0x{status:02x}")


def _uint(value: int, bits: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if not 0 <= value < (1 << bits):
        raise ValueError(f"{name} must fit uint{bits}")
    return int(value)


def _reply(payload: bytes, length: int | tuple[int, ...]) -> bytes:
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError("reply must be bytes-like")
    payload = bytes(payload)
    if not payload:
        raise ControlDecodeError("Empty reply: missing status")
    if payload[0]:
        raise CameraReplyError(payload[0], payload)
    lengths = (length,) if isinstance(length, int) else length
    body = payload[1:]
    if len(body) not in lengths:
        raise ControlDecodeError(
            f"Expected body length {lengths}, received {len(body)}"
        )
    return body


E = TypeVar("E", bound=IntEnum)


def _known(enum: type[E], value: int) -> E | None:
    try:
        return enum(value)
    except ValueError:
        return None


class FrameRate(IntEnum):
    FPS15 = 0
    FPS24 = 1
    FPS25 = 2
    FPS30 = 3
    FPS48 = 4
    FPS50 = 5
    FPS60 = 6
    FPS120 = 7
    FPS240 = 8
    FPS480 = 9
    FPS100 = 10
    FPS96 = 11
    FPS180 = 12
    FPS24_TRUE = 13
    FPS30_TRUE = 14
    FPS48_TRUE = 15
    FPS60_TRUE = 16
    FPS90 = 17
    FPS192 = 18
    FPS200 = 19
    FPS400 = 20
    FPS7_5 = 21
    FPS20 = 22
    FPS8_8 = 23
    FPS120_TRUE = 24
    FPS96_TRUE = 25
    FPS72 = 26
    FPS72_TRUE = 27
    FPS75 = 28
    FPS15_TRUE = 29
    FPS10_TRUE = 30
    FPS5_TRUE = 31


# Retail duss_get_fps_map_tbls(): address0x9800c, 32 records {u8code,pad,u32num,den}.
# Header P30 is the rounded label for 30000/1001; P30T is true 30.
_FPS = (
    15,
    Fraction(24000, 1001),
    25,
    Fraction(30000, 1001),
    Fraction(48000, 1001),
    50,
    Fraction(60000, 1001),
    Fraction(120000, 1001),
    Fraction(240000, 1001),
    Fraction(480000, 1001),
    100,
    Fraction(96000, 1001),
    Fraction(180000, 1001),
    24,
    30,
    48,
    60,
    Fraction(90000, 1001),
    Fraction(192000, 1001),
    200,
    400,
    Fraction(15, 2),
    20,
    Fraction(44, 5),
    120,
    96,
    Fraction(72000, 1001),
    72,
    75,
    15,
    10,
    5,
)


class Resolution(IntEnum):
    P1920X1080 = 10
    P3840X2160 = 16
    # Retail OQ101 protocol-util table extends the bundled header after 110.
    SQUARE_4K = 111
    SQUARE_2880P = 113
    SQUARE_1920P = 115
    SQUARE_3000P = 117


class FieldOfView(IntEnum):
    NONE = 0
    NORMAL = 1
    NARROW = 2
    WIDE = 3
    SUPER_NARROW = 4


@dataclass(frozen=True)
class VideoFormat:
    resolution: int
    frame_rate: int
    fov: int
    secondary_config: int
    secondary_resolution: int

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            _uint(getattr(self, name), 8, name)

    @property
    def fps(self) -> Fraction | None:
        return Fraction(_FPS[self.frame_rate]) if self.frame_rate < len(_FPS) else None

    @property
    def secondary_enabled(self) -> bool:
        return bool(self.secondary_config & 1)

    @property
    def known_resolution(self) -> Resolution | None:
        return _known(Resolution, self.resolution)


def decode_video_format(reply: bytes) -> VideoFormat:
    return VideoFormat(*_reply(reply, 5))


def encode_video_format(value: VideoFormat) -> bytes:
    if not isinstance(value, VideoFormat):
        raise TypeError("value must be VideoFormat")
    return bytes(
        (
            value.resolution,
            value.frame_rate,
            value.fov,
            value.secondary_config,
            value.secondary_resolution,
        )
    )


class ExposureMode(IntEnum):
    AUTO = 0
    PROGRAM = 1
    SHUTTER_PRIORITY = 2
    APERTURE_PRIORITY = 3
    MANUAL = 4
    BULB = 5
    SCENE = 6
    CUSTOM = 7
    FACTORY = 100


@dataclass(frozen=True)
class ExposureModeSetting:
    mode: int
    scene_mode: int = 0

    def __post_init__(self):
        _uint(self.mode, 8, "mode")
        _uint(self.scene_mode, 8, "scene_mode")

    @property
    def known_mode(self) -> ExposureMode | None:
        return _known(ExposureMode, self.mode)


def decode_exposure_mode(reply: bytes) -> ExposureModeSetting:
    return ExposureModeSetting(*_reply(reply, 2))


def encode_exposure_mode(value: ExposureModeSetting) -> bytes:
    if not isinstance(value, ExposureModeSetting):
        raise TypeError("value must be ExposureModeSetting")
    return bytes((value.mode, value.scene_mode))


class WhiteBalanceMode(IntEnum):
    AUTO = 0
    SUNNY = 1
    CLOUDY = 2
    WATER = 3
    INCANDESCENT = 4
    FLUORESCENT = 5
    MANUAL = 6
    WARM_FLUORESCENT = 7
    TWILIGHT = 8
    SHADE = 9
    IRCUT = 10


@dataclass(frozen=True)
class WhiteBalance:
    mode: int
    temperature_code: int
    extended: bool = False

    def __post_init__(self):
        _uint(self.mode, 8, "mode")
        if not isinstance(self.extended, bool):
            raise TypeError("extended must be bool")
        _uint(self.temperature_code, 32 if self.extended else 8, "temperature_code")

    @property
    def kelvin(self) -> int:
        """Stored WB temperature; in AUTO it is not a measured scene temperature."""
        return self.temperature_code * 100

    @property
    def known_mode(self) -> WhiteBalanceMode | None:
        return _known(WhiteBalanceMode, self.mode)

    @classmethod
    def manual(cls, kelvin: int) -> "WhiteBalance":
        _uint(kelvin, 16, "kelvin")
        if not 2000 <= kelvin <= 10000 or kelvin % 100:
            raise ValueError("manual WB must be 2000..10000 K in steps of 100 K")
        return cls(WhiteBalanceMode.MANUAL, kelvin // 100)


def decode_white_balance(reply: bytes) -> WhiteBalance:
    # Retail GET returns {u8mode,u32ct}; SET supports both this and legacy u8ct.
    body = _reply(reply, (2, 5))
    if len(body) == 5:
        return WhiteBalance(body[0], struct.unpack_from("<I", body, 1)[0], True)
    return WhiteBalance(*body)


def encode_white_balance(value: WhiteBalance) -> bytes:
    if not isinstance(value, WhiteBalance):
        raise TypeError("value must be WhiteBalance")
    if (
        value.mode == WhiteBalanceMode.MANUAL
        and not 20 <= value.temperature_code <= 100
    ):
        raise ValueError("manual WB temperature code must be 20..100")
    if value.extended:
        return struct.pack("<BI", value.mode, value.temperature_code)
    return bytes((value.mode, value.temperature_code))


class ColorMode(IntEnum):
    NONE = 0
    D_LOG = 23
    RAW_D_CINELIKE = 44
    RAW_D_LOG = 45
    CINE_LIKE = 52
    HLG = 60
    D_LOG_M = 61
    NORMAL_ENHANCE = 63


@dataclass(frozen=True)
class ColorSetting:
    mode: int

    def __post_init__(self):
        _uint(self.mode, 8, "mode")

    @property
    def known_mode(self) -> ColorMode | None:
        return _known(ColorMode, self.mode)


def decode_color(reply: bytes) -> ColorSetting:
    return ColorSetting(_reply(reply, 1)[0])


def encode_color(value: ColorSetting) -> bytes:
    if not isinstance(value, ColorSetting):
        raise TypeError("value must be ColorSetting")
    return bytes((value.mode,))


_ISO = (
    None,
    None,
    50,
    100,
    200,
    400,
    800,
    1600,
    3200,
    6400,
    12800,
    25600,
    51200,
    102400,
    19200,
    32000,
    9600,
    16000,
    204800,
    409600,
    819200,
)


@dataclass(frozen=True)
class IsoSetting:
    code: int

    def __post_init__(self):
        _uint(self.code, 8, "code")

    @property
    def iso(self) -> int | None:
        return _ISO[self.code] if self.code < len(_ISO) else None

    @property
    def automatic(self) -> bool:
        return self.code in (0, 1)

    @classmethod
    def manual(cls, iso: int) -> "IsoSetting":
        _uint(iso, 32, "iso")
        try:
            return cls(_ISO.index(iso))
        except ValueError as exc:
            raise ValueError("ISO has no known protocol code") from exc


def decode_iso(reply: bytes) -> IsoSetting:
    return IsoSetting(_reply(reply, 1)[0])


def encode_iso(value: IsoSetting) -> bytes:
    if not isinstance(value, IsoSetting):
        raise TypeError("value must be IsoSetting")
    return bytes((value.code,))


@dataclass(frozen=True)
class ShutterValue:
    value: int
    reciprocal: bool
    decimal: int = 0

    def __post_init__(self):
        _uint(self.value, 15, "value")
        _uint(self.decimal, 8, "decimal")
        if not isinstance(self.reciprocal, bool):
            raise TypeError("reciprocal must be bool")

    @property
    def seconds(self) -> Fraction | None:
        # Retail shutter_value_2str prints "%d.%d" or "1/%d.%d".
        amount = Fraction(f"{self.value}.{self.decimal}")
        return (1 / amount if self.reciprocal else amount) if amount else None


@dataclass(frozen=True)
class ShutterSpeed:
    mode: int
    shutter: ShutterValue
    shutter_type: int | None = None
    parameter_flags: int | None = None

    def __post_init__(self):
        _uint(self.mode, 8, "mode")
        if not isinstance(self.shutter, ShutterValue):
            raise TypeError("shutter must be ShutterValue")
        if (self.shutter_type is None) != (self.parameter_flags is None):
            raise ValueError(
                "V2 shutter_type and parameter_flags must be supplied together"
            )
        if self.shutter_type is not None:
            _uint(self.shutter_type, 8, "shutter_type")
            _uint(self.parameter_flags, 16, "parameter_flags")


def _shutter_value(body: bytes) -> ShutterValue:
    packed, decimal = struct.unpack("<HB", body)
    return ShutterValue(packed & 0x7FFF, bool(packed & 0x8000), decimal)


def decode_shutter(reply: bytes) -> ShutterSpeed:
    body = _reply(reply, (4, 7))
    extra = (body[4], struct.unpack_from("<H", body, 5)[0]) if len(body) == 7 else ()
    return ShutterSpeed(body[0], _shutter_value(body[1:4]), *extra)


def encode_shutter(value: ShutterSpeed) -> bytes:
    if not isinstance(value, ShutterSpeed):
        raise TypeError("value must be ShutterSpeed")
    result = struct.pack(
        "<BHB",
        value.mode,
        value.shutter.value | (int(value.shutter.reciprocal) << 15),
        value.shutter.decimal,
    )
    if value.shutter_type is not None:
        result += struct.pack("<BH", value.shutter_type, value.parameter_flags)
    return result


class LensMode(IntEnum):
    SINGLE_LENS = 0
    PANORAMIC = 1


def encode_lens_mode_query() -> bytes:
    """02/8E GET one parameter, type003F; no mode change."""
    return bytes.fromhex("00013f00")


def encode_lens_mode(value: LensMode) -> bytes:
    """02/8E SET one type003F byte, accepting only an explicit LensMode."""
    if not isinstance(value, LensMode):
        raise TypeError("value must be LensMode")
    return bytes.fromhex("01013f0001") + bytes((value,))


def decode_lens_mode(payload: bytes) -> LensMode:
    """Decode strict status/GET/count/type/length/value from the actual camera."""
    body = _reply(payload, 6)
    if body[:5] != bytes.fromhex("00013f0001"):
        raise ControlDecodeError("Expected GET of one type003F parameter with length1")
    try:
        return LensMode(body[5])
    except ValueError as error:
        raise ControlDecodeError(f"Unknown lens mode 0x{body[5]:02x}") from error


class ModeProfile(IntEnum):
    # Exact OQ101 native mapping, libdcam_duss_protocol_util.so table0x945E0.
    # VIDEO_NORMAL is the protocol name; UI labels depend on the product.
    VIDEO_NORMAL = 1
    VIDEO_PANO = 56
    VIDEO_SEMI_PANO = 60
    DUAL_CAM_PANO = 63


@dataclass(frozen=True)
class ModeProfileSetting:
    profile: int

    def __post_init__(self):
        _uint(self.profile, 8, "profile")


def encode_mode_profile(value: ModeProfileSetting) -> bytes:
    if not isinstance(value, ModeProfileSetting):
        raise TypeError("value must be ModeProfileSetting")
    return bytes((value.profile,))


def encode_camera_info_query(
    message_id: int, *, key: int = 0x4261, subscribe_id: int = 0xFF
) -> bytes:
    """02/EB SINGLE_ACQUISITION; no ongoing subscription is created."""
    return struct.pack(
        "<BBBHHHHH",
        6,
        _uint(subscribe_id, 8, "subscribe_id"),
        1,
        _uint(key, 16, "key"),
        0,
        1,
        1,
        _uint(message_id, 16, "message_id"),
    )


def encode_profile_query(*, key: int = 0x4261, subscribe_id: int = 0xFF) -> bytes:
    return encode_camera_info_query(7, key=key, subscribe_id=subscribe_id)


@dataclass(frozen=True)
class SubscriptionGroup:
    group_id: int
    data: bytes


@dataclass(frozen=True)
class SubscriptionReply:
    subscribe_id: int
    flags: int
    key: int
    groups: tuple[SubscriptionGroup, ...]


def decode_subscription(reply: bytes) -> SubscriptionReply:
    if not isinstance(reply, (bytes, bytearray, memoryview)):
        raise TypeError("reply must be bytes-like")
    reply = bytes(reply)
    if not reply:
        raise ControlDecodeError("Empty reply: missing status")
    if reply[0]:
        raise CameraReplyError(reply[0], reply)
    if len(reply) < 5:
        raise ControlDecodeError("Truncated subscription header")
    sid, flags, key = struct.unpack_from("<BBH", reply, 1)
    groups = []
    offset = 5
    while offset < len(reply):
        if offset + 2 > len(reply):
            raise ControlDecodeError("Truncated subscription group header")
        header = struct.unpack_from("<H", reply, offset)[0]
        offset += 2
        size = header >> 6
        if offset + size > len(reply):
            raise ControlDecodeError("Truncated subscription group body")
        groups.append(SubscriptionGroup(header & 63, reply[offset : offset + size]))
        offset += size
    return SubscriptionReply(sid, flags, key, tuple(groups))


def _single_info(
    reply: bytes, message_id: int, size: int, key: int | None, subscribe_id: int | None
) -> bytes:
    result = decode_subscription(reply)
    if key is not None and result.key != key:
        raise ControlDecodeError("Subscription key does not match request")
    if subscribe_id is not None and result.subscribe_id != subscribe_id:
        raise ControlDecodeError("Subscription ID does not match request")
    if result.flags != 1 or len(result.groups) != 1:
        raise ControlDecodeError("Expected one uncombined, message-ID-enabled group")
    group = result.groups[0]
    if group.group_id != message_id // 1000 or len(group.data) != size + 2:
        raise ControlDecodeError("Unexpected subscription group or body length")
    if struct.unpack_from("<H", group.data)[0] != message_id:
        raise ControlDecodeError("Camera info message ID does not match request")
    return group.data[2:]


def decode_mode_profile(
    reply: bytes, *, key: int | None = 0x4261, subscribe_id: int | None = 0xFF
) -> ModeProfileSetting:
    return ModeProfileSetting(_single_info(reply, 7, 1, key, subscribe_id)[0])


@dataclass(frozen=True)
class CaptureParameters:
    """EB6001 current settings, not a camera capability declaration."""

    video_format: VideoFormat
    exposure_mode: ExposureModeSetting
    white_balance: WhiteBalance
    color: ColorSetting
    iso: IsoSetting
    shutter: ShutterValue
    raw: bytes


def decode_capture_parameters(
    reply: bytes, *, key: int | None = 0x4261, subscribe_id: int | None = 0xFF
) -> CaptureParameters:
    body = _single_info(reply, 6001, 104, key, subscribe_id)
    # Offsets verified by compiling the extracted packed C header with offsetof.
    return CaptureParameters(
        VideoFormat(*body[13:18]),
        ExposureModeSetting(*body[20:22]),
        WhiteBalance(*body[23:25]),
        ColorSetting(body[31]),
        IsoSetting(body[5]),
        _shutter_value(body[2:5]),
        body,
    )


def decode_set_ack(reply: bytes) -> None:
    _reply(reply, 0)


@dataclass(frozen=True)
class ColorCapabilities:
    """An explicit supplied capability list; enum existence implies no support."""

    modes: tuple[int, ...]

    def __post_init__(self):
        for value in self.modes:
            _uint(value, 8, "color mode")

    def supports(self, mode: int) -> bool:
        return _uint(mode, 8, "mode") in self.modes

    @property
    def supports_d_log_m(self) -> bool:
        return self.supports(ColorMode.D_LOG_M)

    @property
    def supports_hlg(self) -> bool:
        return self.supports(ColorMode.HLG)

    # HDR recording is a separate mode/capability. No guessed setter or inference
    # from HLG/D-Log M is exposed here.
