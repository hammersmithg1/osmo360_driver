"""Decode the camera's original, versioned SEI tag13 factory calibration.

The layout comes from the camera's installed duml_wl_exteninfo.h, SHA256
b91d33e01871062ce7c630a70fcb6dfaf05de35624ea73606637707626e267e6.
Its packed v1/v2 sizes and offsets were independently checked with C offsetof.
This module does no camera I/O, image warping, time correction or calibration.
Reported factory fields are distinct from a verified model of the USB image.
"""

import hashlib
import math
import struct


class CalibrationDecodeError(ValueError):
    """An unknown or malformed factory calibration must remain opaque."""


HEADER_SHA256 = 'b91d33e01871062ce7c630a70fcb6dfaf05de35624ea73606637707626e267e6'
_BODY_SIZES = {1: 219, 2: 287}
_INTRINSIC_FIELDS = ('fx', 'fy', 'cx', 'cy', *(f'k{i}' for i in range(1, 10)),
                     'p1', 'p2', 'xi', 's')
_STREAM_TYPES = {0: 'normal', 1: 'dual_fisheye', 2: 'dual_hemisphere'}
_TAIL = struct.Struct('<QHf8I2fI')
_MODEL_SOURCE = 'https://github.com/AdrianEddy/telemetry-parser/blob/master/src/dji/rig.rs'


def _finite(values, name):
    if not all(math.isfinite(value) for value in values):
        raise CalibrationDecodeError('Non-finite ' + name)


def _intrinsic(body, offset):
    values = struct.unpack_from('<17f', body, offset)
    _finite(values, 'intrinsic parameters')
    result = dict(zip(_INTRINSIC_FIELDS, values))
    if result['fx'] <= 0 or result['fy'] <= 0:
        raise CalibrationDecodeError('Factory focal lengths must be positive')
    return result


def _extrinsic(body, offset, version):
    count = 3 if version == 1 else 11
    values = struct.unpack_from(f'<{count}f', body, offset)
    _finite(values, 'extrinsic parameters')
    result = dict(zip(('yaw', 'pitch', 'roll'), values[:3]))
    if version == 2:
        # The native quaternion type explicitly declares w,x,y,z. The header
        # does not specify the frames/direction of either transformation.
        result['cam_imu_extri_q_wxyz'] = list(values[3:7])
        result['cam_extri_q_wxyz'] = list(values[7:11])
    return result


def decode_factory_calibration(value):
    """Decode the complete tag13 value (u8 version, u32 size, packed body).

    Current firmware sends the full v1/v2 union even for version1: 287 body
    bytes, with only219 declared. Version and declared length select the
    layout; the unused68 bytes never become v2 quaternions or temperature.
    Unknown formats fail here, so callers can preserve raw bytes and continue
    video/IMU capture with a calibration warning.
    """
    if not isinstance(value, (bytes, bytearray, memoryview)):
        raise TypeError('Calibration value must be bytes-like')
    value = bytes(value)
    if len(value) < 5:
        raise CalibrationDecodeError('Truncated factory calibration envelope')
    version, declared = struct.unpack_from('<BI', value)
    expected = _BODY_SIZES.get(version)
    if expected is None:
        raise CalibrationDecodeError(f'Unsupported factory calibration version {version}')
    body = value[5:]
    allowed = (219, 287) if version == 1 else (287,)
    if declared != expected or len(body) not in allowed:
        raise CalibrationDecodeError('Factory calibration version/declared/actual lengths disagree')
    prefix = body[:declared]
    stream_type = prefix[0]
    extrinsic_size = 12 if version == 1 else 44
    lenses = []
    for index in range(2):
        lenses.append(dict(native_id=f'intrinsic0{index}',
            intrinsic=_intrinsic(prefix, 1 + 68 * index),
            extrinsic=_extrinsic(prefix, 137 + extrinsic_size * index, version)))
    tail_offset = 137 + 2 * extrinsic_size
    fields = _TAIL.unpack_from(prefix, tail_offset)
    (readout, imu_rate, fps, itd, lro, sensor_w, sensor_h, rs_w, rs_h,
     pano_w, pano_h, calibration_w, calibration_h, lens_type) = fields
    _finite((fps, calibration_w, calibration_h), 'frame rate/calibration dimensions')
    if calibration_w <= 0 or calibration_h <= 0:
        raise CalibrationDecodeError('Factory calibration dimensions must be positive')
    temperature = struct.unpack_from('<f', prefix, 283)[0] if version == 2 else None
    if temperature is not None:
        _finite((temperature,), 'temperature')
    # This is an interpretation supported by primary Osmo file decoders,
    # separate from the header-proven binary layout. It does not prove the
    # crop, pixel-centre convention or camera order of a particular USB image.
    compatible = lens_type == 8 and stream_type == 1 and all(
        all(lens['intrinsic'][key] == 0 for key in ('k6', 'k7', 'k8', 'k9', 'xi', 's'))
        for lens in lenses)
    return dict(schema_version=1, source='original_in_band_sei_type245_tag13',
        version=version, declared_body_bytes=declared, actual_body_bytes=len(body),
        native_layout_verified=True, layout_header_sha256=HEADER_SHA256,
        payload_sha256=hashlib.sha256(value).hexdigest(),
        original_value_hex=value.hex(),
        unused_union_tail_hex=body[declared:].hex(),
        stream_frame_type=stream_type, stream_frame_type_name=_STREAM_TYPES.get(stream_type, 'unknown'),
        cam_lens_type=lens_type, calibration_image_size=[calibration_w, calibration_h],
        lenses=lenses,
        timing=dict(read_out_time=readout, imu_sampling_rate=imu_rate, sensor_frame_rate=fps,
                    itd_value_u32=itd, lro_value_u32=lro, units_verified=False),
        sensor_image_size=[sensor_w, sensor_h], rolling_shutter_grid=[rs_w, rs_h],
        panorama_grid=[pano_w, pano_h], temperature=temperature,
        projection_interpretation=dict(candidate_model='dji_kb5_brown_tangential' if compatible else None,
            coefficients_compatible=compatible, evidence=_MODEL_SOURCE,
            verified_for_usb=False),
        stream_mapping_verified=False, camera_imu_transform_verified=False,
        extrinsic_angle_units_verified=False, extrinsic_rotation_convention_verified=False,
        aprilgrid_validated=False)


def uniform_resize_seed(calibration, lens_size=(1024, 1024)):
    """Create an explicitly unverified full-frame resize seed for Aprilgrid.

    This is not an automatic camera_info/Kalibr export. An image crop, lens
    swap or different pixel-centre convention invalidates this assumed affine
    mapping. Original factory parameters always remain in their own space.
    """
    if (len(lens_size) != 2 or any(type(x) is not int or x <= 0 for x in lens_size)):
        raise ValueError('Lens size must contain two positive integers')
    width, height = calibration['calibration_image_size']
    sx, sy = lens_size[0] / width, lens_size[1] / height
    lenses = []
    for index, lens in enumerate(calibration['lenses']):
        original = lens['intrinsic']
        intrinsic = dict(original)
        for name in ('fx', 'cx'):
            intrinsic[name] *= sx
        for name in ('fy', 'cy'):
            intrinsic[name] *= sy
        lenses.append(dict(native_id=lens['native_id'], intrinsic=intrinsic,
            assumed_image_roi_xywh=[index * lens_size[0], 0, *lens_size]))
    return dict(schema_version=1, purpose='unverified_aprilgrid_seed',
        source_payload_sha256=calibration['payload_sha256'],
        assumptions=['full calibration image resized without a crop',
                     'origin-preserving pixel coordinates: u_out=sx*u_cal, v_out=sy*v_cal',
                     'intrinsic00 maps to the left tile; intrinsic01 to the right tile'],
        reference_image_size=list(calibration['calibration_image_size']),
        lens_image_size=list(lens_size), lenses=lenses,
        native_to_image_affine=[[sx, 0, 0], [0, sy, 0], [0, 0, 1]],
        stream_mapping_verified=False, aprilgrid_validated=False,
        kalibr_equi_exact_export=False,
        note='Keep all five radial terms and p1/p2; Kalibr equi has only four radial terms.')
