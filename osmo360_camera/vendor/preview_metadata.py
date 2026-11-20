"""Decode native preview SEI while preserving versioned raw metadata.

Captured protocol boundaries are covered by tests/test_preview_metadata.py.
"""
import re

from ..calibration import CalibrationDecodeError, decode_factory_calibration

START_CODE = re.compile(rb"\x00\x00\x00?\x01")


def nals(data):
    previous = None
    for match in START_CODE.finditer(data):
        if previous is not None:
            yield previous.start(), data[previous.end():match.start()]
        previous = match
    if previous is not None:
        yield previous.start(), data[previous.end():]


def records_from_video(data):
    """Одна запись на SEI кадра, без удаления метаданных из самого видео."""
    import struct
    for offset, nal in nals(data):
        if not nal or nal[0] & 31 != 6:
            continue
        escaped = nal[1:]
        rbsp = escaped.replace(b"\x00\x00\x03", b"\x00\x00")
        pos, kind = 0, 0
        while pos < len(rbsp) and rbsp[pos] == 255:
            kind += 255
            pos += 1
        if pos >= len(rbsp):
            continue
        kind += rbsp[pos]
        pos += 1
        declared_size = 0
        while pos < len(rbsp) and rbsp[pos] == 255:
            declared_size += 255
            pos += 1
        if pos >= len(rbsp) or kind != 245:
            continue
        declared_size += rbsp[pos]
        pos += 1
        if not rbsp.endswith(b"\x80"):
            raise ValueError(f"Нет RBSP stop bit в SEI по смещению {offset}")
        payload = rbsp[pos:-1]
        # В этом потоке DJI считает SEI size ДО снятия emulation-prevention.
        # Из-за этого FFmpeg пишет SEI truncated, хотя TLV внутри целы.
        if declared_size not in (len(payload), len(escaped) - pos - 1):
            raise ValueError(f"Неизвестный формат длины SEI по смещению {offset}")
        record = {"nal_offset": offset, "sei_type": kind,
                  "sei_declared_size": declared_size, "raw_tlv_hex": payload.hex()}
        cursor = 0
        while cursor < len(payload):
            if cursor + 4 > len(payload):
                raise ValueError("Неполный заголовок TLV")
            tag, length = struct.unpack_from("<HH", payload, cursor)
            value = payload[cursor + 4:cursor + 4 + length]
            if len(value) != length:
                raise ValueError(f"Неполный TLV {tag}")
            if tag == 13:
                # A repeated tag uses its last complete value. Do not combine
                # an earlier successful decode with a later malformed body.
                for key in tuple(record):
                    if key.startswith('calibration_'):
                        del record[key]
            if len(value) < 5:
                if tag != 13:
                    raise ValueError(f"Неполный TLV {tag}")
                record['calibration_value_hex'] = value.hex()
                record['calibration_decode_error'] = 'Truncated factory calibration envelope'
                cursor += 4 + length
                continue
            version = value[0]
            body_length = struct.unpack_from("<I", value, 1)[0]
            body = value[5:]
            if tag == 13:
                # Calibration is opaque here. A new/unknown variant must not
                # prevent extraction of an independently valid frame timestamp.
                record.update({"calibration_hex": body.hex(),
                               "calibration_version": version,
                               "calibration_declared_body_length": body_length,
                               "calibration_body_length": len(body),
                               "calibration_value_hex": value.hex()})
                if body_length < len(body):
                    record["calibration_extension_hex"] = body[body_length:].hex()
                # Calibration is diagnostic: an unknown version or invalid
                # optical value must never discard an otherwise valid video
                # AU or hide its independent motion timestamp.
                try:
                    record['calibration_decoded'] = decode_factory_calibration(value)
                except CalibrationDecodeError as error:
                    record['calibration_decode_error'] = str(error)
            elif tag == 12:
                record.update({"motion_version": version,
                               "motion_declared_body_length": body_length,
                               "motion_body_length": len(body)})
                if version != 1:
                    record["motion_decode_error"] = "unsupported_version"
                    cursor += 4 + length
                    continue
                if body_length > len(body):
                    raise ValueError("Объявленное тело TLV 12 неполное")
                # The native packed structure contains a union of v1/v2.
                # Current firmware transmits its larger storage with version1
                # and the v1 declared length. Do not select v2 from size alone.
                # With no fusion samples the native producer keeps len=0 but
                # still writes the image clock into its zeroed union storage.
                header_only = (body_length == 0 and len(body) == 68
                               and not any(body[:12]) and not any(body[28:]))
                prefix = body[:40] if header_only else body[:body_length]
                if len(prefix) < 40:
                    raise ValueError("Слишком короткий блок движения")
                count = struct.unpack_from("<I", prefix, 36)[0]
                if len(prefix) != 40 + 16 * count:
                    raise ValueError("Длина массива кватернионов не соответствует заголовку")
                if body_length < len(body):
                    record["motion_extension_hex"] = body[body_length:].hex()
                record.update({
                    "imu_clock_us_raw": struct.unpack_from("<Q", body, 4)[0],
                    "frame_clock_ns_raw": struct.unpack_from("<Q", body, 20)[0],
                    "header_hex": body[:40].hex(),
                    "unknown_u32_0": struct.unpack_from("<I", body, 0)[0],
                    "unknown_u32_12": struct.unpack_from("<I", body, 12)[0],
                    "unknown_u32_16": struct.unpack_from("<I", body, 16)[0],
                    "unknown_u32_28": struct.unpack_from("<I", body, 28)[0],
                    "unknown_float_32": struct.unpack_from("<f", body, 32)[0],
                    "sample_count": count,
                    "quaternions": [list(struct.unpack_from("<4f", body, 40 + 16 * i))
                                    for i in range(count)],
                })
                if header_only:
                    record.update({"motion_header_only": True,
                                   "motion_samples_available": False,
                                   "imu_clock_available": False,
                                   "frame_clock_available": bool(record["frame_clock_ns_raw"]),
                                   "motion_body_hex": body.hex()})
                    # A zero IMU clock is not an anchor. Keep this record out of
                    # the orientation export's quaternion/anchor path.
                    del record["quaternions"]
                    if not record["frame_clock_available"]:
                        del record["frame_clock_ns_raw"]
            cursor += 4 + length
        yield record
