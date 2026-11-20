"""Export native factory calibration from metadata/manifest JSON or original H264.

This offline command never opens a camera. It preserves the original coordinate
system and all coefficients. An optional resize seed is explicitly unverified;
it is not a Kalibr-equidistant conversion or a camera-to-IMU calibration.
"""

import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import struct

from osmo360_camera.calibration import decode_factory_calibration, uniform_resize_seed


def _find_value(document, depth=0):
    if depth > 12:
        return None
    if isinstance(document, dict):
        original = document.get('calibration_value_hex')
        if isinstance(original, str):
            return bytes.fromhex(original)
        if (document.get('source') == 'original_in_band_sei_type245_tag13'
                and isinstance(document.get('original_value_hex'), str)):
            return bytes.fromhex(document['original_value_hex'])
        # Older status snapshots retained the exact body plus its envelope
        # fields, which are sufficient to reconstruct the original tag value.
        if (isinstance(document.get('calibration_hex'), str)
                and type(document.get('calibration_version')) is int
                and type(document.get('calibration_declared_body_length')) is int):
            return struct.pack('<BI', document['calibration_version'],
                               document['calibration_declared_body_length']) + bytes.fromhex(document['calibration_hex'])
        for child in document.values():
            found = _find_value(child, depth + 1)
            if found is not None:
                return found
    elif isinstance(document, list):
        for child in document:
            found = _find_value(child, depth + 1)
            if found is not None:
                return found
    return None


def extract_calibration(path, format='auto'):
    """Return a decoded tag value and bounded, explicit input provenance."""
    path = Path(path)
    if format == 'auto':
        format = 'h264' if path.suffix.lower() in ('.h264', '.264') else 'json'
    if format == 'h264':
        from osmo360_camera.vendor.preview_metadata import records_from_video
        prefix = bytearray()
        with path.open('rb') as stream:
            for _ in range(16):  # Only a16MiB prefix; never load a long recording.
                chunk = stream.read(1024 * 1024)
                prefix.extend(chunk)
                value = None
                try:
                    for row in records_from_video(bytes(prefix)):
                        value = _find_value(row)
                        if value is not None:
                            break
                except ValueError:
                    # The prefix can end inside a SEI NAL. Add more bytes;
                    # a complete unsupported calibration is decoded below.
                    if not chunk:
                        raise
                if value is not None:
                    return decode_factory_calibration(value), dict(
                        format=format, path=str(path), prefix_bytes=len(prefix),
                        prefix_sha256=hashlib.sha256(prefix).hexdigest())
                if not chunk:
                    break
        raise ValueError('No usable factory tag13 in the first16MiB; try the session manifest JSON')
    if format != 'json':
        raise ValueError('Format must be auto, json or h264')
    with path.open('rb') as stream:
        raw = stream.read(16 * 1024 * 1024 + 1)
    if len(raw) > 16 * 1024 * 1024:
        raise ValueError('JSON input exceeds16MiB; use the small session manifest/status file')
    text = raw.decode('utf-8')
    try:
        documents = [json.loads(text)]
    except json.JSONDecodeError:
        # AU-index JSONL starts with the initial metadata record. Return as
        # soon as it is found rather than parsing every frame into memory.
        documents = (json.loads(line) for line in text.splitlines() if line.strip())
    for document in documents:
        value = _find_value(document)
        if value is not None:
            return decode_factory_calibration(value), dict(
                format='json_or_jsonl', path=str(path), bytes=len(raw),
                sha256=hashlib.sha256(raw).hexdigest())
    raise ValueError('No original factory tag13 value in JSON')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--format', choices=('auto', 'json', 'h264'), default='auto')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--resize-seed', type=Path,
                        help='Optional separate, unverified full-frame resize seed JSON')
    parser.add_argument('--lens-width', type=int, default=1024)
    parser.add_argument('--lens-height', type=int, default=1024)
    args = parser.parse_args(argv)
    if args.output.exists() or (args.resize_seed is not None and args.resize_seed.exists()):
        parser.error('Output files already exist')
    if args.resize_seed is not None and args.output.resolve() == args.resize_seed.resolve():
        parser.error('Native calibration and seed must use different output files')
    try:
        calibration, provenance = extract_calibration(args.input, args.format)
        seed = (uniform_resize_seed(calibration, (args.lens_width, args.lens_height))
                if args.resize_seed is not None else None)
    except (OSError, ValueError, struct.error) as error:
        parser.error(str(error))
    calibration['input_provenance'] = provenance
    outputs = [(args.output, calibration)]
    if seed is not None:
        outputs.append((args.resize_seed, seed))
    created = []
    try:
        serialized = [(path, json.dumps(document, indent=2, allow_nan=False) + '\n')
                      for path, document in outputs]
        with ExitStack() as stack:
            streams = []
            # Acquire both exclusive outputs before writing either document.
            for path, text in serialized:
                stream = stack.enter_context(path.open('x'))
                identity = os.fstat(stream.fileno())
                created.append((path, (identity.st_dev, identity.st_ino)))
                streams.append((stream, text))
            for stream, text in streams:
                stream.write(text)
    except (OSError, ValueError) as error:
        retained = []
        for path, identity in reversed(created):
            try:
                info = path.lstat()
                if (info.st_dev, info.st_ino) == identity:
                    path.unlink()
                else:
                    retained.append(str(path))
            except FileNotFoundError:
                pass
            except OSError:
                retained.append(str(path))
        suffix = ' (could not clean created paths: ' + ', '.join(retained) + ')' if retained else ''
        parser.error('Cannot write calibration outputs: ' + str(error) + suffix)
    print(json.dumps(dict(output=str(args.output), version=calibration['version'],
                         calibration_image_size=calibration['calibration_image_size'],
                         stream_mapping_verified=False)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
