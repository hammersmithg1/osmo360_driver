"""Read one *candidate* stabilization parameter without changing the camera.

Action 6 Mimo captures supply candidate enum labels for parameter0008. The
retail Osmo360 library with SHA256e7fc710965f10c4f8ffb5e1860e923c438fecbb752db30c320171730ec459b2d
independently identifies PID0008 as main EIS property01010063, and PID003E as
the separate liveview EIS property01010064. This probe reads only PID0008:
its zero reply does not certify liveview EIS-off or fixed image geometry.
This diagnostic has no SET, retry, connect, mode transition or recovery path.
Use an already-owned, connected Camera.

Primary evidence for the candidate and the generic GET envelope:
https://opencapture.org/openpocketcine/docs/devices/action-6/controls/
https://github.com/erik-sutton95/OpenCine/blob/main/Sources/OpenCineDJI/Commands.swift
"""

import math

from osmo360_camera.controls import CameraCommand, CameraReplyError, ControlDecodeError


_REQUEST = bytes.fromhex('00010800')
_REPLY_PREFIX = bytes.fromhex('000001080001')
_ACTION6_LABELS = {
    0: 'Off',
    1: 'RockSteady',
    2: 'HorizonSteady',
    3: 'RockSteady+',
    4: 'HorizonBalancing',
}


def decode_candidate_reply(payload):
    """Require status00, GET, count1, echoed PID0008, length1 and one value.

    Unknown enum bytes remain numbers; enum recognition itself is not evidence
    that this parameter has the Action 6 meaning on Osmo 360.
    """
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError('reply must be bytes-like')
    payload = bytes(payload)
    if not payload:
        raise ControlDecodeError('Missing camera status')
    if payload[0]:
        raise CameraReplyError(payload[0], payload)
    if len(payload) != 7 or payload[:6] != _REPLY_PREFIX:
        raise ControlDecodeError('Expected GET of one PID0008 parameter with length1')
    return payload[6]


def probe_stabilization_candidate(camera, *, timeout=2.0):
    """Issue exactly one readonly GET and return JSON-safe diagnostic evidence.

    The transport still verifies DUML sequence/command/endpoints. This decoder
    additionally verifies the parameter echo so another 02/8E reply cannot be
    confused with this GET. Failures are reported without issuing another I/O.
    """
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
        raise TypeError('timeout must be positive finite seconds')
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout must be positive finite seconds')
    result = {
        'diagnostic': 'candidate_stabilization_parameter',
        'command': '02/8e',
        'parameter_id': 8,
        'request_hex': _REQUEST.hex(),
        'response_hex': None,
        'camera_status': None,
        'wire_reply_valid': False,
        'parameter_value': None,
        'candidate_action6_label': None,
        'candidate_semantics_source':
            'https://opencapture.org/openpocketcine/docs/devices/action-6/controls/',
        'model_semantics_verified': False,
        'video_effect_verified': False,
        'onboard_stabilization_disabled_verified': False,
    }
    try:
        with camera._control_lock:
            payload = camera._get(CameraCommand.PARAM_OPTION, _REQUEST, timeout=timeout)
        if isinstance(payload, (bytes, bytearray, memoryview)):
            result['response_hex'] = bytes(payload).hex()
            result['camera_status'] = payload[0] if payload else None
        value = decode_candidate_reply(payload)
        result.update(wire_reply_valid=True, parameter_value=value,
                      candidate_action6_label=_ACTION6_LABELS.get(value))
    except Exception as error:
        result['error'] = {'type': type(error).__name__, 'message': str(error)}
    return result
