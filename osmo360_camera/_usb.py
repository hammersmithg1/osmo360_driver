"""Minimal libusb-1.0 ctypes binding shared by control and IMU transports."""

import ctypes as C
import ctypes.util


def load_usb():
    path = ctypes.util.find_library("usb-1.0")
    if not path:
        raise RuntimeError(
            "Не найдена системная библиотека libusb-1.0. Установите её средствами вашей ОС."
        )
    usb = C.CDLL(path)
    signatures = {
        "init": ([C.POINTER(C.c_void_p)], C.c_int),
        "exit": ([C.c_void_p], None),
        "get_device_list": (
            [C.c_void_p, C.POINTER(C.POINTER(C.c_void_p))],
            C.c_ssize_t,
        ),
        "free_device_list": ([C.POINTER(C.c_void_p), C.c_int], None),
        "get_device_descriptor": ([C.c_void_p, C.c_void_p], C.c_int),
        "open": ([C.c_void_p, C.POINTER(C.c_void_p)], C.c_int),
        "close": ([C.c_void_p], None),
        "claim_interface": ([C.c_void_p, C.c_int], C.c_int),
        "release_interface": ([C.c_void_p, C.c_int], C.c_int),
        "bulk_transfer": (
            [C.c_void_p, C.c_ubyte, C.c_void_p, C.c_int, C.POINTER(C.c_int), C.c_uint],
            C.c_int,
        ),
        "error_name": ([C.c_int], C.c_char_p),
    }
    for name, (args, result) in signatures.items():
        function = getattr(usb, "libusb_" + name)
        function.argtypes, function.restype = args, result
    return usb


def check(usb, code, operation):
    if code < 0:
        raise RuntimeError(
            f"{operation}: {usb.libusb_error_name(code).decode()} ({code})"
        )
    return code
