#!/usr/bin/env python3
"""Разовая установка или удаление служебных программ Osmo 360."""

import argparse
import json
import math


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("install", "remove"))
    parser.add_argument("--timeout", type=float, default=180)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout должен быть конечным положительным числом")

    # Импорт/подключение отложены: импорт примера и --help не обращаются к USB.
    from osmo360_camera import Camera

    with Camera().connect(timeout=30) as camera:
        operation = (
            camera.install_helpers
            if args.action == "install"
            else camera.remove_helpers
        )
        report = operation(timeout=args.timeout)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return report


if __name__ == "__main__":
    main()
