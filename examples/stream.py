#!/usr/bin/env python3
"""Видео + raw IMU по USB, с необязательными циклами сна. Завершение оставляет сон."""

import argparse
import math
import threading
import time


def capture(stream, seconds):
    """Два читателя: обработка кадра не должна задерживать IMU около 8 кГц."""
    stop = threading.Event()
    errors = []
    counts = dict(video=0, imu=0)

    def consume_imu():
        try:
            while not stop.is_set():
                try:
                    sample = stream.read_imu(timeout=0.1)
                except TimeoutError:
                    continue
                # Передайте в SLAM/ROS: время камеры/ns, rad/s, m/s².
                # Медленную работу выносите в свою ограниченную очередь.
                _ = sample.timestamp_ns, sample.gyro, sample.accel
                counts["imu"] += 1
        except Exception as error:
            errors.append(error)
            stop.set()

    worker = threading.Thread(target=consume_imu, name="slam-imu", daemon=True)
    worker.start()
    began = time.monotonic()
    try:
        while time.monotonic() - began < seconds and not stop.is_set():
            frame = stream.read_video(timeout=3)
            # Передайте в SLAM/ROS: (1024, 2048, 3), BGR; время камеры/ns.
            _ = frame.bgr, frame.timestamp_ns
            counts["video"] += 1
    finally:
        # До любого lifecycle-перехода пользовательские читатели должны выйти.
        stop.set()
        worker.join(2)
        if worker.is_alive():
            raise RuntimeError(
                "Читатель IMU не остановился; переход состояния запрещён"
            )
    if errors:
        raise errors[0]
    stats = stream.stats
    print(
        "Получено:",
        counts,
        "IMU queue drops:",
        stats["imu"]["queue_drops"],
        "video drops:",
        stats["preview"]["video"]["dropped_frames"],
        flush=True,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--seconds",
        type=float,
        default=10,
        help="Длительность каждого захвата, секунды",
    )
    parser.add_argument("--sleep-seconds", type=float, default=1)
    parser.add_argument(
        "--cycles",
        type=int,
        default=1,
        help="Число промежуточных циклов сна/пробуждения; 0 — только захват",
    )
    parser.add_argument(
        "--retain-ecm",
        action="store_true",
        help="Проверенный Linux-путь быстрого сна с сохранением USB",
    )
    args = parser.parse_args(argv)
    if any(not math.isfinite(v) or v <= 0 for v in (args.seconds, args.sleep_seconds)):
        parser.error("Длительности должны быть конечными положительными числами")
    if args.cycles < 0:
        parser.error("cycles должен быть неотрицательным")

    from osmo360_camera import Camera

    with Camera().connect(timeout=30) as camera:
        with camera.usb_stream(
            fast_sleep=args.retain_ecm, retain_ecm=args.retain_ecm
        ) as stream:
            for index in range(args.cycles + 1):
                try:
                    capture(stream, args.seconds)
                except KeyboardInterrupt:
                    break
                if index == args.cycles:
                    break
                began = time.monotonic()
                stream.sleep()
                print(f"Сон: {time.monotonic() - began:.2f} с", flush=True)
                time.sleep(args.sleep_seconds)
                began = time.monotonic()
                stream.wake()
                print(
                    f"Пробуждение до видео и IMU: {time.monotonic() - began:.2f} с",
                    flush=True,
                )
            if stream.state == "AWAKE":
                stream.sleep()
            print("Камера спит; закрывается хостовая сессия.", flush=True)


if __name__ == "__main__":
    main()
