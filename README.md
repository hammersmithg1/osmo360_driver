# Osmo 360 driver + USB SDK

Python-библиотека для управления DJI Osmo 360 и получения видео с сырыми IMU
по одному USB-кабелю. Wi-Fi и Bluetooth не используются; для live-потока
записывать видео на карту не требуется.

Проверено на прошивке **V01.03.08.70 / модуль 10.00.25.29**, macOS и
Raspberry Pi 5 / Ubuntu ARM64. Быстрый сон с сохранением USB проверен на Linux.
Библиотека `osmo360_camera` не зависит от ROS. Пакет `osmo360_driver` добавляет
владельца камеры, ROS 2 Jazzy адаптер, управление состоянием и общие часы видео/IMU.

## Подключение и установка

Камера должна быть включена, с аккумулятором, в режиме **360° → Видео**,
без записи. Оставьте включённой настройку длительного сна без полного
выключения. Закройте DJI Mimo. При выборе USB-режима используйте «Только зарядка».
Для Pi нужен **активный USB-хаб с отдельным питанием** и кабель с линиями данных:
прямое подключение на проверенном стенде вызывало перегрузку питания USB.

Нужны Python 3.10+, libusb и, на Linux, `iproute2`. Декодирование кадров
требует PyAV/NumPy; проверены Python 3.12 / PyAV 11 и Python 3.12+ / PyAV 19.

```sh
# Ubuntu / Raspberry Pi
sudo apt install libusb-1.0-0 iproute2
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[video]'
# macOS: сначала brew install libusb, затем те же venv/pip команды.
```

На Linux операции с USB-сетью требуют соответствующих прав; примеры ниже
запускаются через `sudo`. Библиотека использует временную USB Ethernet-сеть
`192.168.253.0/30`; этот диапазон должен быть свободен. Wi-Fi, default route
и DNS хоста сохраняются. Во время preview соединение работает на USB 2.0.

## Разовая настройка камеры

```sh
sudo .venv/bin/python examples/helpers.py install
# Убрать служебный код и вернуть штатный запуск:
sudo .venv/bin/python examples/helpers.py remove
```

Те же операции доступны в Python:

```python
from osmo360_camera import Camera

with Camera().connect(timeout=30) as cam:
    result = cam.install_helpers()  # Один раз для нового экземпляра.
    # result = cam.remove_helpers()  # Когда библиотека больше не нужна.
```

`install_helpers()` сохраняет проверенные программы в `/data/osmo360-sdk`,
добавляет вызов в штатный загрузочный скрипт, перезагружает камеру и проверяет
автозагрузку. При boot программы только копируются в RAM; поток начинается
по запросу хоста. Повторный вызов с готовой установкой не перезагружает камеру.
Установка выполняется вне активного потока, при `recording_state == 0`.

Если новая версия SDK требует другой helper-профиль, сначала явно выполните
`examples/helpers.py remove` прежней версией SDK, затем обновите SDK и выполните
`examples/helpers.py install`. Обе операции проверяют штатную перезагрузку камеры; работающий owner
не заменяется на месте. Новая камера по-прежнему требует только `install`.

`remove_helpers()` закрывает принадлежащие этой `Camera` потоки, удаляет наш
код, восстанавливает исходные байты и права загрузочного скрипта и проверяет
очистку после reboot. Медиа, настройки и калибровки сохраняются. В overlay
остаётся обычная копия штатного загрузочного файла с исходным содержимым;
это функциональное восстановление, а не побайтовый сброс файловой системы.
Перед обновлением прошивки удалите helpers и отдельно проверьте совместимость
этой overlay-копии с обновлением. Другие версии прошивки SDK не проверены.

## Видео, IMU, сон и пробуждение

```python
from osmo360_camera import Camera

with Camera().connect(timeout=30) as cam:
    with cam.usb_stream(fast_sleep=True, retain_ecm=True) as stream:
        frame = stream.read_video(timeout=3)
        sample = stream.read_imu(timeout=1)
        print(frame.bgr.shape, frame.timestamp_ns)
        print(sample.gyro, sample.accel, sample.timestamp_ns)
        stream.sleep()
        stream.wake()  # Возвращает управление после свежих видео И IMU.
        stream.sleep() # Оставить камеру спящей при завершении процесса.
```

Это иллюстрация API. Для непрерывного потока используйте **отдельного читателя
IMU**: около 8000 образцов/с не должны ждать видеокадр. Пример с двумя читателями,
остановкой перед переходами и счётчиками:

```sh
sudo .venv/bin/python examples/stream.py --retain-ecm --seconds 10 --cycles 1
```

Перед `sleep()`, `wake()` и закрытием контекста остановите и дождитесь своих
читателей. После `wake()` используйте тот же `stream`: внутренние дескрипторы
и сборщики создаются заново. Закрытие контекста само по себе не усыпляет камеру.
При `retain_ecm=True` пробуждение переносит heartbeat исходного USB-плана на
каждый проверенный новый сетевой интерфейс ещё во время ожидания native receipt.
Это поддерживает прежнюю lease с TTL 10 с; видео и DIAG запускаются после
конечного подтверждения и нового ACK. Потеря интерфейса или deadline закрывает
такой временный heartbeat локально, сохраняя защиту неизвестного перехода.
До команды Wake камера резервирует свободный UDP-порт через ядро и передаёт
тот же открытый сокет worker и responder. Хост получает адрес до пробуждения;
эта предварительная квитанция не заменяет конечное подтверждение reset,
проверку исходной lease и новый ACK перед запуском видео и IMU.
Если перед закрытием завершился `sleep()`, следующий процесс может обнаружить
сохранённую спящую сессию и возобновить её.

Потеря C1 при чтении уже принятого статуса Sleep допускает только восстановление
читателя в пределах исходного дедлайна. Сначала SDK проверяет RESULT именно
прежней задачи; новый запрос статуса разрешён лишь после её завершения или
подтверждённого отсутствия. Команда Sleep не повторяется. Сведения о новом
читателе и подтверждении того же перехода доступны в `stream.stats["control"]`;
неизвестная задача или смена владельца по-прежнему блокируют переход.

`retain_ecm=True` требует `fast_sleep=True` и проверен на Linux. Для обычного
пути на macOS используйте `cam.usb_stream()` без этих опций. При завершении
библиотека проверяет восстановление своей USB-конфигурации и исходной
DIAG-маски IMU. Не обходите ошибки cleanup принудительным убийством сборщика
или повтором перехода: исключения и `stream.stats` сохраняют сведения об операции.
Полное выключение и включение из off в API не входят. `connect(timeout=...)`
только ждёт уже включённую камеру; `reboot()` выполняет тёплую перезагрузку.

При остановке IMU подтверждение исходной DIAG-маски ещё не означает, что камера
завершила уже поставленные USB-записи. Тот же сборщик дочитывает их до двух
пустых USB-таймаутов и минимум 0,30 с без данных; общий бюджет — 3 с.
`report.restored` описывает маску, а `report.quiescent` — этот отдельный барьер.
Любая ненулевая исходная маска equipment1 восстанавливается без изменений,
но блокирует USB-переход: редкие чужие логи могут возобновиться после периода
тишины. Неполный пакет или ошибка USB также дают явную ошибку остановки.
Проверка описывает наблюдаемую тишину USB, не подтверждение DSP и не завершение
всех callbacks прошивки; дополнительные маски других equipment она не запрашивает.
До успешного барьера SDK запрещает CLOSE lease, rollback
USB и переход в сон, сохраняя владельца и heartbeat, пока процесс работает.

Барьер нужен из-за подтверждённого дефекта USB-очереди в исследованной
прошивке Osmo: позднее завершение отменённой записи может оставить счётчик
буфера, после чего raw DIAG прекращается при продолжающемся видео. Проверка
SDK использует обычное чтение USB и не зависит от адресов памяти или версии
бинарника камеры. Уже потерянные прошивкой ссылки этим чтением не исправить:
после безопасной остановки требуется явная штатная перезагрузка камеры;
SDK не перезапускает системный daemon и не меняет память прошивки.

Измерено на Pi 5 с постоянной установкой:

| Сценарий | Время |
| --- | ---: |
| Разовая установка, включая reboot и проверку | 35 с |
| Новый `stream.start()` на загрузившейся камере | 17,6–20,7 с |
| `sleep()` с остановкой и дочитыванием IMU | 7,8–8,3 с |
| `wake()` до свежих видео и IMU, 12 повторов | 5,2–5,7 с |

Это время запуска/переходов, а не задержка доставки отдельного кадра.
Абсолютная задержка от экспозиции до получения кадра пока не измерена.

## Форматы данных и интеграция со SLAM

- `frame.bgr`: NumPy-массив **1024×2048×3**, BGR, два соседних изображения
  «рыбий глаз», около 30 fps. Это не сшитая эквидистантная панорама.
- `sample.gyro`: XYZ в **rad/s**; `sample.accel`: XYZ в **m/s²**.
  Нефьюженные значения физических сенсоров после заводской калибровки,
  не кватернионы и не ADC counts. Проверенная частота — около **7960 Гц**.
- `timestamp_ns` у кадра и IMU относится к аппаратным часам камеры.
  `host_receive_monotonic_ns` отмечает приём на хосте. Это не Unix/ROS time;
  преобразование в часы ROS нужно выполнить в драйвере.
- После reboot начинается новая временная шкала. Первый кадр после wake
  может предшествовать первому IMU: начинайте SLAM с пересечения диапазонов.
- Очереди ограничены: медленный потребитель теряет старые данные. Смотрите
  `stream.stats["imu"]["queue_drops"]`, `max_native_interval_ns` и
  `stream.stats["preview"]["video"]["dropped_frames"]`. Потери очереди
  отличаются от потерь до приёма на хосте.

`stream.stats["imu"]["udp_socket"]` показывает запрошенный и фактический буфер
локального сокета. На Linux SDK запрашивает прежние 4 MiB через
`SO_RCVBUFFORCE`, если процесс уже имеет нужные права; глобальные sysctl и права
не меняются. Обычный fallback и его errno также видны в статистике. Linux
показывает удвоенный размер, поэтому успешный запрос даёт 8 MiB. Счётчик
`kernel_drops` поступает из `SO_RXQ_OVFL`; `None` означает отсутствие наблюдения,
а не подтверждённый ноль потерь.

Для SLAM нужны модель обоих объективов, IMU noise/bias, camera↔IMU extrinsics
и временная калибровка. Раздельный API — `cam.usb_preview()`, затем `cam.imu()`;
preview запускайте раньше IMU, поскольку его старт меняет USB-конфигурацию.
У `USBPreview` доступен сырой блок `calibration`, который не является готовым
Kalibr YAML. Точный момент экспозиции в timestamp, rolling shutter и синхронизация
с часами робота не откалиброваны. ROS-адаптер описан ниже; `ros2_control` hardware interface здесь отсутствует.

## Параметры камеры

Чтение: `state()`, `lens_mode()`, `mode_profile()`, `video_format()`,
`capture_parameters()`, `exposure_mode()`, `white_balance()`, `iso()`,
`shutter_setting()`, `color()`, `temperatures()`.

Пример при остановленном потоке и выключенной записи:

```python
from osmo360_camera import (
    Camera, LensMode, ModeProfile, FrameRate, ExposureMode, ExposureModeSetting,
    ShutterSpeed, ShutterValue, ColorMode,
)

with Camera() as cam:
    cam.set_lens_mode(LensMode.PANORAMIC)
    cam.set_mode_profile(ModeProfile.VIDEO_PANO)
    cam.set_frame_rate(FrameRate.FPS25)
    cam.set_exposure_mode(ExposureModeSetting(ExposureMode.MANUAL))
    cam.set_iso(400)
    cam.set_shutter(ShutterSpeed(1, ShutterValue(250, True)))  # 1/250 с
    cam.set_white_balance(5000)  # Kelvin
    cam.set_color(ColorMode.D_LOG_M)
    print(cam.temperatures())   # °C: ntc, qcs8550, ddr, sens, bat
```

Для одного объектива задайте `LensMode.SINGLE_LENS`, затем
`ModeProfile.VIDEO_NORMAL`. Смена объективов может автоматически сменить
профиль и формат; задавайте их явно. Разрешение выбирается через
`video_format()` / `set_video_format(VideoFormat(...))`. FPS настройки съёмки
не повышают частоту USB-preview выше 30 fps; `FPS30` означает 30000/1001,
`FPS30_TRUE` — ровно 30. HDR отдельным методом пока не реализован.

SET-методы проверяют ACK и readback, при ошибке пытаются вернуть прежнее значение.
`SettingApplyError` содержит `previous`, `actual`, `restored`. D-Log M проверен
по readback параметра. Запись на карту запускают только явные
`start_recording()` / `stop_recording()`; это отдельная функция от стриминга.

## Проверка исходников

```sh
.venv/bin/python -m unittest discover -s tests -q
```

Тесты не требуют камеры. Встроенный DUML/preview-код основан на MIT-проекте
`osmo-client`; атрибуция и лицензия сохранены в `osmo360_camera/vendor/NOTICE`
и `LICENSE.osmo-client`.

Диагностика использует стандартный Python logging с логгером `osmo360_camera`.
Файлы `_device/` закреплены побайтовыми хешами установленного helper-профиля:
не форматируйте и не переименовывайте их автоматически, включая исторические имена.
Изменение этих файлов требует согласованного обновления профиля установки.


## ROS 2 Jazzy

В workspace импортируйте репозиторий через vcstool:

```yaml
repositories:
  osmo360_driver:
    type: git
    url: https://github.com/hammersmithg1/osmo360_driver.git
    version: main
```

```sh
vcs import src < dependencies.repos
sudo apt install python3-av python3-numpy python3-opencv libusb-1.0-0 iproute2
colcon build --symlink-install --packages-select osmo360_driver
source install/local_setup.bash
# Linux USB networking needs root; source the same workspace in the root shell.
sudo bash -c 'source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && ros2 launch osmo360_driver osmo360.launch.xml enabled:=true'
```

Node name: `/osmo360`. Default `enabled:=false` does not open USB.
Do not run the standalone node alongside another owner (e.g. the rover camera node).
Firmware, battery, activation and video-mode requirements above still apply.
Before the first ordinary stream in a process, the ROS driver calls the SDK's
idempotent `install_helpers()`: a new camera is prepared once, while a verified
installation and boot cache are reused without reinstalling or rebooting.
An existing sleeping session is discovered and resumed by the SDK without
requiring ordinary camera services to answer first.
Once the control generation is verified, ordinary GET readiness waits within
its existing deadline for transient timeouts/E3. A fresh Record0 is required
before any diagnostic task START and rechecked last; input/SET transitions are
never retried by this readiness wait.

Disabling a stream uses SDK `sleep()` and preserves the session; enabling it
again uses `wake()`. Complete cleanup happens on driver shutdown, configuration
or reboot. Status includes the actual startup phase, command elapsed time and
the installation/autostart proof returned by the SDK.

Video and IMU readers start as soon as the SDK supplies a healthy stream,
before optional settings queries. A failed GET clears the settings snapshot
and appears in `settings_error` with the affected field; it does not stop
video/IMU or fail a recording. Configuration SETs and their readback remain
strict. A later successful refresh clears the warning.

The ROS adapter caches graph subscriptions from a one-second executor timer.
With no subscribers or recording, IMU messages and metadata JSON are not built.
Recording demand is checked on each sample, so a newly accepted session receives
the next sample immediately; native rates and timestamps are preserved. SDK
diagnostic snapshots are copied once per second, outside the video frame lock.
Failed wake reports include the actual prepared backend report and original
exception chain in `stats.report.wake_failure`.

If the SDK reports `RECOVERY_REQUIRED`, ordinary transitions are blocked:
the driver must not label an uncertain session as sleeping or repeat wake.
Stop the driver, power-cycle only the camera, and restart the driver to clear
an unknown pending camera operation. Restarting the Pi is not required.

| Topic | Type / contents |
| --- | --- |
| `/osmo360/image/compressed` | `sensor_msgs/CompressedImage`, dual-fisheye JPEG, quality 92 |
| `/osmo360/video/h264` | `sensor_msgs/CompressedImage`, custom `format=h264`, original Annex-B AU |
| `/osmo360/imu` | `sensor_msgs/Imu`, native SSC axes, rad/s and m/s², all received samples |
| `/osmo360/video/timing` | `std_msgs/UInt64MultiArray`: `[ros_ns, native_ns, host_receive_monotonic_ns, sequence, valid_native]` |
| `/osmo360/imu/timing` | `std_msgs/UInt64MultiArray`: `[ros_ns, native_ns, host_receive_monotonic_ns, native_ticks, accel_status, gyro_status]` |
| `/osmo360/video/metadata` | `std_msgs/String`, original metadata JSON; bytes encoded as `{encoding: hex, data: ...}` |
| `/osmo360/status` | `std_msgs/String`, state, actual settings, queue counters, calibration and clock epoch |

Standard compressed-image consumers should use the JPEG topic. The `h264` topic
is an original-video archive channel; it needs an H.264 decoder, not compressed_image_transport.
JPEG images, their native timing and IMU are the primary input for ROS/SLAM.
A recording may join the H.264 stream between random-access pictures, so its
raw access units can depend on parameter sets and reference pictures sent
before the recording started. Standalone decoding of such an archive is not
verified by this adapter. The rover integration builds session playback from
the independent JPEG images.

SDK video frames can lack native timestamps after missing/unsupported SEI.
These pictures remain visible in the live preview, and their original H.264 AUs
stay in the same archive stream so subsequent P-frames retain their references.
Their archive stamp is the actual host receive time, `valid_native=0`,
`native_ns=0` (unavailable), and video metadata explicitly records
`_rover_timing={valid_native:false, stamp_source:"host_receive"}`. Such frames
are omitted from the synchronized JPEG topic. Normal frames use the shared
camera clock and `valid_native=1` / `stamp_source:"camera_native"`. IMU and the
clock mapping keep running; arrival time never replaces an unknown exposure
time in the SLAM image stream. `timestamp_missing_frames` and SDK bounded SEI
diagnostics expose these quality events; they do not claim lossless transport.
Frame IDs: `osmo360_dual_fisheye` and `osmo360_imu_native`. Orientation is absent
(`orientation_covariance[0] = -1`), gyro/accel covariances unknown (zero).
No uncalibrated body TF, CameraInfo or fused quaternion is invented.

Services (private namespace): `~/stream` (`std_srvs/SetBool`), `~/wake`, `~/sleep`,
`~/reboot`, `~/refresh` (`std_srvs/Trigger`). Success means **queued**; wait for
`~/status.busy` to clear and check `error`/actual settings. Commands are serialized.
Reboot leaves the stream disabled; wake/start enables joint video+IMU.

Requested settings are standard ROS parameters: `exposure_mode` (0/1/4), `iso`
(0=auto or supported ISO), `shutter_denominator` (1…8000), `white_balance`
(0=auto or 2000…10000 K, step 100), `color` (0/60/61/63), `frame_rate`
(protocol codes 2=25 FPS, 3=30000/1001 FPS, 14=30 FPS). A parameter change queues a camera command;
`~/status.settings` is the actual readback. Use atomic parameter setting for groups.
Firmware SET commands are individually verified, not a multi-setting transaction.
The rover UI blocks all changes during a recording session.

`CameraClock` fixes the host-monotonic→Unix mapping, then anchors one shared
native→Unix offset at the first received sample. An embedding host can pass
`host_offset_ns` to share the exact same host clock base with another camera. Video and IMU retain their
native intervals. Start/wake/reboot creates a new epoch; clock reversal stops
the stream. This is arrival-based initial alignment, **not calibrated hardware
synchronization**. Native timestamps, receive times and clock mapping remain
available for offline temporal calibration. The exact relation between the
video exposure clock and SSC timestamps must be verified for SLAM.

For embedded use, compose `OsmoCamera(on_video, on_imu, on_error)` and
`OsmoTopics(node, acquisition_sink=...)` for an IPC recording worker. This callback
receives immutable `imu`/`video` tuples before any ROS message is constructed;
`osmo360_driver.acquisition.materialize_acquisition()` builds the same standard
messages in the worker. Tuples carry the already mapped/native/receive clocks,
raw video bytes, JPEG and immutable metadata JSON; no AV image enters IPC.
Only cached DDS subscribers cause message construction in the capture process.
The earlier `batch_sink` and per-message `sink` remain supported, with archival
priority `acquisition_sink` → `batch_sink` → `sink` to prevent duplicate delivery.
A recorder can store full IMU without depending on subscription QoS. Video and
IMU readers stop and join before any sleep/wake/close transition. Errors retain
the SDK's verified cleanup behavior; there is no forced collector kill or
automatic reboot/retry of an uncertain USB owner.

The SDK helpers and vendor codecs were carried over without modification.
Project code is MIT; upstream attribution remains in `osmo360_camera/vendor/`.
