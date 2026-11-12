#!/usr/bin/env python3
"""Joy-Con motion mouse for Linux.

The kernel hid-nintendo driver normalizes every Joy-Con into one frame:

  +X  toward the shoulder end (R/ZR on the right Joy-Con, L/ZL on the left)
  +Y  to the left
  +Z  up, out of the buttons

Hold the controller with that shoulder end pointed at the screen and the
buttons facing the ceiling. Twisting left and right moves the cursor
horizontally. Tipping the shoulder end up and down moves it vertically.
Rotation is measured around gravity, so a tilted wrist still aims correctly.

Gyro units come from the IMU event device: value / resolution is degrees
per second, and each sample is 5333 microseconds apart. Buttons and the
stick are read from hidraw because joycond keeps the button event node
for root only.
"""

from __future__ import annotations

import math
import os
import select
import subprocess
import sys
import time
from dataclasses import dataclass

import evdev
from evdev import ecodes

IMU_NAMES = ("Joy-Con (R) (IMU)", "Joy-Con (L) (IMU)")
SENS = 22.0
DEADZONE_DPS = 1.2
PRECISION = 0.35
CALIBRATION_SAMPLES = 150
STICK_DEADZONE = 450
STICK_SPAN = 1500
SCROLL_NOTCHES_PER_SEC = 8.0
DEFAULT_DT = 0.005333

# 24-bit button word, little-endian from report bytes 3..5.
BTN_Y = 1 << 0
BTN_X = 1 << 1
BTN_B = 1 << 2
BTN_A = 1 << 3
BTN_R = 1 << 6
BTN_ZR = 1 << 7
BTN_MINUS = 1 << 8
BTN_PLUS = 1 << 9
BTN_RSTICK = 1 << 10
BTN_LSTICK = 1 << 11
BTN_DOWN = 1 << 16
BTN_UP = 1 << 17
BTN_RIGHT = 1 << 18
BTN_LEFT = 1 << 19
BTN_L = 1 << 22
BTN_ZL = 1 << 23

FORWARD = (1.0, 0.0, 0.0)


@dataclass
class Report:
    buttons: int
    stick_x: int
    stick_y: int


def dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def normalize(v):
    mag = math.sqrt(dot(v, v))
    if mag < 1e-6:
        return (0.0, 0.0, 1.0)
    return (v[0] / mag, v[1] / mag, v[2] / mag)


class Pointer:
    """Gravity-referenced gyro pointer. Gyro is degrees/second, accel is g."""

    def __init__(self, sens: float = SENS, deadzone: float = DEADZONE_DPS):
        self.sens = sens
        self.deadzone = deadzone
        self.gravity = (0.0, 0.0, 1.0)
        self.fx = 0.0
        self.fy = 0.0

    def update_gravity(self, accel_g) -> None:
        mag = math.sqrt(dot(accel_g, accel_g))
        if not (0.7 < mag < 1.4):
            return
        sample = (accel_g[0] / mag, accel_g[1] / mag, accel_g[2] / mag)
        mixed = tuple((1 - 0.03) * old + 0.03 * new for old, new in zip(self.gravity, sample))
        self.gravity = normalize(mixed)

    def motion(self, gyro_dps, accel_g, dt: float, precision: float = 1.0):
        self.update_gravity(accel_g)
        if dt <= 0 or dt > 0.05:
            dt = DEFAULT_DT
        up = self.gravity
        right = normalize(cross(FORWARD, up))
        yaw = dot(gyro_dps, up)  # positive: turn left
        pitch = dot(gyro_dps, right)  # positive: tip up
        if abs(yaw) < self.deadzone:
            yaw = 0.0
        if abs(pitch) < self.deadzone:
            pitch = 0.0
        scale = self.sens * dt * precision
        self.fx += -yaw * scale
        self.fy += -pitch * scale
        ix = math.trunc(self.fx)
        iy = math.trunc(self.fy)
        self.fx -= ix
        self.fy -= iy
        if abs(ix) > 120:
            ix = 120 if ix > 0 else -120
        if abs(iy) > 120:
            iy = 120 if iy > 0 else -120
        return ix, iy


def parse_report(data: bytes, right: bool) -> Report | None:
    if len(data) < 12 or data[0] != 0x30:
        return None
    buttons = data[3] | (data[4] << 8) | (data[5] << 16)
    base = 9 if right else 6
    stick_x = data[base] | ((data[base + 1] & 0xF) << 8)
    stick_y = (data[base + 1] >> 4) | (data[base + 2] << 4)
    return Report(buttons, stick_x, stick_y)


def stick_notches(raw_x: int, raw_y: int, center_x: float, center_y: float, dt: float, acc_x: float, acc_y: float):
    """Return (horizontal notches, vertical notches, new accumulators).

    Raw stick Y above center is up. Linux REL_WHEEL positive scrolls up.
    Raw stick X above center is right.
    """

    def axis(raw: int, center: float, acc: float):
        delta = raw - center
        if abs(delta) < STICK_DEADZONE:
            vel = 0.0
        else:
            sign = 1.0 if delta > 0 else -1.0
            mag = min(1.0, (abs(delta) - STICK_DEADZONE) / STICK_SPAN)
            vel = sign * mag * SCROLL_NOTCHES_PER_SEC
        acc += vel * dt
        notches = math.trunc(acc)
        acc -= notches
        return notches, acc

    horizontal, acc_x = axis(raw_x, center_x, acc_x)
    vertical, acc_y = axis(raw_y, center_y, acc_y)
    return horizontal, vertical, acc_x, acc_y


def clicks(buttons: int, right: bool) -> tuple[bool, bool, bool, bool, bool]:
    """Left, right, middle, pause-button, precision-held."""
    if right:
        left = bool(buttons & (BTN_ZR | BTN_A))
        right_btn = bool(buttons & (BTN_R | BTN_B))
        middle = bool(buttons & BTN_RSTICK)
        pause = bool(buttons & BTN_PLUS)
        precision = bool(buttons & BTN_X)
    else:
        left = bool(buttons & (BTN_ZL | BTN_LEFT))
        right_btn = bool(buttons & (BTN_L | BTN_DOWN))
        middle = bool(buttons & BTN_LSTICK)
        pause = bool(buttons & BTN_MINUS)
        precision = bool(buttons & BTN_UP)
    return left, right_btn, middle, pause, precision


def _set_nonblock(fd: int) -> None:
    import fcntl

    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)


def find_imu():
    found = []
    for path in evdev.list_devices():
        dev = evdev.InputDevice(path)
        if dev.name in IMU_NAMES:
            found.append(dev)
        else:
            dev.close()
    found.sort(key=lambda dev: 0 if "(R)" in dev.name else 1)
    chosen = found[:1]
    for extra in found[1:]:
        extra.close()
    return chosen[0] if chosen else None


def find_hidraw(mac: str, hid_name: str) -> str | None:
    root = "/sys/class/hidraw"
    if not os.path.isdir(root):
        return None
    mac = mac.lower()
    for node in os.listdir(root):
        uevent_path = os.path.join(root, node, "device", "uevent")
        try:
            text = open(uevent_path, encoding="utf-8", errors="replace").read()
        except OSError:
            continue
        fields = dict(
            line.split("=", 1) for line in text.splitlines() if "=" in line
        )
        name = fields.get("HID_NAME", "")
        uniq = fields.get("HID_UNIQ", "").lower()
        if name == hid_name or (mac and uniq == mac):
            return os.path.join("/dev", node)
    return None


def notify(summary: str, body: str) -> None:
    try:
        subprocess.run(
            ["notify-send", "-a", "Joy-Con Mouse", "-t", "4000", summary, body],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        pass


class DeviceLost(Exception):
    pass


def _read_imu(imu):
    """Return one (timestamp_us, gyro_raw, accel_raw) tuple per IMU sample.

    A Joy-Con packet carries three samples. They arrive together, and each
    one has to be integrated or the cursor moves at a third of the real rate.
    """
    try:
        events = list(imu.read())
    except BlockingIOError:
        return []
    except OSError as exc:
        raise DeviceLost from exc
    samples = []
    gyro = [None, None, None]
    accel = [None, None, None]
    ts = None
    for event in events:
        if event.type == ecodes.EV_MSC and event.code == ecodes.MSC_TIMESTAMP:
            ts = event.value
        elif event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_RX:
                gyro[0] = event.value
            elif event.code == ecodes.ABS_RY:
                gyro[1] = event.value
            elif event.code == ecodes.ABS_RZ:
                gyro[2] = event.value
            elif event.code == ecodes.ABS_X:
                accel[0] = event.value
            elif event.code == ecodes.ABS_Y:
                accel[1] = event.value
            elif event.code == ecodes.ABS_Z:
                accel[2] = event.value
        elif event.type == ecodes.EV_SYN and event.code == ecodes.SYN_REPORT:
            if None not in gyro and None not in accel:
                samples.append((ts, tuple(gyro), tuple(accel)))
    return samples


def _read_hid(fd: int) -> list[bytes]:
    packets = []
    while True:
        try:
            data = os.read(fd, 256)
        except BlockingIOError:
            break
        except OSError as exc:
            raise DeviceLost from exc
        if not data:
            raise DeviceLost
        packets.append(data)
    return packets


def _emit_rel(ui, code: int, value: int) -> None:
    if value:
        ui.write(ecodes.EV_REL, code, int(value))


def run_session(ui, imu, hid_fd: int, right: bool) -> None:
    res = imu.absinfo(ecodes.ABS_RX).resolution or 14247
    accel_res = imu.absinfo(ecodes.ABS_X).resolution or 4096
    pointer = Pointer()
    last_ts = None
    quiet_gyro = []
    quiet_stick = []
    bias = [0.0, 0.0, 0.0]
    center = [2048.0, 2048.0]
    calibrated = False
    enabled = True
    pause_down = False
    keys = {ecodes.BTN_LEFT: False, ecodes.BTN_RIGHT: False, ecodes.BTN_MIDDLE: False}
    wheel_acc = [0.0, 0.0]
    report = Report(0, 2048, 2048)
    log("Calibrating. Hold the Joy-Con still.")

    def handle_sample(ts, gyro, accel):
        nonlocal last_ts, calibrated, enabled, pause_down

        dt = DEFAULT_DT
        if ts is not None and last_ts is not None:
            delta_us = ts - last_ts
            if delta_us < 0:
                delta_us += 1 << 32
            if 0 < delta_us < 50000:
                dt = delta_us / 1_000_000
        if ts is not None:
            last_ts = ts

        gyro_dps = tuple(value / res for value in gyro)
        accel_g = tuple(value / accel_res for value in accel)

        if not calibrated:
            mag = math.sqrt(dot(gyro_dps, gyro_dps))
            amag = math.sqrt(dot(accel_g, accel_g))
            if mag < 15 and 0.7 < amag < 1.3:
                quiet_gyro.append(gyro_dps)
                quiet_stick.append((report.stick_x, report.stick_y))
            if len(quiet_gyro) < CALIBRATION_SAMPLES:
                return
            for axis in range(3):
                bias[axis] = sum(sample[axis] for sample in quiet_gyro) / len(quiet_gyro)
            center[0] = sum(sample[0] for sample in quiet_stick) / len(quiet_stick)
            center[1] = sum(sample[1] for sample in quiet_stick) / len(quiet_stick)
            pointer.gravity = normalize(accel_g)
            calibrated = True
            pause_name = "+" if right else "−"
            log(
                "Ready. bias dps "
                f"{bias[0]:+.2f} {bias[1]:+.2f} {bias[2]:+.2f}, "
                f"stick center {center[0]:.0f},{center[1]:.0f}. "
                f"Point the shoulder end at the screen. {pause_name} pauses."
            )
            shoulder = "R/ZR" if right else "L/ZL"
            notify("Joy-Con mouse", f"Point the shoulder ({shoulder}) end at the screen. {pause_name} pauses.")
            return

        corrected = tuple(gyro_dps[i] - bias[i] for i in range(3))
        if math.sqrt(dot(corrected, corrected)) < 0.8:
            for axis in range(3):
                bias[axis] += 0.0004 * corrected[axis]
            corrected = tuple(gyro_dps[i] - bias[i] for i in range(3))

        left, right_btn, middle, pause, precision_held = clicks(report.buttons, right)
        if pause and not pause_down:
            enabled = not enabled
            log("Paused." if not enabled else "Resumed.")
            notify("Joy-Con mouse", "Paused." if not enabled else "Resumed.")
        pause_down = pause
        if not enabled:
            left = right_btn = middle = False

        changed = False
        if enabled:
            dx, dy = pointer.motion(
                corrected, accel_g, dt, PRECISION if precision_held else 1.0
            )
            _emit_rel(ui, ecodes.REL_X, dx)
            _emit_rel(ui, ecodes.REL_Y, dy)
            hx, hy, wheel_acc[0], wheel_acc[1] = stick_notches(
                report.stick_x,
                report.stick_y,
                center[0],
                center[1],
                dt,
                wheel_acc[0],
                wheel_acc[1],
            )
            _emit_rel(ui, ecodes.REL_HWHEEL, hx)
            _emit_rel(ui, ecodes.REL_WHEEL, hy)
            if dx or dy or hx or hy:
                changed = True
        for code, down in (
            (ecodes.BTN_LEFT, left),
            (ecodes.BTN_RIGHT, right_btn),
            (ecodes.BTN_MIDDLE, middle),
        ):
            if keys[code] != down:
                keys[code] = down
                ui.write(ecodes.EV_KEY, code, 1 if down else 0)
                changed = True
        if changed:
            ui.syn()

    while True:
        readable, _, _ = select.select([imu.fd, hid_fd], [], [], 1.0)
        if not readable:
            continue
        samples = _read_imu(imu) if imu.fd in readable else []
        if hid_fd in readable:
            for packet in _read_hid(hid_fd):
                parsed = parse_report(packet, right)
                if parsed is not None:
                    report = parsed
        for ts, gyro, accel in samples:
            handle_sample(ts, gyro, accel)


def log(message: str) -> None:
    print(message, flush=True)


def open_mouse():
    return evdev.UInput(
        {
            ecodes.EV_REL: [ecodes.REL_X, ecodes.REL_Y, ecodes.REL_WHEEL, ecodes.REL_HWHEEL],
            ecodes.EV_KEY: [ecodes.BTN_LEFT, ecodes.BTN_RIGHT, ecodes.BTN_MIDDLE],
        },
        name="Joy-Con Motion Mouse",
    )


def acquire_lock():
    import fcntl

    path = os.path.join(os.environ.get("XDG_RUNTIME_DIR", "/tmp"), "joycon-mouse.lock")
    handle = open(path, "w")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        log("joycon-mouse is already running.")
        sys.exit(0)
    handle.write(str(os.getpid()))
    handle.flush()
    return handle


def main() -> None:
    lock = acquire_lock()
    while True:
        imu = find_imu()
        if imu is None:
            time.sleep(2)
            continue
        right = "(R)" in imu.name
        hid_name = imu.name.replace(" (IMU)", "")
        hid_path = find_hidraw(imu.uniq or "", hid_name)
        if hid_path is None:
            log(f"Found {imu.name} but no hidraw yet.")
            imu.close()
            time.sleep(2)
            continue
        try:
            hid_fd = os.open(hid_path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            log(f"Cannot read {hid_path}: {exc}")
            imu.close()
            time.sleep(2)
            continue
        _set_nonblock(imu.fd)
        log(f"Using {imu.name} ({imu.uniq}) and {hid_path}")
        ui = open_mouse()
        try:
            run_session(ui, imu, hid_fd, right)
        except DeviceLost:
            log("Joy-Con disconnected. Waiting for it to come back.")
        finally:
            ui.close()
            imu.close()
            os.close(hid_fd)
        time.sleep(1)


if __name__ == "__main__":
    main()
