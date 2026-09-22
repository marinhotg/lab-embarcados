#!/usr/bin/env python3
"""Bluetooth control client for the rover, over a native RFCOMM socket.

Modes:

    ./bt_control.py                    keyboard, sends every 100 ms
    ./bt_control.py --hex "01 32"      sends raw bytes and exits
    ./bt_control.py --timeout-test     sends one packet and goes silent, to
                                       watch the 450 ms timeout fire in the log
    ./bt_control.py --motor-test       four maneuvers to check the wiring
    ./bt_control.py --ramp             ramps forward from 20% to 100% to find
                                       where the wheels start turning
    ./bt_control.py --power-test       phased profile to measure supply
                                       current, with the rover on the floor

Pair once before first use:

    bluetoothctl
    scan on                 # wait for "Rover"
    pair E4:65:B8:1F:C1:92
    trust E4:65:B8:1F:C1:92
    scan off
"""

from __future__ import annotations

import argparse
import select
import socket
import sys
import termios
import time
import tty

DEFAULT_MAC = "E4:65:B8:1F:C1:92"

CMD_STOP = 0x00
CMD_LINEAR = 0x01
CMD_ROTATION = 0x02

PERIOD_S = 0.1
STEP = 10

HELP = """
  w / s   forward / reverse    (10% step)
  a / d   turn left / right    (10% step)
  space   stop (0x00), zeroes both references
  0       zero the references without sending stop
  q       quit
"""


def connect(mac: str, channel: int | None) -> socket.socket:
    channels = [channel] if channel else range(1, 6)
    last_error = None

    for c in channels:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                             socket.BTPROTO_RFCOMM)
        sock.settimeout(10)
        try:
            sock.connect((mac, c))
            print(f"connected to {mac}, RFCOMM channel {c}")
            return sock
        except OSError as exc:
            last_error = exc
            sock.close()

    print(f"could not connect to {mac}: {last_error}", file=sys.stderr)
    print("paired with bluetoothctl? see the header of this file.",
          file=sys.stderr)
    sys.exit(1)


def send(sock: socket.socket, command: int, value: int) -> None:
    sock.send(bytes([command, value & 0xFF]))


def hold(sock: socket.socket, v: int, w: int, duration_s: float) -> None:
    """Repeats both commands for duration_s, keeping the firmware timeout alive."""
    end = time.monotonic() + duration_s
    while time.monotonic() < end:
        send(sock, CMD_LINEAR, v)
        send(sock, CMD_ROTATION, w)
        time.sleep(PERIOD_S)


def hex_mode(sock: socket.socket, text: str) -> None:
    data = bytes(int(b, 16) for b in text.replace(",", " ").split())
    sock.send(data)
    print("sent:", " ".join(f"{b:02X}" for b in data))


def timeout_test(sock: socket.socket) -> None:
    print("sending 01 32 (forward 50%) and going silent.")
    print("the ESP32 log should show the timeout stop after ~450 ms.")
    send(sock, CMD_LINEAR, 50)
    time.sleep(2.0)
    print("done, check the serial monitor.")


def motor_test(sock: socket.socket) -> None:
    """Four maneuvers that tell apart every H-bridge wiring mistake."""
    steps = [
        (0, 0, 1.0, "stopped"),
        (40, 0, 2.5, "FORWARD     - both wheels forward"),
        (0, 0, 1.0, "stopped"),
        (-40, 0, 2.5, "REVERSE     - both wheels backward"),
        (0, 0, 1.0, "stopped"),
        (0, 60, 2.5, "TURN LEFT   - left backward, right forward"),
        (0, 0, 1.0, "stopped"),
        (0, -60, 2.5, "TURN RIGHT  - left forward, right backward"),
        (0, 0, 1.0, "stopped"),
    ]
    for v, w, duration, label in steps:
        print(f"  {label}", flush=True)
        hold(sock, v, w, duration)
    send(sock, CMD_STOP, 0)


def ramp(sock: socket.socket) -> None:
    """The step where the wheels start turning gives the real MOTOR_DUTY_MIN."""
    print("  note the percentage at which the wheels start turning\n")
    for pct in range(20, 101, 5):
        print(f"  command {pct:3d}%", flush=True)
        hold(sock, pct, 0, 1.5)
    send(sock, CMD_STOP, 0)
    print("\n  stopped.")


def power_test(sock: socket.socket) -> None:
    """Phase 1 is the baseline (ESP, radios and sensors, motors idle); every
    later phase minus the baseline is traction current."""
    phases = [
        (0, 0, 30, "BASELINE      ESP + radios + sensors, motors stopped"),
        (50, 0, 20, "FORWARD  50%  steady traction"),
        (100, 0, 20, "FORWARD 100%  maximum traction"),
        (0, 0, 5, "pause"),
        (0, 100, 20, "SPIN    100%  both sides in opposite directions"),
        (0, 0, 5, "pause"),
    ]

    print("\n  Rover ON THE FLOOR. Write down the supply current in each phase.\n")
    for v, w, duration, label in phases:
        print(f"\n  {label}")
        end = time.monotonic() + duration
        while time.monotonic() < end:
            send(sock, CMD_LINEAR, v)
            send(sock, CMD_ROTATION, w)
            print(f"\r    {end - time.monotonic():4.0f} s ", end="", flush=True)
            time.sleep(PERIOD_S)

    print("\n\n  REPEATED STARTS - watch the supply PEAK")
    for i in range(8):
        print(f"\r    start {i+1}/8 ", end="", flush=True)
        hold(sock, 100, 0, 1.0)
        hold(sock, 0, 0, 1.0)

    send(sock, CMD_STOP, 0)
    print("""

  Fill in:

    baseline (no motor) ...... ____ A   -> ESP + radios + sensors
    forward 50% .............. ____ A
    forward 100% ............. ____ A
    spin 100% ................ ____ A
    PEAK on starts ........... ____ A   -> sizes the power bank

  traction = (forward 100%) - (baseline)
""")


def read_key() -> str | None:
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def keyboard_mode(sock: socket.socket) -> None:
    v = 0
    w = 0
    print(HELP)

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        next_send = time.monotonic()

        while True:
            key = read_key()

            if key == "q":
                send(sock, CMD_STOP, 0)
                print("\nstop sent, exiting.")
                return
            elif key == " ":
                v = w = 0
                send(sock, CMD_STOP, 0)
            elif key == "0":
                v = w = 0
            elif key == "w":
                v = min(100, v + STEP)
            elif key == "s":
                v = max(-100, v - STEP)
            elif key == "a":
                w = min(100, w + STEP)
            elif key == "d":
                w = max(-100, w - STEP)

            now = time.monotonic()
            if now >= next_send:
                send(sock, CMD_LINEAR, v)
                send(sock, CMD_ROTATION, w)
                next_send = now + PERIOD_S
                print(f"\r  v={v:+4d}%   w={w:+4d}%   "
                      f"[01 {v & 0xFF:02X}] [02 {w & 0xFF:02X}]   ",
                      end="", flush=True)

            time.sleep(0.005)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def main() -> None:
    p = argparse.ArgumentParser(description="Rover Bluetooth control")
    p.add_argument("--mac", default=DEFAULT_MAC)
    p.add_argument("--channel", type=int, default=None,
                   help="RFCOMM channel; tries 1 to 5 by default")
    p.add_argument("--hex", dest="raw_hex",
                   help='raw bytes, e.g. --hex "01 32"')
    p.add_argument("--timeout-test", action="store_true")
    p.add_argument("--motor-test", action="store_true")
    p.add_argument("--ramp", action="store_true")
    p.add_argument("--power-test", action="store_true")
    args = p.parse_args()

    sock = connect(args.mac, args.channel)
    try:
        if args.raw_hex:
            hex_mode(sock, args.raw_hex)
        elif args.timeout_test:
            timeout_test(sock)
        elif args.motor_test:
            motor_test(sock)
        elif args.ramp:
            ramp(sock)
        elif args.power_test:
            power_test(sock)
        else:
            keyboard_mode(sock)
    except KeyboardInterrupt:
        try:
            send(sock, CMD_STOP, 0)
        except OSError:
            pass
        print("\ninterrupted, stop sent.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
