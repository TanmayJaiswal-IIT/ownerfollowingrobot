#!/usr/bin/env python3
"""
Motor + safety bench test for the Arduino (no camera, no YOLO).
Keep the WHEELS OFF THE GROUND for every test.

  python motor_test.py                                  # list serial ports
  python motor_test.py --port /dev/cu.usbmodem1101      # run the test menu
  python motor_test.py --port /dev/cu.usbmodem1101 --speed 180

Commands are re-sent 20 times a second, like follower.py does, so the Arduino's
500 ms watchdog does not stop the motors in the middle of a test.
The Arduino's "D,<cm>" distance lines are ignored, except in test 10.
"""
import argparse
import time

import serial
from serial.tools import list_ports

from control import SEARCH_SPEED, parse_distance

RATE_HZ = 20     # how often commands are re-sent
STEP_S = 2.0     # s each simple test runs


def run(ser, left, right, seconds):
    """Send M,left,right repeatedly for `seconds`, then stop."""
    end = time.time() + seconds
    while time.time() < end:
        ser.reset_input_buffer()             # discard the Arduino's D,<cm> lines
        ser.write(f"M,{left},{right}\n".encode())
        time.sleep(1 / RATE_HZ)
    ser.write(b"M,0,0\n")


def sonar_readout(ser, seconds=10.0):
    """Print the latest ultrasonic distance 5 times a second, motors stopped."""
    ser.reset_input_buffer()
    buf, end = "", time.time() + seconds
    while time.time() < end:
        ser.write(b"M,0,0\n")
        cm, buf = parse_distance(buf + ser.read(ser.in_waiting).decode(errors="ignore"))
        print(f"   {cm} cm" if cm is not None else "   no reading (old firmware or sensor wiring?)")
        time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="Arduino port, e.g. /dev/cu.usbmodem1101 (Mac) or /dev/ttyACM0 (Pi)")
    ap.add_argument("--speed", type=int, default=150, help="PWM for the tests (0..255)")
    args = ap.parse_args()

    if not args.port:
        print("Serial ports found (the Arduino is usually usbmodem / usbserial / ttyACM / ttyUSB):")
        for p in list_ports.comports():
            print(f"  {p.device:<35} {p.description}")
        print("\nThen run: python motor_test.py --port <port>")
        return

    s = args.speed
    tests = {
        "1": ("LEFT wheels forward only", lambda ser: run(ser, s, 0, STEP_S)),
        "2": ("RIGHT wheels forward only", lambda ser: run(ser, 0, s, STEP_S)),
        "3": ("Both forward", lambda ser: run(ser, s, s, STEP_S)),
        "4": ("Both backward", lambda ser: run(ser, -s, -s, STEP_S)),
        "5": ("Spin RIGHT (clockwise from above): left fwd, right back", lambda ser: run(ser, s, -s, STEP_S)),
        "6": ("Spin LEFT: left back, right fwd", lambda ser: run(ser, -s, s, STEP_S)),
        "7": ("Speed sweep 40..255: note the value where BOTH sides start turning -> MIN_SPEED", None),
        "8": ("Watchdog: forward 1 s, then Python goes silent -> wheels must stop within 0.5 s", None),
        "9": ("Obstacle: forward for 10 s, put your hand < 20 cm in front of the sensor", None),
        "10": ("Ultrasonic readout for 10 s (motors stopped): compare with a tape measure",
               lambda ser: sonar_readout(ser)),
        "11": (f"Search spin calibration (ON THE FLOOR): spin right at SEARCH_SPEED={SEARCH_SPEED} for 4 s",
               lambda ser: (run(ser, SEARCH_SPEED, -SEARCH_SPEED, 4.0),
                            print("   Measure the angle turned. SPIN_DEG_PER_S = angle / 4 (set it in control.py)"))),
    }

    print(f"Opening {args.port} ...")
    ser = serial.Serial(args.port, 115200, timeout=0)
    time.sleep(2)  # the Arduino resets when the port opens
    print("Connected. WHEELS OFF THE GROUND. Keep a hand on the emergency-stop switch.\n")

    try:
        while True:
            for key, (name, _) in tests.items():
                print(f"  {key}. {name}")
            choice = input("\nTest number (q = quit): ").strip().lower()
            if choice == "q":
                break
            if choice not in tests:
                continue
            name, fn = tests[choice]
            print(f"-> {name}")
            if fn:
                fn(ser)
            elif choice == "7":
                for v in range(40, 256, 15):
                    print(f"   PWM {v}")
                    run(ser, v, v, 0.8)
            elif choice == "8":
                run(ser, s, s, 1.0)
                ser.write(f"M,{s},{s}\n".encode())   # last command is 'go', then silence
                t0 = time.time()
                print("   Python is now silent. Wheels should stop by ~0.5 s.")
                time.sleep(2.0)
                print(f"   ({time.time() - t0:.1f} s of silence done)")
            elif choice == "9":
                print("   Hand in front -> wheels stop. Hand away -> wheels resume.")
                run(ser, s, s, 10.0)
            print("   done (stopped)\n")
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"M,0,0\n")
        ser.close()
        print("Motors stopped, port closed.")


if __name__ == "__main__":
    main()
