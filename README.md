# DesiPachinos — Owner-Following Robot

> *"A follower that knows its owner, listens to your body, and never takes a risk."*

Academic project by **Karan Kumar Prasad, Vidit Shrimali, Tanmay Jaiswal**.

A 4WD robot that locks onto one specific owner, follows them at a safe distance using body gestures (no remote, no app, no glove), and stops for strangers and obstacles.

---

## Demo highlights
- **Raise one hand** → robot locks onto you
- **Walk around** → robot follows at ~1 m; ignores other people
- **Step out of view and return** → robot re-identifies you by clothing
- **Raise both hands** → robot unlocks, goes idle
- **Obstacle in path** → robot stops automatically
- **Circle wrist in the air** → robot spins (Spin Mode)

---

## Hardware
| Component | Detail |
|---|---|
| Raspberry Pi 5 | Runs all vision + control logic |
| Pi Camera Module 3 | `imx708`, connected via CSI |
| Arduino Uno | Motor control + safety watchdog |
| L298N motor driver | Drives 4 BO motors (left pair / right pair) |
| HC-SR04 | Front ultrasonic — stops robot at < 20 cm |
| 18650 battery pack | Powers motors via L298N |
| Emergency-stop switch | Hardware kill for motors |

---

## Software architecture
```
Pi Camera
  └─ follower.py (Python, Pi)
       ├─ YOLOv8n-pose (NCNN, ~40 FPS on Pi 5)
       ├─ BoT-SORT tracker
       ├─ State machine: IDLE → FOLLOW ↔ SEARCH → IDLE
       ├─ perception.py  — gestures, re-ID, torso size
       ├─ control.py     — P-control, speed ramp, search scan
       └─ USB serial 115200 baud
              └─ motor_controller.ino (Arduino Uno)
                   ├─ L298N PWM drive
                   ├─ 500 ms watchdog (auto-stop if Pi silent)
                   └─ HC-SR04 sonar (blocks fwd when < 20 cm)
```

**Serial protocol (simple text lines):**
- Pi → Arduino: `M,<left>,<right>\n` — PWM values −255 … 255
- Arduino → Pi: `D,<cm>\n` — ultrasonic distance (~16 Hz; `999` = no echo)

---

## Quick start

> **All commands run on the Pi over SSH. Laptop only needs to be on the same WiFi.**
> No cable needed — Pi auto-connects to saved WiFi networks on boot.

### Step 1 — SSH into the Pi

```bash
ssh pi@pi.local
# password: password
```

> `pi.local` works via mDNS on any network (college WiFi, hotspot, home — whatever the Pi joined).
> If it doesn't resolve: `arp -a | findstr pi` on Windows, or `arp -a | grep pi` on Mac/Linux.

### Step 2 — Activate the environment (every session)

```bash
cd /home/pi/ownerfollowingrobot
source .venv/bin/activate
```

### Step 3 — Run the follower

```bash
libcamerify .venv/bin/python3 follower.py \
    --port /dev/ttyACM0 \
    --imgsz 320 \
    --no-show \
    --stream

# Open in your laptop browser (same WiFi):
#   http://pi.local:8080             ← full-screen live feed + FPS overlay
#   http://pi.local:8080/stream.mjpg ← raw MJPEG (VLC, ffplay, etc.)
```

> **Why `libcamerify`?** Pi 5 uses the libcamera backend — this prefix rewires OpenCV's `VideoCapture` to use it.

### Other commands (run on Pi over SSH)

```bash
# Motor + ultrasonic bench test (no camera needed, wheels off ground)
libcamerify .venv/bin/python3 motor_test.py --port /dev/ttyACM0

# Offline unit tests (no camera, no Arduino)
.venv/bin/python3 test_follow.py      # should print "37 passed"

# Reflash Arduino firmware
~/.local/bin/arduino-cli compile --fqbn arduino:avr:uno motor_controller/
~/.local/bin/arduino-cli upload  --fqbn arduino:avr:uno -p /dev/ttyACM0 motor_controller/
```

### One-time setup (already done on our Pi — keep for reference)

```bash
cd /home/pi/ownerfollowingrobot
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Export NCNN model — ~40 FPS vs ~14 FPS with the default .pt model
# Takes ~2 min; only needs to be done once; auto-detected at runtime after this
python3 -c "from ultralytics import YOLO; YOLO('yolov8n-pose.pt').export(format='ncnn', imgsz=320)"
```

---

## CLI flags

| Flag | Default | Description |
|---|---|---|
| `--port` | *(none)* | Arduino serial port (`/dev/ttyACM0` on Pi, `/dev/cu.usbmodem…` on Mac) |
| `--imgsz` | 640 | YOLO inference size; use `320` on Pi for best FPS |
| `--model` | auto | Model path; auto-detects NCNN dir if present |
| `--no-show` | off | Skip display window (use when headless over SSH) |
| `--stream [PORT]` | 8080 | Start MJPEG HTTP server for browser live feed |
| `--no-search` | off | Disable rotation search; robot waits when owner lost |

---

## Files
| File | Purpose |
|---|---|
| `follower.py` | Main loop, state machine, overlay, serial I/O, web stream |
| `perception.py` | Gesture detection, re-ID, torso keypoints (pure functions) |
| `control.py` | Follow PID, speed ramp, search scan (pure functions) |
| `motor_test.py` | Interactive bench test: motors, watchdog, ultrasonic |
| `test_follow.py` | 37 offline unit tests (no camera) — `python test_follow.py` |
| `motor_controller/motor_controller.ino` | Arduino firmware |
| `requirements.txt` | `ultralytics`, `lap`, `pyserial` |

---

## Running unit tests
```bash
# On Pi over SSH (no camera/Arduino needed):
source .venv/bin/activate
.venv/bin/python3 test_follow.py      # should print "37 passed"
```

---

## Performance (Raspberry Pi 5)
| Mode | FPS |
|---|---|
| YOLOv8n-pose `.pt` (default) | ~14 FPS |
| YOLOv8n-pose NCNN (`--imgsz 320`) | **~29–40 FPS** |

NCNN model is auto-detected; no extra flag needed once exported.

---

## Notes
- Safety lives on the Arduino — a Python crash or serial disconnect always stops the motors within 500 ms.
- Never drive forward when the owner is not visible (enforced in code).
- State changes only happen through `change_state()` in `follower.py`.
