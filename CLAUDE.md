# CLAUDE.md: DesiPachinos Owner-Following Robot

## Vision
A small 4WD robot car that **knows who it works for**. Most "human following" robots chase the
nearest person. Ours locks onto one owner, follows only them at a safe distance, takes commands
from body gestures (no remote, app or glove), and stops rather than follow a stranger.

Pitch line: *"A follower that knows its owner, listens to your body, and never takes a risk."*

This is an **academic project** (team DesiPachinos: Karan Kumar Prasad, Vidit Shrimali,
Tanmay Jaiswal). Favour simple, readable, demo-reliable code over industry-grade
complexity. No ROS, no cloud, no custom model training.

## End goal (final demo, ~2 minutes)
1. Owner raises one hand -> robot locks onto them.
2. Owner walks around -> robot follows at ~0.4 m (lock from ~1 m away); a second person crossing the view is ignored.
3. Owner steps out of view and returns -> robot re-locks by clothing appearance.
4. Owner raises both hands -> robot unlocks and goes idle.
5. Obstacle placed in the path -> robot stops on its own.
6. Owner draws a circle in the air with a hand -> robot spins in that direction ("Spin Mode", finale).

## Hardware
- Raspberry Pi 5 (primary) or Jetson Nano, running the vision code
- Raspberry Pi Camera Module (`--source picam`) or USB webcam
- Arduino Uno + L298N motor driver + 4 BO motors (left pair / right pair in parallel)
- HC-SR04 ultrasonic sensor on the front, 18650 battery pack, physical emergency-stop switch
- Development and testing happen first on a MacBook with its webcam (no robot attached)

## Architecture
```
camera -> follower.py (Pi/Mac)                      -> USB serial ->  motor_controller.ino (Arduino)
          YOLOv8-pose + BoT-SORT tracking                              drives L298N
          state machine: IDLE -> FOLLOW <-> SEARCH -> IDLE             500 ms watchdog stop
            (+ LOST: outrun wait); allowed changes = TRANSITIONS
          perception.py: gestures, torso size, re-ID                   ultrasonic < 20 cm blocks forward
          control.py: P-control + owner-speed term, ramped; search scan
```
- Serial protocol, 115200 baud. Keep it this simple:
  - Pi -> Arduino: `M,<left>,<right>\n`, values -255..255.
  - Arduino -> Pi: `D,<cm>\n` front ultrasonic distance after every ping (~16 Hz, `999` = no echo).
- Safety lives on the Arduino too, so a Python crash or unplugged cable always stops the car.
- Perception comes from `ultralytics` (`yolov8n-pose.pt`, `model.track(..., tracker="botsort.yaml")`).
  We do not clone other GitHub repos; our code is only the feature logic on top.

## Files
- `follower.py`: main loop, state machine, overlay, serial I/O. Gesture/re-ID timing constants at the top.
- `camera.py`: `open_camera(source)`: Pi Camera Module via picamera2 (`--source picam`) or OpenCV webcam/video file.
- `perception.py`: keypoints, hand gestures, torso size, clothing histogram and `best_match` re-ID (pure functions).
- `control.py`: follow control (distance, steering, dynamic speed, ramp, back-off) and the `Search` scan as pure
  functions; their tuning constants at the top.
- `test_follow.py`: offline tests for `control.py` and `perception.py` with fake keypoints (`python test_follow.py`).
- `motor_controller/motor_controller.ino`: Arduino motor + safety firmware (wiring in header comment).
- `motor_test.py`: bench test menu for motors, watchdog and ultrasonic (no camera).
- `requirements.txt`: `ultralytics`, `lap`, `pyserial`.

## Features and status
| Feature | Logic | Status |
| --- | --- | --- |
| Owner lock | one wrist above nose for 1.0 s -> save track ID + torso HSV histogram | done, needs real-camera test |
| Unlock | both wrists above nose for 1.5 s -> IDLE, 2 s cooldown; robot pauses while held | done, needs test |
| Re-lock | owner ID lost -> compare torso histograms, re-lock if correlation >= 0.75 | done, needs test |
| Safe-distance follow | steer on box centre, drive on smoothed shoulder-to-hip length vs the owner's torso size at lock time (`LOCK_TARGET`; fallback `TARGET_TORSO`), so lock from ~1 m away; curves towards an off-centre owner while driving (`arc`) with stop/start hysteresis; steer only if torso hidden; stop when lost > 0.3 s | done, needs calibration (`plans/distance_plan.md`) |
| Dynamic speed | fwd = distance term + owner-speed term (gap rate + own speed from PWM × `ROBOT_TOP_SPEED`), capped at `MAX_SPEED`; ramped up, instant stops; outrun (full speed, gap > 2.5 m for 1.5 s) -> LOST | done, needs calibration (`plans/speed_plan.md`) |
| Steering + minimum distance | P-turn on box centre with centre deadband; sonar stop when centred owner < `TARGET_DIST`; owner centred and sonar < 60 cm (camera not saying "far") -> reverse slowly until 75 cm, max 2 s | done, needs test (`plans/distance_plan.md` Phase 7) |
| Owner search | owner lost > 0.3 s -> stop and wait; still lost after `SEARCH_DELAY` (2 s) -> SEARCH: turn 30° in place, look 0.7 s, repeat, first towards the last-seen side; angle = time × `SPIN_DEG_PER_S`; original ID back or clothing match for 3 frames -> FOLLOW; 360° with no match -> IDLE + "OWNER NOT FOUND"; `--no-search` = old stop-and-wait | done, needs calibration (`plans/search_plan.md`) |
| Obstacle stop | Arduino ultrasonic < 20 cm blocks forward motion | done, needs test |
| Spin Mode | track owner wrist ~1.5 s; summed angle around its centroid > ±300° -> spin CW/CCW; only when stopped and no obstacle. Fallback: arms in an "O" above head | **next** |

## Roadmap
1. Mac webcam test: verify lock/unlock/re-lock, watch `L=`/`R=` overlay, tune constants.
2. Wire Arduino + L298N, upload sketch, test with wheels off the ground.
3. Tethered test (Mac -> Arduino), then move to Pi (`--no-show --imgsz 320`).
4. Add Spin Mode as a new state in `follower.py`.
5. Demo rehearsal in the real room (even lighting, 2-3 people max).

## Run
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python follower.py                                   # laptop test, no robot
python follower.py --port /dev/cu.usbmodemXXXX       # Mac + Arduino
python motor_test.py --port /dev/cu.usbmodemXXXX     # motor/safety bench test, wheels off ground
python follower.py --port /dev/ttyACM0 --no-show     # Raspberry Pi + USB webcam
python follower.py --source picam --port /dev/ttyACM0 --no-show   # Raspberry Pi + Camera Module
```
On the Pi the venv needs `python3 -m venv --system-site-packages .venv` so it can see the apt package
`python3-picamera2` (picamera2 can't be pip-installed into a plain venv).

## Conventions for changes
- Keep everything in `follower.py` unless it grows past ~500 lines; then split into `perception.py`,
  `gestures.py`, `control.py`.
- Change the state only via `change_state(old, new, reason)` in `follower.py`; new states or edges go into
  `STATES` / `TRANSITIONS` in `control.py` (Spin Mode will need new edges), and T35-T37 must still pass.
- Every new gesture needs a hold time or motion threshold so it can't trigger by accident while walking.
- Never drive forward or backward when the owner is not visible. The only motion allowed without the owner is
  slow in-place rotation in SEARCH (`L = -R`). When in doubt, send `M,0,0`.
- New tuning values go in the constants block at the top with a comment and unit.
- Test gesture/control helpers with fake keypoint arrays (no camera needed) before trying on hardware.
# CLAUDE.md: DesiPachinos Owner-Following Robot

## Vision
A small 4WD robot car that **knows who it works for**. Most "human following" robots chase the
nearest person. Ours locks onto one owner, follows only them at a safe distance, takes commands
from body gestures (no remote, app or glove), and stops rather than follow a stranger.

Pitch line: *"A follower that knows its owner, listens to your body, and never takes a risk."*

This is an **academic project** (team DesiPachinos: Karan Kumar Prasad, Vidit Shrimali,
Tanmay Jaiswal). Favour simple, readable, demo-reliable code over industry-grade
complexity. No ROS, no cloud, no custom model training.

## End goal (final demo, ~2 minutes)
1. Owner raises one hand -> robot locks onto them.
2. Owner walks around -> robot follows at ~0.4 m (lock from ~1 m away); a second person crossing the view is ignored.
3. Owner steps out of view and returns -> robot re-locks by clothing appearance.
4. Owner raises both hands -> robot unlocks and goes idle.
5. Obstacle placed in the path -> robot stops on its own.
6. Owner draws a circle in the air with a hand -> robot spins in that direction ("Spin Mode", finale).

## Hardware
- Raspberry Pi 5 (primary) or Jetson Nano, running the vision code
- USB webcam / Pi camera
- Arduino Uno + L298N motor driver + 4 BO motors (left pair / right pair in parallel)
- HC-SR04 ultrasonic sensor on the front, 18650 battery pack, physical emergency-stop switch
- Development and testing happen first on a MacBook with its webcam (no robot attached)

## Architecture
```
camera -> follower.py (Pi/Mac)                      -> USB serial ->  motor_controller.ino (Arduino)
          YOLOv8-pose + BoT-SORT tracking                              drives L298N
          state machine: IDLE -> FOLLOW <-> SEARCH -> IDLE             500 ms watchdog stop
            (+ LOST: outrun wait); allowed changes = TRANSITIONS
          perception.py: gestures, torso size, re-ID                   ultrasonic < 20 cm blocks forward
          control.py: P-control + owner-speed term, ramped; search scan
```
- Serial protocol, 115200 baud. Keep it this simple:
  - Pi -> Arduino: `M,<left>,<right>\n`, values -255..255.
  - Arduino -> Pi: `D,<cm>\n` front ultrasonic distance after every ping (~16 Hz, `999` = no echo).
- Safety lives on the Arduino too, so a Python crash or unplugged cable always stops the car.
- Perception comes from `ultralytics` (`yolov8n-pose.pt`, `model.track(..., tracker="botsort.yaml")`).
  We do not clone other GitHub repos; our code is only the feature logic on top.

## Files
- `follower.py`: main loop, state machine, overlay, serial I/O. Gesture/re-ID timing constants at the top.
- `perception.py`: keypoints, hand gestures, torso size, clothing histogram and `best_match` re-ID (pure functions).
- `control.py`: follow control (distance, steering, dynamic speed, ramp, back-off) and the `Search` scan as pure
  functions; their tuning constants at the top.
- `test_follow.py`: offline tests for `control.py` and `perception.py` with fake keypoints (`python test_follow.py`).
- `motor_controller/motor_controller.ino`: Arduino motor + safety firmware (wiring in header comment).
- `motor_test.py`: bench test menu for motors, watchdog and ultrasonic (no camera).
- `requirements.txt`: `ultralytics`, `lap`, `pyserial`.

## Features and status
| Feature | Logic | Status |
| --- | --- | --- |
| Owner lock | one wrist above nose for 1.0 s -> save track ID + torso HSV histogram | done, needs real-camera test |
| Unlock | both wrists above nose for 1.5 s -> IDLE, 2 s cooldown; robot pauses while held | done, needs test |
| Re-lock | owner ID lost -> compare torso histograms, re-lock if correlation >= 0.75 | done, needs test |
| Safe-distance follow | steer on box centre, drive on smoothed shoulder-to-hip length vs the owner's torso size at lock time (`LOCK_TARGET`; fallback `TARGET_TORSO`), so lock from ~1 m away; curves towards an off-centre owner while driving (`arc`) with stop/start hysteresis; steer only if torso hidden; stop when lost > 0.3 s | done, needs calibration (`plans/distance_plan.md`) |
| Dynamic speed | fwd = distance term + owner-speed term (gap rate + own speed from PWM × `ROBOT_TOP_SPEED`), capped at `MAX_SPEED`; ramped up, instant stops; outrun (full speed, gap > 2.5 m for 1.5 s) -> LOST | done, needs calibration (`plans/speed_plan.md`) |
| Steering + minimum distance | P-turn on box centre with centre deadband; sonar stop when centred owner < `TARGET_DIST`; owner centred and sonar < 60 cm (camera not saying "far") -> reverse slowly until 75 cm, max 2 s | done, needs test (`plans/distance_plan.md` Phase 7) |
| Owner search | owner lost > 0.3 s -> stop and wait; still lost after `SEARCH_DELAY` (2 s) -> SEARCH: turn 30° in place, look 0.7 s, repeat, first towards the last-seen side; angle = time × `SPIN_DEG_PER_S`; original ID back or clothing match for 3 frames -> FOLLOW; 360° with no match -> IDLE + "OWNER NOT FOUND"; `--no-search` = old stop-and-wait | done, needs calibration (`plans/search_plan.md`) |
| Obstacle stop | Arduino ultrasonic < 20 cm blocks forward motion | done, needs test |
| Spin Mode | track owner wrist ~1.5 s; summed angle around its centroid > ±300° -> spin CW/CCW; only when stopped and no obstacle. Fallback: arms in an "O" above head | **next** |

## Roadmap
1. ~~Mac webcam test~~ — skipped; using Pi over SSH from the start.
2. Wire Arduino + L298N, upload sketch, test with wheels off the ground.
3. Live lock/unlock/re-lock test on Pi via SSH (`--stream` feed on laptop browser).
4. Tune constants (`TARGET_TORSO`, `MAX_SPEED`, etc.) with battery connected.
5. Add Spin Mode as a new state in `follower.py`.
6. Demo rehearsal in the real room (even lighting, 2-3 people max).

## Run

> All development and execution happens **on the Pi over SSH**.
> Both laptop and Pi connect to the same WiFi (or hotspot). No Ethernet cable required.

### 1 — SSH into the Pi
```bash
# From laptop terminal on same WiFi
ssh pi@pi.local
# password: password
# (Fallback if mDNS is disabled on router: ssh pi@<pi-ip-from-hotspot/router>)
```

### 2 — Activate the environment (every session)
```bash
cd /home/pi/ownerfollowingrobot
source .venv/bin/activate
```

### 3 — Run the follower
```bash
# Headless + live browser stream (standard demo mode)
libcamerify .venv/bin/python3 follower.py \
    --port /dev/ttyACM0 \
    --imgsz 320 \
    --no-show \
    --stream

# Then open on your laptop browser:
#   http://pi.local:8080             ← full-screen live feed + FPS overlay
#   http://pi.local:8080/stream.mjpg ← raw MJPEG (VLC, ffplay, etc.)
```

### 4 — Other useful commands (run on Pi over SSH)
```bash
# Motor + ultrasonic bench test (no camera needed)
libcamerify .venv/bin/python3 motor_test.py --port /dev/ttyACM0

# Offline unit tests (no camera, no Arduino)
.venv/bin/python3 test_follow.py

# Reflash Arduino firmware
~/.local/bin/arduino-cli compile --fqbn arduino:avr:uno motor_controller/
~/.local/bin/arduino-cli upload  --fqbn arduino:avr:uno -p /dev/ttyACM0 motor_controller/
```

### CLI flags
| Flag | Default | Description |
|---|---|---|
| `--port` | *(none)* | Arduino serial port — `/dev/ttyACM0` on Pi |
| `--imgsz` | 640 | YOLO inference size; use `320` on Pi for best FPS |
| `--model` | auto | Model path; auto-detects `yolov8n-pose_ncnn_model/` if present |
| `--no-show` | off | Skip OpenCV display window (required over SSH) |
| `--stream [PORT]` | 8080 | Start MJPEG HTTP server; watch in laptop browser |
| `--straight-only` | off | Hard mode: L == R always (no turning): forward / stop / slow reverse (camera-decided, max 2 s), no search |
| `--search` | off | Enable the 360° owner search (off for now: robot stops and waits when owner lost) |
| `--mirror` | off | Also flip the image left-right; use if the robot turns away from the owner |
| `--no-flip` | off | Camera is mounted upside down, so frames are rotated 180° by default; pass this if it is mounted upright |

### One-time setup (already done on our Pi — keep for reference)
```bash
cd /home/pi/ownerfollowingrobot
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
# NCNN export (~2 min, once) — auto-detected at runtime after this
python3 -c "from ultralytics import YOLO; YOLO('yolov8n-pose.pt').export(format='ncnn', imgsz=320)"
```

## Conventions for changes
- Keep everything in `follower.py` unless it grows past ~500 lines; then split into `perception.py`,
  `gestures.py`, `control.py`.
- Change the state only via `change_state(old, new, reason)` in `follower.py`; new states or edges go into
  `STATES` / `TRANSITIONS` in `control.py` (Spin Mode will need new edges), and T35-T37 must still pass.
- Every new gesture needs a hold time or motion threshold so it can't trigger by accident while walking.
- Never drive forward or backward when the owner is not visible. The only motion allowed without the owner is
  slow in-place rotation in SEARCH (`L = -R`). When in doubt, send `M,0,0`.
- New tuning values go in the constants block at the top with a comment and unit.
- Test gesture/control helpers with fake keypoint arrays (no camera needed) before trying on hardware.
- Target ~10 FPS on the Pi; prefer smaller `imgsz` over heavier models.