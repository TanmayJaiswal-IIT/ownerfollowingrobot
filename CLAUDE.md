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
2. Owner walks around -> robot follows at ~1 m; a second person crossing the view is ignored.
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
          state machine: IDLE -> FOLLOW <-> LOST                       500 ms watchdog stop
          gesture detection, re-ID, P-control                          ultrasonic < 20 cm blocks forward
```
- Serial protocol: `M,<left>,<right>\n`, values -255..255, 115200 baud. Keep it this simple.
- Safety lives on the Arduino too, so a Python crash or unplugged cable always stops the car.
- Perception comes from `ultralytics` (`yolov8n-pose.pt`, `model.track(..., tracker="botsort.yaml")`).
  We do not clone other GitHub repos; our code is only the feature logic on top.

## Files
- `follower.py`: vision, gestures, state machine, follow control, serial output. Tuning constants at the top.
- `motor_controller/motor_controller.ino`: Arduino motor + safety firmware (wiring in header comment).
- `motor_test.py`: bench test menu for motors, watchdog and ultrasonic (no camera).
- `requirements.txt`: `ultralytics`, `lap`, `pyserial`.

## Features and status
| Feature | Logic | Status |
| --- | --- | --- |
| Owner lock | one wrist above nose for 1.0 s -> save track ID + torso HSV histogram | done, needs real-camera test |
| Unlock | both wrists above nose for 1.5 s -> IDLE, 2 s cooldown; robot pauses while held | done, needs test |
| Re-lock | owner ID lost -> compare torso histograms, re-lock if correlation >= 0.75 | done, needs test |
| Safe-distance follow | steer on box centre, drive on box height vs `TARGET_H` (~1 m); stop when lost > 0.3 s | done, needs tuning |
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
python follower.py --port /dev/ttyACM0 --no-show     # Raspberry Pi
```

## Conventions for changes
- Keep everything in `follower.py` unless it grows past ~500 lines; then split into `perception.py`,
  `gestures.py`, `control.py`.
- Every new gesture needs a hold time or motion threshold so it can't trigger by accident while walking.
- Never make the robot move when the owner is not visible. When in doubt, send `M,0,0`.
- New tuning values go in the constants block at the top with a comment and unit.
- Test gesture/control helpers with fake keypoint arrays (no camera needed) before trying on hardware.
- Target ~10 FPS on the Pi; prefer smaller `imgsz` over heavier models.