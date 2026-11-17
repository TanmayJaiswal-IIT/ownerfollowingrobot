# Ideal Features and Team Ownership: DesiPachinos

The full feature set we aim to demo, and which team member owns (builds, tests and defends) each part.
✅ built   🟡 built, needs tuning/testing   🔲 to build

| Member | Area | Features |
| --- | --- | --- |
| **Vidit Shrimali** | Perception and gestures | 1–5 |
| **Tanmay Jaiswal** | Motion control and behaviour | 6–10 |
| **Karan Kumar Prasad** | Hardware, safety and signalling | 11–15 |

---

## Vidit Shrimali: Perception and gestures

**1. Owner lock** 🟡
One hand above the head for 1 s → the robot saves that person's track ID and torso colour histogram.
*Defend:* Why a 1 s hold? So a hand raised while walking or waving does not lock by accident.

**2. Stranger rejection** 🟡
YOLOv8-pose detects people; BoT-SORT gives each a stable ID. Only the owner's ID drives the motors.
*Defend:* Why not follow the nearest person? A bystander could walk between the robot and the owner and hijack it.

**3. Unlock** 🟡
Both hands above the head for 1.5 s → IDLE, 2 s cooldown. The robot pauses while the hands are up.
*Defend:* Why longer than lock and why a cooldown? Unlocking is the riskier action, and the cooldown stops an instant re-lock.

**4. Re-lock by appearance** 🟡
If the owner returns with a new ID, the torso hue/saturation histograms are compared; correlation ≥ 0.75 → re-lock.
*Defend:* Limitation: similar clothing can confuse it, so the robot stops and waits rather than guess below the threshold.

**5. Spin Mode gesture** 🔲
Track the owner's wrist for about 1.5 s; summed angle around its centre > ±300° → spin CW/CCW. Only when stopped and no obstacle.
*Defend:* Why ±300° and not 360°? People rarely draw a perfect circle; 300° still rules out random arm movement.

---

## Tanmay Jaiswal: Motion control and behaviour

**6. Safe-distance follow (1 m gap)** 🟡
Distance is estimated from the owner's shoulder-to-hip length vs `TARGET_TORSO` (calibrated at `TARGET_DIST`); the robot drives until that gap is reached.
*Defend:* No depth sensor needed; accuracy is about ±20–30 % and is calibrated per camera and owner.

**7. Dynamic speed** 🟡
Speed = distance error term + owner-speed term. The camera only sees the relative speed (how fast the estimated gap grows over 0.5 s), so the robot's own speed (last PWM × `ROBOT_TOP_SPEED`) is added back to get the owner's walking speed. Walk faster → robot speeds up; still stops at `TARGET_DIST` (same stop rule as feature 6). Speed rises at most `RAMP_UP` PWM/s; stops are always instant. Calibration and tests: `plans/speed_plan.md`.
*Defend:* Limit: BO motors top out below a brisk walk (measured `ROBOT_TOP_SPEED` = ___ m/s), so speed is capped at `MAX_SPEED`. If the owner stays more than 2.5 m away at full speed for 1.5 s, the robot logs `[OUTRUN]` and stops; if the owner then leaves view, it starts Owner search.

**8. Steering and minimum distance** 🟡
Turns to keep the owner centred (proportional control). Up close the camera can't see the hips, so the Arduino's front ultrasonic reports the distance (`D,<cm>`). If the centred owner is closer than 60 cm, the robot backs off slowly until 75 cm. Tests: `plans/distance_plan.md` Phase 7.
*Defend:* Deadbands stop wobbling when the owner is almost centred or almost at 1 m. Reversing is blind (no rear sensor), so it is slow, at most 2 s, and only with the owner centred and the camera agreeing; if the camera says the owner is far, the sonar is seeing an obstacle, so the robot stops instead.

**9. Owner search (360° scan)** 🟡
Owner lost > 0.3 s → stop, then rotate in place in "stop-and-look" steps (turn 30°, stand still 0.7 s so detection sees sharp frames), first towards the side the owner was last seen. Owner found (original ID, or a clothing match held for 3 frames while the robot stands still) → FOLLOW. Full 360° with no match → stop, "OWNER NOT FOUND", go IDLE. `--no-search` keeps the old stop-and-wait. Tests: `plans/search_plan.md`.
*Defend:* Safe because it only rotates in place at low speed, never drives forward or backward without the owner, and the obstacle and watchdog rules still apply. Limit: no encoders, so the angle is time-based (`SPIN_DEG_PER_S`, calibrated per floor); the confirm frames lower the risk of re-locking on a stranger in similar clothes.

**10. State machine, display and logs** 🟡
IDLE → FOLLOW ↔ SEARCH → IDLE, plus LOST (stopped after an outrun, or with `--no-search`). The allowed changes are one table (`TRANSITIONS` in `control.py`); every change goes through `change_state()` and is logged as `[STATE] old -> new (reason)`. Large on-screen state and motor values; every event logged to `follower.log`. Offline tests T35–T37 check the table.
*Defend:* One clear state at a time makes behaviour predictable and easy to debug from the log (grep `[STATE]` for the timeline). Illegal changes are logged as ERROR, and tests prove every state can be reached and can always return to IDLE.

---

## Karan Kumar Prasad: Hardware, safety and signalling

**11. Arduino motor control** 🟡
Arduino Uno + L298N drive the left and right motor pairs from `M,<left>,<right>` serial commands (115200 baud).
*Defend:* Why an Arduino as well as the Pi? Real-time motor PWM and safety keep working even if Linux or Python freezes.

**12. Obstacle stop** 🟡
HC-SR04 checks the distance about every 60 ms; under 20 cm, forward motion is blocked (reverse and turning still allowed).
*Defend:* This runs on the Arduino, so it works even if the vision code sends a wrong command.

**13. Obstacle avoidance** 🔲
When blocked while following: stop, check left and right (servo-swept sensor), turn towards the clear side, pass the obstacle, turn back and re-acquire the owner (→ Owner search if not visible).
*Defend:* Simple "stop, look, go around" behaviour; reliable enough for a demo without mapping or SLAM.

**14. Failsafes** ✅
500 ms watchdog stops the motors if commands stop (crash or unplugged cable); a physical emergency-stop switch cuts battery power.
*Defend:* Two independent layers: one in software on the Arduino and one physical.

**15. Status LED and buzzer** 🔲
Python sends the state as `S,<n>`. LED: IDLE blue, FOLLOW green, SEARCH yellow blinking, obstacle red. Buzzer: 1 beep on lock, 2 on unlock, long beep when the owner is lost.
*Defend:* People nearby can see what the robot is doing without looking at a screen. Suggested pins: RGB LED on D2/D3/D4, buzzer on A1.

---

## Optional extras (if time allows)
- **Battery monitor** (Karan): voltage divider on A0; warn and stop when the battery is low.
- **Shoulder-level gestures** (Vidit): lock with the hand at shoulder height; feels more natural.
- **Shoulder-width distance** (Tanmay): steadier distance estimate than box height; raised arms don't affect it.
