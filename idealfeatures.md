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
Distance is estimated from the owner's box height vs `TARGET_H` (calibrated at 1 m); the robot drives until that gap is reached.
*Defend:* No depth sensor needed; accuracy is about ±20–30 % and is calibrated per camera and owner.

**7. Dynamic speed** 🔲
Speed = distance error term + owner-speed term (how fast the box is shrinking). Walk faster → robot speeds up; still stops at 1 m. Speed changes are ramped to avoid jerks.
*Defend:* Limit: BO motors top out below a brisk walk, so speed is capped at `MAX_SPEED`; if the owner outruns it, the robot goes to Owner search.

**8. Steering and minimum distance** 🟡
Turns to keep the owner centred (proportional control). If the owner comes closer than about 0.6 m, it backs off slowly.
*Defend:* Deadbands stop wobbling when the owner is almost centred or almost at 1 m.

**9. Owner search (360° scan)** 🔲
Owner lost > 0.3 s → stop, then rotate slowly in place, first towards the side the owner was last seen. Owner found (ID or clothing match) → FOLLOW. Full 360° with no match → stop, signal LOST, go IDLE.
*Defend:* Safe because it only rotates in place at low speed, never drives forward without the owner, and the obstacle and watchdog rules still apply.

**10. State machine, display and logs** 🟡
IDLE → FOLLOW ↔ SEARCH → IDLE. Large on-screen state and motor values; every event logged to `follower.log`.
*Defend:* One clear state at a time makes behaviour predictable and easy to debug from the log.

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
