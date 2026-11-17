# Features: DesiPachinos Owner-Following Robot

✅ built   🟡 built, needs tuning/testing   🔲 planned

## Core features

1. **Owner lock** 🟡: owner raises one hand above the head for 1 s; the robot saves their track ID and clothing colours and follows only them.
2. **Stranger rejection** 🟡: every person gets a tracking ID; anyone who is not the owner is ignored, even if they are closer.
3. **Safe-distance follow** 🟡: keeps `TARGET_DIST` (about 1 m) from the owner by comparing the owner's shoulder-to-hip length with `TARGET_TORSO` (size ∝ 1 / distance, unaffected by raised arms); moves forward when the owner is far, stops when close, with hysteresis so it doesn't stop-start. Calibration: `plans/distance_plan.md`. **Dynamic speed** 🟡: adds an owner-speed term so a walking owner is matched before the gap opens; speed changes are ramped, stops are instant, and if the owner outruns the robot it stops (`[OUTRUN]`). Tests: `plans/speed_plan.md`.
4. **Steering** 🟡: turns towards the owner to keep them in the centre of the camera view (proportional control on the box centre).
5. **Minimum distance** 🟡: never drives closer than the target distance (torso size, plus a sonar stop when the centred owner is under `TARGET_DIST`). If the centred owner comes closer than 60 cm on the front ultrasonic, the robot reverses slowly until 75 cm (max 2 s, no rear sensor); if the camera says the owner is far, the sonar is seeing an obstacle, so it only stops.
6. **Unlock** 🟡: owner raises both hands for 1.5 s; the robot pauses while the hands are up, then goes idle with a 2 s cooldown.
7. **Stop when owner lost** ✅: if the owner is not seen for 0.3 s the motors stop; the robot never follows a stranger.
8. **Re-lock by appearance** 🟡: when the owner returns with a new tracking ID, they are re-identified by clothing colour (match ≥ 0.75).
9. **Obstacle stop** 🟡: the Arduino ultrasonic sensor blocks forward motion under 20 cm; reversing and turning are still allowed.
10. **Watchdog failsafe** ✅: the Arduino stops the motors if no command arrives for 500 ms (Python crash, frozen laptop, unplugged cable).
11. **Emergency stop** ✅: physical switch that cuts battery power to the motors.
12. **Live display and logs** ✅: large on-screen state, owner, motor values and gesture progress; timestamped events in the terminal and `follower.log`. State changes are checked against one table (`TRANSITIONS` in `control.py`) and logged as `[STATE] old -> new (reason)`; an illegal change is logged as ERROR. Tests T35–T37 prove every state is reachable and can return to IDLE.
13. **Owner search** 🟡: owner lost for 0.3 s → turn 30° in place, stand still 0.7 s to look, repeat, first towards the side they were last seen on. Original ID back, or a clothing match held for 3 frames → FOLLOW; full 360° with no match → "OWNER NOT FOUND", IDLE. Angle is time-based (calibrated `SPIN_DEG_PER_S`). Only in-place rotation, never forward/backward without the owner; `--no-search` keeps the old stop-and-wait. Tests: `plans/search_plan.md`.
14. **Obstacle avoidance (simple pathfinding)** 🔲: when blocked, stop, check the left and right side (servo-mounted or extra side sensors), turn towards the clearer side, drive past, then turn back towards the owner.
15. **Re-lock after avoidance** 🔲: after a detour, look for the owner by tracking ID and clothing match; if not found, start the owner search (13).
16. **Spin Mode** 🔲: owner draws a circle in the air with one hand; the robot spins in the same direction (demo finale).

## Suggested extras

- **Shoulder-level gestures**: lock with the hand at shoulder height instead of above the head; feels more natural.
- **Slow down near the owner**: scale the top speed down as the owner gets closer, for smoother stops.
- **Better distance estimate**: use shoulder width instead of box height, so raised arms don't change the estimate.
- **Status LED / buzzer**: different colour or beep for IDLE, FOLLOW, LOST and obstacle.
- **Battery monitor**: Arduino measures battery voltage and stops with a warning when it is low.
- **Stop gesture**: palm held towards the camera pauses the robot without unlocking.
