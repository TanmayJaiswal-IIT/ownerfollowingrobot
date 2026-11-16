# Features: DesiPachinos Owner-Following Robot

✅ built   🟡 built, needs tuning/testing   🔲 planned

## Core features

1. **Owner lock** 🟡: owner raises one hand above the head for 1 s; the robot saves their track ID and clothing colours and follows only them.
2. **Stranger rejection** 🟡: every person gets a tracking ID; anyone who is not the owner is ignored, even if they are closer.
3. **Safe-distance follow** 🟡: keeps about 1 m from the owner by comparing the owner's box height with `TARGET_H`; moves forward when the owner is far, stops when close.
4. **Steering** 🟡: turns towards the owner to keep them in the centre of the camera view (proportional control on the box centre).
5. **Minimum distance** 🟡: never drives closer than the target distance; forward speed drops to 0 inside the deadband. (Planned: reverse slowly if the owner comes closer than about 0.6 m.)
6. **Unlock** 🟡: owner raises both hands for 1.5 s; the robot pauses while the hands are up, then goes idle with a 2 s cooldown.
7. **Stop when owner lost** ✅: if the owner is not seen for 0.3 s the motors stop; the robot never follows a stranger.
8. **Re-lock by appearance** 🟡: when the owner returns with a new tracking ID, they are re-identified by clothing colour (match ≥ 0.75).
9. **Obstacle stop** 🟡: the Arduino ultrasonic sensor blocks forward motion under 20 cm; reversing and turning are still allowed.
10. **Watchdog failsafe** ✅: the Arduino stops the motors if no command arrives for 500 ms (Python crash, frozen laptop, unplugged cable).
11. **Emergency stop** ✅: physical switch that cuts battery power to the motors.
12. **Live display and logs** ✅: large on-screen state, owner, motor values and gesture progress; timestamped events in the terminal and `follower.log`.
13. **Owner search** 🔲: after losing the owner, rotate slowly in place towards the side they were last seen on for a few seconds; stop and go idle if not found. (Needs a team decision: it relaxes the "never move when the owner is not visible" rule.)
14. **Obstacle avoidance (simple pathfinding)** 🔲: when blocked, stop, check the left and right side (servo-mounted or extra side sensors), turn towards the clearer side, drive past, then turn back towards the owner.
15. **Re-lock after avoidance** 🔲: after a detour, look for the owner by tracking ID and clothing match; if not found, start the owner search (13).
16. **Spin Mode** 🔲: owner draws a circle in the air with one hand; the robot spins in the same direction (demo finale).

## Suggested extras

- **Shoulder-level gestures**: lock with the hand at shoulder height instead of above the head; feels more natural.
- **Lost timeout**: after about 10 s in LOST, go back to IDLE so anyone can lock again.
- **Slow down near the owner**: scale the top speed down as the owner gets closer, for smoother stops.
- **Better distance estimate**: use shoulder width instead of box height, so raised arms don't change the estimate.
- **Status LED / buzzer**: different colour or beep for IDLE, FOLLOW, LOST and obstacle.
- **Battery monitor**: Arduino measures battery voltage and stops with a warning when it is low.
- **Stop gesture**: palm held towards the camera pauses the robot without unlocking.
