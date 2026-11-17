# Owner Search: Calibration and Test Plan

Hardware steps for feature 9 (owner search). The code is in `control.py` (`Search` class and `SEARCH_*`
constants) and `follower.py` (the SEARCH state). Disable it any time with `python follower.py --no-search`.

## How it works (for the defence)

- **Start:** the owner has not been seen for 0.3 s → state SEARCH. The robot turns first towards the side the
  owner was last seen on.
- **Stop-and-look:** turn `SEARCH_STEP_DEG` (30°) in place, then stand still for `SEARCH_LOOK` (0.7 s), and repeat.
  Detection and the clothing match only see sharp frames while the robot is still.
- **Angle is time-based:** there are no encoders or IMU, so one step lasts `SEARCH_STEP_DEG / SPIN_DEG_PER_S`
  seconds. `SPIN_DEG_PER_S` is measured once on the demo floor (S0).
- **Found:**
  - The original track ID comes back → FOLLOW immediately.
  - A clothing match ≥ `REID_THRESH` on the same new ID for `SEARCH_CONFIRM` (3) frames in a row → re-lock and
    FOLLOW. The robot stands still while confirming.
- **Not found:** after 360° the log shows `[SEARCH] ... owner not found -> IDLE`, the screen shows
  `OWNER NOT FOUND`, and there is a 2 s cooldown before anyone can lock.
- **Safety:** the only command in SEARCH is a pure spin (`L = -R = ±SEARCH_SPEED`). There is never forward or
  backward motion without the owner. A pure spin never triggers the Arduino's obstacle block, and the 500 ms
  watchdog still applies.
- **Outrun:** if the owner outran the robot and then leaves view, SEARCH starts. If they are found but still more
  than 2.5 m away, the robot waits as before.

---

## S0: Calibrate `SPIN_DEG_PER_S` (on the floor)

1. Put a tape mark on the floor in line with the front of the car.
2. `python motor_test.py --port <port>`, test **11** (spins right at `SEARCH_SPEED` for 4 s).
3. Measure the angle turned (tape marks every 45°, or a phone compass app on the car).
4. Repeat 3 times. `SPIN_DEG_PER_S = mean angle / 4`. Set it in `control.py`.

| Run | Angle (°) |
| --- | --- |
| 1 | |
| 2 | |
| 3 | |
| Mean → `SPIN_DEG_PER_S` | |

**Check:** run `follower.py --port <port>`, lock, then leave the room. After the full search, the car should face
within about ±45° of where it started. If it turns too far, raise `SPIN_DEG_PER_S`; if too little, lower it.
If the car doesn't turn at all on carpet, raise `SEARCH_SPEED` by 10 and recalibrate.

## Laptop check (no `--port`)

| # | Test | Expected | Result |
| --- | --- | --- | --- |
| P1 | Lock, step out of view | `[SEARCH] ... turning left/right first`, banner `SEARCHING FOR OWNER`, progress counts up | |
| P2 | Step back in during the scan | `[FOUND]` (or `[RE-LOCK] ... during search`), FOLLOW | |
| P3 | Stay out | After about 12 × (step + look) ≈ 13 s: `OWNER NOT FOUND`, IDLE, cooldown | |
| P4 | Same with `--no-search` | Old behaviour: `[LOST]`, stopped, waits | |

## Tethered (wheels off the ground)

| # | Test | Expected | Result |
| --- | --- | --- | --- |
| X1 | Step out of view to the **left** | Left wheels back, right wheels forward first (turning left) | |
| X2 | Step out to the **right** | Left forward, right back first | |
| X3 | Watch a full scan | Wheels stop during every look (~0.7 s) | |

## Floor (open space, e-stop person ready)

| # | Test | Expected | Result |
| --- | --- | --- | --- |
| X4 | Owner walks behind the robot | Robot turns, finds them, FOLLOW resumes | time to find: ___ s |
| X5 | Stranger in different clothes in view | No re-lock, scan continues | |
| X6 | Stranger in similar clothes | Record whether a false re-lock happens; if yes raise `SEARCH_CONFIRM` or `REID_THRESH` | |
| X7 | Nobody returns | Full turn, `OWNER NOT FOUND`, IDLE | end-angle error: ___ ° |
| X8 | Box 15 cm in front during the scan | Still turns in place, never drives forward | |

**Pass:** P1 to P4, X1 to X5, X7 and X8 pass. X6 is recorded (it shows the limit of clothing re-ID).

## Tuning

| Symptom | Fix |
| --- | --- |
| Misses the owner during the scan | Raise `SEARCH_LOOK` (0.7 → 1.0) or lower `SEARCH_STEP_DEG` (30 → 25) |
| Scan takes too long | Lower `SEARCH_LOOK` or raise `SEARCH_STEP_DEG` (keep it under the camera's horizontal view) |
| Stops short / overshoots 360° | Recalibrate `SPIN_DEG_PER_S` (S0) |
| Re-locks on strangers | Raise `SEARCH_CONFIRM` (3 → 5) or `REID_THRESH` |

## Results sheet

| Item | Value |
| --- | --- |
| `SPIN_DEG_PER_S` / `SEARCH_SPEED` | ° / s at PWM |
| End-angle error after a full search | ° |
| Time to find the owner (X4) | s |
| False re-locks (X6) | of runs |
| Notes / problems | |
