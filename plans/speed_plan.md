# Dynamic Speed: Calibration and Test Plan

Hardware steps for feature 7 (dynamic speed). The code is in `control.py` (helpers and tuning constants) and
`follower.py` (main loop). Do `distance_plan.md` Phases 0 to 4 first: dynamic speed relies on a calibrated
`TARGET_TORSO`.

## How it works (for the defence)

- **Forward speed = distance term + owner-speed term**, capped at `MAX_SPEED`.
  - Distance term: `K_FWD × (TARGET_TORSO − torso size)`, as before.
  - Owner-speed term: the PWM needed to match the owner's walking pace, scaled by `VEL_GAIN`.
- **Owner speed:** the camera only sees the *relative* speed (owner minus robot), measured as how fast the
  estimated gap grows over the last `VEL_WINDOW` seconds. The robot's own speed, estimated from the last forward PWM
  and `ROBOT_TOP_SPEED`, is added back on.
- **Still stops at the target:** the stop rule is unchanged. At `TARGET_TORSO` the robot stops even if the owner
  is walking.
- **Ramp:** forward speed rises at most `RAMP_UP` PWM/s and falls at most `RAMP_DOWN` PWM/s while driving. Full
  stops (target reached, owner lost, unlock gesture) are always instant.
- **Outrun:** at ≥ 90 % of `MAX_SPEED` with the gap above `OUTRUN_DIST` for `OUTRUN_TIME`, the log shows
  `[OUTRUN]` and the robot stops in LOST until the owner is back within `OUTRUN_DIST`. If the owner leaves
  view while the robot waits, Owner search (feature 9, `search_plan.md`) starts.

Log fields used below (`[STATUS]` lines in `follower.log`): `ov=` owner speed in m/s, `fwd=` ramped forward PWM,
`dist~` estimated gap.

---

## Step 1: Measure `ROBOT_TOP_SPEED` (wheels on the floor, no camera)

1. Mark a 2 m straight line on the floor.
2. Using `motor_test.py`, drive both sides forward at `MAX_SPEED` (200). Start timing once the car is moving
   at full speed, and time it over the 2 m.
3. Repeat 3 times. `ROBOT_TOP_SPEED = 2 / mean time` (m/s). Set it in `control.py`.

| Run | Time (s) |
| --- | --- |
| 1 | |
| 2 | |
| 3 | |
| Mean / speed (m/s) | |

**Pass:** the three times differ by less than 10 %.

## Step 2: Tethered check (wheels off the ground)

```bash
python follower.py --port <port> --source 1
```

| # | Test | Expected | Result |
| --- | --- | --- | --- |
| S1 | Lock at 1.5 m, stand still | `ov=` stays within ±0.15 m/s, `fwd=` steady | |
| S2 | Walk slowly backwards | `ov=` about +0.3 to +0.6, `fwd=` climbs smoothly (no jumps above 25 per 0.1 s) | |
| S3 | Stop walking at the target mark | `fwd=0` straight away, wheels stop | |
| S4 | Walk towards the camera | `ov=` negative, no extra forward speed | |
| S5 | Hold both hands up while walking away | Wheels stop instantly (unlock pause) | |

Note: with the wheels off the ground the robot isn't really moving, so `ov=` here includes a fake robot speed.
S2 only checks that it rises and stays smooth.

**Pass:** S1 to S5 pass.

## Step 3: Floor walk tests

Open space, even lighting, someone ready at the e-stop. Measure the stop gap the same way as in
`distance_plan.md` (camera lens to the owner's toes).

For each walking speed, run **5 times with dynamic speed on** and **5 times with `VEL_GAIN = 0`** (pace term off).
Each run: walk 3 m straight away from the stopped robot, then stop. Record the stop gap, the largest `dist~`
while walking (from the log), and whether the robot jerked.

| Walk | Mode | Stop gap mean / min / max (m) | Largest gap while walking (m) | Jerks? |
| --- | --- | --- | --- | --- |
| Slow | on | | | |
| Slow | off | | | |
| Normal | on | | | |
| Normal | off | | | |
| Brisk | on | | | |
| Brisk | off | | | |

**Pass:**
- Every stop gap is within ±20 % of `TARGET_DIST`.
- The largest gap while walking is smaller with dynamic speed on than off, for normal and slow walking.
- No visible jerks.

## Step 4: Outrun test

1. Lock at about 1.5 m and jog away.
2. Expected: `[OUTRUN]` in the log about 1.5 s after the robot reaches full speed, the robot stopped, and the hint
   "Too fast for me - come back within 2.5 m".
3. Walk back to within 2 m. Expected: `[FOUND] owner back within 2.5 m after outrun`, and following resumes.
4. Repeat with the owner stepping out of view while far away: expected `[SEARCH]` and the robot turning in place
   (with `--no-search`: `[LOST]` and wheels stopped).

**Pass:** both cases stop the robot, and step 3 resumes following.

## Tuning (one value at a time, then redo the Normal walk test)

| Symptom | Fix |
| --- | --- |
| Overshoots / stops too close after walking | Lower `VEL_GAIN` (0.8 → 0.6) or `RAMP_UP` |
| Surges when the owner sways or shifts weight | Raise `OWNER_MOVING` (0.15 → 0.25) or `VEL_WINDOW` (0.5 → 0.7) |
| Falls behind even at a normal walk | Check `ROBOT_TOP_SPEED`; raise `VEL_GAIN` (max 1.0) |
| Sluggish when starting | Raise `RAMP_UP` (250 → 350) |
| `[OUTRUN]` while walking normally | Raise `OUTRUN_DIST` or `OUTRUN_TIME` |

## Results sheet

| Item | Value |
| --- | --- |
| `ROBOT_TOP_SPEED` | m/s |
| Final `VEL_GAIN` / `RAMP_UP` / `OWNER_MOVING` / `VEL_WINDOW` | |
| Step 2 tethered (S1 to S5) | pass / fail |
| Normal walk, on vs off: largest gap | m vs m |
| Stop gap worst error % (all runs) | |
| Outrun test | pass / fail |
| Notes / problems | |
