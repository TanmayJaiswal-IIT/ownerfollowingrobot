# Safe-Distance Follow: Implementation Plan

Step-by-step plan to turn feature 3 ("keeps about 1 m from the owner") from a guess into a
calibrated, measured feature. Do the phases **in order**; each one ends with a pass check.
Record numbers in the [Results sheet](#results-sheet) at the end.

| Phase | What | Robot needed |
| --- | --- | --- |
| 0 | Camera mount and framing check, choose the follow distance | camera on the car, no motors |
| 1 | Code changes in `follower.py` | no |
| 2 | Offline tests with fake keypoints (`test_follow.py`) | no |
| 3 | Calibration with a tape measure | camera only |
| 4 | Tethered check, wheels off the ground | yes |
| 5 | Floor accuracy test and tuning | yes |
| 6 | Update the docs with the measured numbers | no |
| 7 | Ultrasonic telemetry and back-off (feature 8) | yes |

---

## Why the current method has to change

Today the robot compares the owner's **whole-body box height** with `TARGET_H = 0.55` (an untested
placeholder). That only measures distance while the whole person is inside the frame, and a normal
webcam cannot fit a whole standing person at 1 m:

| Camera | Vertical field of view | Height visible at 1 m |
| --- | --- | --- |
| Logitech C270 | about 35° | about 0.6 m |
| Logitech C920 / laptop webcam | about 43° | about 0.8 m |
| Pi Camera Module 3 Wide | about 66° | about 1.3 m |

A person is about 1.7 m tall (about 2.1 m with hands raised). At 1 m the box gets cut off by the frame
edge, so its height stops changing with distance. Raised arms also make the box taller, which
looks like "closer".

**New distance cue: torso length**, the vertical pixel distance from the shoulders to the hips
(keypoints 5, 6, 11, 12). It is about 0.5 m on a real person, fits in the frame at 1 to 3 m, and does
not change when the owner raises their arms or turns sideways. Pinhole camera rule:
size in the image is proportional to 1 / distance, so `distance ≈ TARGET_DIST × TARGET_TORSO / torso`.

---

## Phase 0: Camera mount and framing check

Goal: the camera sees the owner's **hips, nose and raised wrists** at the follow distance
(needed for the distance cue *and* for the lock and unlock gestures).

1. Mount the camera on the car at its final position. Height 20 to 50 cm, tilted **up** about 20 to 30°.
2. Show skeletons with the stock Ultralytics viewer (no code needed):
   ```bash
   yolo pose predict model=yolov8n-pose.pt source=0 show=True
   ```
   (`source=1` for the USB webcam if `0` is the laptop camera.)
3. Measure from the **camera lens to the owner's toes** with a tape measure. Use this same way of
   measuring in every later phase.
4. Stand at 1.0 m, 1.5 m and 3.0 m. At each, check that you see: both hips, the nose, and both wrists with
   both hands raised above the head.
5. Change the tilt in steps of about 5° until the checks pass at as many distances as possible.

Decision:
- All checks pass at 1.0 m and 3.0 m: `TARGET_DIST = 1.0`.
- They only pass from 1.5 m: `TARGET_DIST = 1.5`. Write this down; the pitch becomes "about 1.5 m".
- They fail at 3.0 m: lower the tilt a little; locking happens from further away, so 3 m matters.

**Pass:** a tilt and `TARGET_DIST` are chosen and written in the results sheet; the camera is fixed in place
(tape or screw it, any later change of tilt means redoing Phase 3).

---

## Phase 1: Code changes in `follower.py`

`follower.py` is about 380 lines; these changes add about 50, so it stays one file.

### 1.1 Constants

Replace `TARGET_H` and `DEADBAND` in the tuning block with:

```python
L_HIP, R_HIP = 11, 12    # add to the COCO keypoint indices line

TARGET_DIST = 1.0        # m, wanted gap from camera lens to owner (set in Phase 0)
TARGET_TORSO = 0.30      # shoulder-to-hip length / frame height at TARGET_DIST; measure in Phase 3!
DEADBAND = 0.03          # once stopped, start again only when the torso is this much smaller (~10 % of TARGET_TORSO)
SIZE_SMOOTH = 0.3        # 0..1, weight of the newest frame in the smoothed torso size
K_FWD = 1000             # forward gain (doubled because torso sizes are about half of box sizes)
```

### 1.2 New helper: `torso_ratio(k, H)`

```python
def torso_ratio(k, H):
    """Shoulder-to-hip length / frame height, or None if shoulders or hips are not visible."""
    sh = [p for p in (keypoint(k, L_SHOULDER), keypoint(k, R_SHOULDER)) if p]
    hp = [p for p in (keypoint(k, L_HIP), keypoint(k, R_HIP)) if p]
    if not sh or not hp:
        return None
    length = np.mean([p[1] for p in hp]) - np.mean([p[1] for p in sh])
    return length / H if length > 0 else None
```

### 1.3 New helper: `est_distance(size)`

```python
def est_distance(size):
    """Rough distance in metres from the torso size (size ∝ 1 / distance)."""
    return TARGET_DIST * TARGET_TORSO / size if size else None
```

Only used for the on-screen text and logs, never for control. It makes calibration and the demo explanation easy ("~1.3 m").

### 1.4 Rewrite `follow_command` with hysteresis

The current rule stops and starts at the same size, and the jump to `MIN_SPEED` (70) makes the robot
stop and start over and over near the target. New rule: stop when the torso reaches `TARGET_TORSO`; once stopped,
drive again only when it shrinks below `TARGET_TORSO - DEADBAND`.

```python
def follow_command(box, W, size, driving):
    """Steer toward the owner; drive until the torso is TARGET_TORSO tall.
    `driving` is the previous decision: once stopped, start again only when the owner is DEADBAND farther."""
    x1, _, x2, _ = box
    err_x = ((x1 + x2) / 2 - W / 2) / (W / 2)      # -1 (left) .. +1 (right)
    if size is None:
        driving = False                             # torso not visible: steer only, never drive blind
    elif driving:
        driving = size < TARGET_TORSO
    else:
        driving = size < TARGET_TORSO - DEADBAND
    fwd = K_FWD * (TARGET_TORSO - size) if driving else 0.0
    turn = K_TURN * err_x if abs(err_x) > CENTER_DEADBAND else 0.0
    return shape_speed(fwd + turn), shape_speed(fwd - turn), driving
```

### 1.5 Main loop changes

1. New state next to `owner_ratio`: `size_s, driving = None, False` (smoothed torso size, hysteresis flag).
   Remove `owner_ratio`.
2. **Reset** `size_s = None` and `driving = False` on `[LOCK]`, `[RE-LOCK]`, `[LOST]` and `[UNLOCK]`, so an old
   value never carries over to a new track.
3. In the owner-visible branch, replace `owner_ratio = (box[3] - box[1]) / H` with:
   ```python
   raw = torso_ratio(k, H)
   if raw is not None:
       size_s = raw if size_s is None else (1 - SIZE_SMOOTH) * size_s + SIZE_SMOOTH * raw
   size = size_s if raw is not None else None     # torso hidden this frame -> steer only
   ```
4. Replace the `follow_command` call:
   `left, right, driving = follow_command(box, W, size, driving)`.
   While the unlock gesture is held, also set `driving = False`.
5. Hint text in FOLLOW:
   - torso visible: `f"Torso {size:.2f} / {TARGET_TORSO:.2f}  ~{est_distance(size):.1f} m   both hands up = unlock"`
   - torso hidden: `"Torso not visible - steering only"`
6. `[STATUS]` log line: replace `size=...` with `torso={size_s:.3f}/{TARGET_TORSO:.2f} dist~{d:.2f}m drive={driving}`
   (print `-` for missing values). Phase 3 and 5 read their numbers from this line in `follower.log`.

**Pass:** `python follower.py` (no `--port`) runs on the laptop, lock works, the hint shows torso size and
an estimated distance that grows when you step back.

---

## Phase 2: Offline tests with fake keypoints

New file `test_follow.py`, run with `python test_follow.py` (plain `assert`, no extra dependency). Build fake
17x3 keypoint arrays: helper `fake_person(shoulder_y, hip_y, conf=0.9)` that sets keypoints 5, 6, 11, 12 and
leaves the rest at confidence 0.

| # | Test | Expected |
| --- | --- | --- |
| T1 | `torso_ratio` with shoulders at y=100, hips at y=250, H=500 | 0.30 |
| T2 | Hips at confidence 0.1 | `None` |
| T3 | Hips above the shoulders (lying down / bad detection) | `None` |
| T4 | Far: size = 0.15, centred box, `driving=False` | both sides forward (> 0), `driving=True` |
| T5 | At target: size = `TARGET_TORSO`, `driving=True` | `L = R = 0`, `driving=False` |
| T6 | Hysteresis: stopped, size = `TARGET_TORSO - 0.01` | stays stopped |
| T7 | Hysteresis: stopped, size = `TARGET_TORSO - DEADBAND - 0.01` | drives |
| T8 | Driving, size = `TARGET_TORSO - 0.01` | keeps driving at `MIN_SPEED` |
| T9 | size = `None`, owner far right of the frame | no forward part, left > right (turns towards the owner) |
| T10 | size = `None`, owner centred | `L = R = 0` |
| T11 | `est_distance(TARGET_TORSO)` and `est_distance(TARGET_TORSO / 2)` | `TARGET_DIST` and `2 × TARGET_DIST` |

**Pass:** all tests print OK. Rerun after every later change to these helpers.

---

## Phase 3: Calibration with a tape measure (camera on the car, no `--port`)

```bash
python follower.py --source 1
```

1. Put tape marks on the floor at 0.75, 1.0, 1.25, 1.5, 2.0, 2.5 and 3.0 m from the camera lens.
2. Lock (one hand up) from the 3.0 m mark.
3. Stand still on each mark for 5 s, arms down, facing the camera. Read `torso=` from the last
   `[STATUS]` lines and fill the table.

| Distance d (m) | Torso size s | Torso visible? | s × d |
| --- | --- | --- | --- |
| 0.75 | | | |
| 1.0 | | | |
| 1.25 | | | |
| 1.5 | | | |
| 2.0 | | | |
| 2.5 | | | |
| 3.0 | | | |

4. Set `TARGET_TORSO` = the size measured at `TARGET_DIST`.
5. **Jitter check:** stand still at `TARGET_DIST` for 10 s and note the lowest and highest `torso=` value.
   If the spread is bigger than `DEADBAND`, lower `SIZE_SMOOTH` (e.g. 0.3 → 0.2) and repeat.
6. **Per-owner check:** repeat step 3 at 1.0 m and 2.0 m for each team member and note their sizes.
   The demo owner uses their own `TARGET_TORSO`.

**Pass:**
- The torso is visible at every mark from `TARGET_DIST` to 3.0 m.
- The size goes down at every step.
- `s × d` stays within ±15 % of its average (confirms the 1 / distance rule; if not, the camera tilt is too steep).
- Jitter spread is smaller than `DEADBAND`.

---

## Phase 4: Tethered check (wheels off the ground)

Same setup as `arduino.md` Phase F (all of Phase E must already pass).

```bash
python follower.py --port <port> --source 1
```

| # | Test | Expected | Result |
| --- | --- | --- | --- |
| H1 | Lock at 2.5 m, centred | Both sides forward, `drive=True`, estimate about 2.5 m | |
| H2 | Walk slowly to the `TARGET_DIST` mark | Wheels slow down, then stop at the mark | |
| H3 | Step back 10 cm | Wheels stay stopped (hysteresis) | |
| H4 | Step back 40 cm | Wheels start again | |
| H5 | At the target, raise one arm / wave | No change in `torso=`, wheels stay stopped | |
| H6 | At the target, turn sideways (profile) | `torso=` changes less than `DEADBAND` | |
| H7 | Lean so your hips leave the frame (or cover them) | `Torso not visible - steering only`, no forward motion | |
| H8 | Stand 1 m to the side at 2 m | Turns towards you and drives | |

**Pass:** H1 to H8 pass.

---

## Phase 5: Floor accuracy test and tuning

Set `MAX_SPEED = 150`. Open space, even lighting, someone ready at the e-stop. Measure the final gap the same
way as in Phase 0 (camera lens to the owner's toes).

| Test | How | Runs |
| --- | --- | --- |
| Approach | Owner stands still at 3 m, locks; robot drives up and stops | 5 |
| Follow | Robot stopped at the target; owner walks 2 m straight back and stops | 5 |
| Angled | Owner starts 30° to the side at 2.5 m | 3 |
| Other owner | Another team member, **without** recalibrating | 2 |
| Other owner, recalibrated | Same person with their own `TARGET_TORSO` | 2 |

Per run, write the measured gap. Then for each test: mean, minimum, maximum, and
**worst error % = max |gap − TARGET_DIST| / TARGET_DIST × 100**.

Tuning (change **one** value at a time, then redo the Approach test):

| Symptom | Fix |
| --- | --- |
| Robot keeps stopping and starting near the owner | Raise `DEADBAND` or lower `SIZE_SMOOTH` |
| Robot always stops too close (overshoots) | Lower `K_FWD`, or `MAX_SPEED`; if still too close, lower `TARGET_TORSO` a little |
| Robot always stops too far | Raise `TARGET_TORSO` a little |
| Robot crawls the last half metre | Expected (`MIN_SPEED` creep); raise `K_FWD` only if it's too slow |
| Large spread between runs | Check FPS ≥ 10 in `[STATUS]`; redo the Phase 3 jitter check |

**Pass:** in the Approach, Follow and Angled tests, every run stops within **±20 %** of `TARGET_DIST`
(0.8 to 1.2 m for 1 m). The "Other owner" numbers are recorded (they show why calibration is per owner).

---

## Phase 6: Update the docs

- `features.md` item 3: torso length instead of box height, the real `TARGET_DIST`, status 🟡 → ✅ if Phase 5 passed.
- `arduino.md` Phase G step 1: replace "read `Size x`, set `TARGET_H`" with "follow `distance_plan.md` Phase 3";
  rename `TARGET_H` to `TARGET_TORSO` in the results sheet; F3 text "under 1 m" → "closer than `TARGET_DIST`".
- `CLAUDE.md`: features table row "Safe-distance follow" and the vision text if `TARGET_DIST` is no longer 1 m.
- Defend line, filled in with the Phase 5 numbers:

  > *Distance is estimated from the owner's shoulder-to-hip length in the image (size ∝ 1 / distance), calibrated
  > once per camera and owner with a tape measure. Measured over N runs, the robot stops at X m ± Y cm
  > (worst error Z %). No depth sensor needed, and raised arms don't affect it.*

---

## Phase 7: Ultrasonic telemetry and back-off (feature 8)

The HC-SR04 measures what is straight ahead (about 2 cm to 4 m, beam about 15° wide). It is now built in:

- **Firmware:** after every ping the Arduino sends `D,<cm>\n` (about 16 Hz; `999` = no echo). It only sends when the
  TX buffer has room, so motor control and the watchdog never stall. **Re-upload the sketch.**
- **Python:** `Motors.read_distance()` returns the latest reading, or `None` if it is older than 0.3 s. `[STATUS]`
  shows `us=<cm>`, which is a second measurement of the gap in Phase 5.
- **Sonar stop:** when the owner is centred (`|err_x| < BACKOFF_CENTER`) and the reading is under `TARGET_DIST × 100`
  cm, forward speed is 0, even if `TARGET_TORSO` is miscalibrated.
- **Back-off:** when the centred owner is under `MIN_DIST_CM` (60 cm) and the camera does not say they are far,
  the robot reverses at `BACKOFF_SPEED` until the reading is above `BACKOFF_CLEAR_CM` (75 cm). Reversing lasts at
  most `BACKOFF_MAX_TIME` (2 s), because there is no rear sensor. After that it waits until the owner steps back.
  If the camera says the owner is far (≥ `TARGET_DIST`), the sonar is seeing an obstacle: stop, no reverse.

### 7.1 Bench (`motor_test.py`, wheels off the ground)

| Target | Distance | `D` reading (cm) |
| --- | --- | --- |
| Flat board | 30 cm | |
| Flat board | 60 cm | |
| Flat board | 100 cm | |
| Person in a sweater | 30 cm | |
| Person in a sweater | 60 cm | |
| Person in a sweater | 100 cm | |

Use test 10 (ultrasonic readout). Then rerun tests 8 (watchdog) and 9 (obstacle stop): they must still pass.

### 7.2 Tethered (wheels off the ground, `python follower.py --port <port> --source 1`)

| # | Test | Expected | Result |
| --- | --- | --- | --- |
| B1 | Stand at 50 cm, centred | Wheels reverse slowly, `[BACKOFF] owner at .. cm -> reversing` | |
| B2 | Then step back to 80 cm | Reversing stops, `[BACKOFF] done` | |
| B3 | Stay at 50 cm | Stops after 2 s, `[BACKOFF] time limit`; no restart until you step back past 75 cm and close in again | |
| B4 | Stand at 50 cm, off to the side | No reverse | |
| B5 | Box at 50 cm, owner at 2 m behind it | Stopped, no reverse, `[SONAR] obstacle ...` | |
| B6 | Both hands up at 50 cm | Wheels stop (unlock pause) | |
| B7 | Stand at 0.8 m centred with `TARGET_TORSO` set too high on purpose | Wheels stay stopped (sonar stop) | |

### 7.3 Floor

5 runs: the owner walks towards the stopped robot and stops at about 0.4 m. Record the final gap and how far the
robot reversed.

| Run | Final gap (m) | Reversed (m) |
| --- | --- | --- |
| 1 | | |
| 2 | | |
| 3 | | |
| 4 | | |
| 5 | | |

**Pass:** B1 to B7 pass; every final gap is 0.6 to 0.8 m and every reverse is under 0.5 m. If soft clothing
reads too far in 7.1, lower `BACKOFF_CENTER` or raise `MIN_DIST_CM` a little.

---

## Results sheet

Date: __________  Testers: __________________

| Item | Value |
| --- | --- |
| Camera model and vertical field of view | |
| Camera height / tilt | cm / ° |
| `TARGET_DIST` chosen in Phase 0 | m |
| `TARGET_TORSO` (demo owner) | |
| Other owners' torso sizes at 1 m | |
| Jitter spread at target / `DEADBAND` | / |
| Final `SIZE_SMOOTH` / `K_FWD` / `MAX_SPEED` | |
| Phase 2 offline tests | pass / fail |
| Phase 4 tethered (H1 to H8) | pass / fail |
| Approach: mean / min / max / worst % | |
| Follow: mean / min / max / worst % | |
| Angled: mean / min / max / worst % | |
| Other owner without / with recalibration | |
| Notes / problems | |
