# Arduino Testing Plan

Step-by-step plan for testing the robot car's Arduino side: wiring, firmware, motors, safety, and then
the link with `follower.py`. Do the phases **in order** and do not skip a phase that failed.
Record results in the [Results sheet](#results-sheet) at the end.

| Phase | What | Wheels |
| --- | --- | --- |
| A | Wiring inspection | power off |
| B | Power-up check | off the ground |
| C | Upload firmware | off the ground |
| D | Motor tests (`motor_test.py`) | off the ground |
| E | Safety tests (watchdog, cable pull, obstacle, e-stop) | off the ground |
| F | Vision + Arduino, tethered to the laptop | off the ground |
| G | Floor tests and tuning | on the floor |

---

## What you need

- Assembled car: Arduino Uno, L298N, 4 BO motors, HC-SR04, 18650 battery pack, emergency-stop switch
- Mac with the project set up (`source .venv/bin/activate` works)
- USB **data** cable for the Arduino (some cables only carry power)
- USB webcam (for phases F and G)
- A box or stack of books to lift the car so the wheels spin freely
- Multimeter (recommended), tape measure, a book or piece of cardboard as an "obstacle"
- Arduino IDE installed

## Safety rules (read before starting)

1. **Wheels off the ground** for phases B to F.
2. One person keeps a hand on the **emergency-stop switch** whenever the battery is on.
3. Battery off whenever you change wiring.
4. When the Arduino is on USB, **do not** connect the L298N's `5V` pin to the Arduino's `5V`. Only GND is shared.
5. Only one program can use the serial port: close the Arduino IDE's Serial Monitor before running Python.
6. First floor runs use `MAX_SPEED = 150` in `control.py`, in an open space.

---

## Phase A: Wiring inspection (battery off, USB unplugged)

Check every wire against this table (it matches the header of `motor_controller/motor_controller.ino`).

| From | To | Check |
| --- | --- | --- |
| Left motor pair (2 in parallel) | L298N `OUT1` / `OUT2` | [ ] |
| Right motor pair (2 in parallel) | L298N `OUT3` / `OUT4` | [ ] |
| L298N `ENA` | Arduino `D5` (ENA jumper cap **removed**) | [ ] |
| L298N `IN1` / `IN2` | Arduino `D6` / `D7` | [ ] |
| L298N `IN3` / `IN4` | Arduino `D8` / `D9` | [ ] |
| L298N `ENB` | Arduino `D10` (ENB jumper cap **removed**) | [ ] |
| HC-SR04 `VCC` / `GND` | Arduino `5V` / `GND` | [ ] |
| HC-SR04 `TRIG` / `ECHO` | Arduino `D11` / `D12` | [ ] |
| Battery `+` → e-stop switch → | L298N `12V` terminal | [ ] |
| Battery `−` | L298N `GND` | [ ] |
| L298N `GND` | Arduino `GND` (**common ground, required**) | [ ] |
| L298N `5V` pin | **not** connected to the Arduino while it is on USB | [ ] |

Multimeter checks (continuity / beep mode):
- [ ] L298N `GND` ↔ Arduino `GND`: beeps.
- [ ] Battery `+` ↔ battery `−`: **no** beep (no short circuit).
- [ ] E-stop switch: beeps when ON, no beep when OFF.

**Pass:** every box ticked.

---

## Phase B: Power-up check (wheels off the ground)

1. Measure the battery voltage with the multimeter and write it in the results sheet
   (2S pack: about 7.4–8.4 V, 3S pack: about 11.1–12.6 V; must be **≤ 12 V** if the L298N 5V jumper is on).
2. Switch on the battery (e-stop ON). The L298N power LED lights up. Nothing smells hot, no motor moves.
3. Switch the e-stop OFF: the L298N LED goes out. Switch it back ON.
4. Plug the Arduino into the Mac by USB. The Arduino power LED lights up.
5. Find the port:
   ```bash
   source .venv/bin/activate
   python motor_test.py
   ```
   A new port appears: `/dev/cu.usbmodemXXXX` (official Uno) or `/dev/cu.usbserial-XXXX` (clone with CH340).
   Write it in the results sheet.

**Pass:** LEDs behave as described, no motor moves on power-up, port is visible.
**If no port appears:** try another cable / USB port; for a CH340 clone, check that the CH340 driver is installed.

---

## Phase C: Upload the firmware

1. Arduino IDE → open `motor_controller/motor_controller.ino`.
2. Tools → Board → **Arduino Uno**; Tools → Port → the port from Phase B.
3. Click **Upload**. Wait for "Done uploading".
4. **Close the Serial Monitor** if it is open.

**Pass:** upload succeeds, the motors do not move afterwards (the watchdog keeps them stopped with no commands).

**Optional quick check with Serial Monitor** (115200 baud, line ending **Newline**): type `M,150,150` and press Enter.
All wheels turn forward for only about **0.5 s** and stop. That short burst is expected: it is the watchdog.
The monitor also shows a stream of `D,<cm>` lines: that is the ultrasonic distance, sent after every ping
(`999` = no echo). `follower.py` uses it for the minimum-distance back-off.
Close the Serial Monitor before Phase D.

---

## Phase D: Motor tests with `motor_test.py` (wheels off the ground)

```bash
python motor_test.py --port /dev/cu.usbmodemXXXX
```
The script re-sends each command 20 times a second, so the motors run for 2 s per test.
"Forward" means the direction the car would drive towards the camera side (the front).

| # | Test | Expected | If wrong | Result |
| --- | --- | --- | --- | --- |
| D1 | `1` LEFT forward | Only the **left** wheels spin, both **forward** | Right side spins → swap the left and right motor pairs (OUT1/2 ↔ OUT3/4). Backward → swap OUT1 and OUT2. | |
| D2 | `2` RIGHT forward | Only the **right** wheels spin, both **forward** | Backward → swap OUT3 and OUT4. | |
| D3 | `3` Both forward | All 4 wheels forward | One wheel backward → swap that single motor's two wires. | |
| D4 | `4` Both backward | All 4 wheels backward | Same as D3. | |
| D5 | `5` Spin right | Left side forward, right side backward | Re-check D1/D2. | |
| D6 | `6` Spin left | Left side backward, right side forward | Re-check D1/D2. | |
| D7 | `7` Speed sweep | Note the PWM printed when **both** sides start turning | — | PWM = ___ |
| D8 | `10` Ultrasonic readout | Distance in cm 5 times a second; matches a tape measure within a few cm on a flat board | "no reading" → re-upload the sketch, check TRIG/ECHO wiring | |

After D7: set `MIN_SPEED` in `control.py` to the D7 value + 10.
Also note if one side is clearly slower than the other (the car will drift; we can add a trim later).

**Pass:** D1–D6 and D8 correct, D7 value recorded.

---

## Phase E: Safety tests (wheels off the ground)

These are the most important tests. **Do not go to the floor until all of them pass.**

| # | Test | How | Expected | Result |
| --- | --- | --- | --- | --- |
| E1 | Watchdog | `motor_test.py`, test `8` | Wheels stop about **0.5 s** after "Python is now silent" | |
| E2 | USB cable pull | Run test `9` (10 s forward) and pull the USB cable | Wheels stop within about 0.5 s | |
| E3 | Obstacle stop | Test `9`; hold a book **15 cm** in front of the sensor | Wheels stop; remove the book → wheels start again | |
| E4 | Obstacle distance | Test `9`; book at **25 cm**, then slowly closer | Wheels keep turning at 25 cm, stop at about **20 cm** | stops at ___ cm |
| E5 | Backward allowed | Hold the book at 10 cm, run test `4` (backward) | Wheels **do** spin backward (only forward is blocked, so the car can back away) | |
| E6 | Emergency stop | Test `9`, flip the e-stop switch OFF | Wheels stop immediately | |
| E7 | Python crash | Run `follower.py --port ...` while locked and wheels spinning; in a 2nd terminal run `pkill -9 -f follower.py` | Wheels stop within about 0.5 s | |

Notes on E3/E4: the HC-SR04 sees hard flat surfaces best. Soft clothing, thin chair legs and objects at a
steep angle may not be detected. That is a known sensor limitation, not a code bug.

**Pass:** E1–E7 all pass. If E1, E2 or E7 fails, re-upload the sketch and repeat; never skip these.

---

## Phase F: Vision + Arduino, tethered (wheels off the ground)

Mount the USB webcam on the front of the car, facing forward. Plug the webcam and the Arduino into the Mac.

```bash
python follower.py --port /dev/cu.usbmodemXXXX --source 1
```
(`--source 1` is the USB webcam; if the window shows the Mac's own camera, try `--source 2`.)

| # | Test | Expected | If wrong | Result |
| --- | --- | --- | --- | --- |
| F1 | Raise one hand for 1 s | `[LOCK]` in the log, green OWNER box, state FOLLOW | See the gesture tips in CLAUDE.md | |
| F2 | Stand far (2–3 m), centred | Both sides forward, `L ≈ R` | — | |
| F3 | Stand closer than `TARGET_DIST` | Wheels stop, `L=0 R=0` | Adjust `TARGET_TORSO` (Phase G) | |
| F4 | Stand to the car's **right** | Left wheels faster than right → car would turn **towards** you | Turns away → swap the left and right motor pairs at the L298N, then redo D1–D6 | |
| F5 | Stand to the car's **left** | Right wheels faster than left | Same as F4 | |
| F6 | Step out of view | `[LOST]`, wheels stop within about 0.3 s | — | |
| F7 | Step back in | `[FOUND]` or `[RE-LOCK]`, wheels resume | Lower `REID_THRESH` slightly | |
| F8 | Both hands up 1.5 s | Wheels pause, then `[UNLOCK]`, state IDLE, wheels stopped | — | |
| F9 | Second person walks past | Their box stays grey, the wheels keep reacting only to you | — | |
| F10 | Book in front of the sensor while you stand far | Wheels stop although `L`/`R` on screen are positive (Arduino overrides) | — | |

**Pass:** F1–F10 pass.

---

## Phase G: Floor tests and tuning (car on the floor)

Set `MAX_SPEED = 150` in `control.py` first. Open space, even lighting, e-stop person ready.

1. **Calibrate distance:** follow `plans/distance_plan.md` Phase 3.
2. **Straight follow:** walk slowly straight away from the car, then stop. The car follows and stops at about 1 m.
3. **Turning:** walk in a wide curve.
   - Car weaves left/right → lower `K_TURN` (e.g. 160 → 120).
   - Car turns too slowly / loses you on curves → raise `K_TURN`.
4. **Speed:**
   - Car lunges or overshoots and bumps towards you → lower `K_FWD` (e.g. 1000 → 700).
   - Car falls behind → raise `K_FWD`, then `MAX_SPEED` (max 200 for now).
5. **Obstacle on the floor:** place a box between you and the car while it follows → it stops before the box.
6. **Full demo run:** lock → walk around → second person crosses → step out and back → unlock.
7. Check FPS in the `[STATUS]` log lines (should stay ≥ 10).

Change **one** constant at a time and write every change in the results sheet.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| No serial port appears | Power-only USB cable, or missing CH340 driver | Other cable; install driver |
| `Resource busy` / port error in Python | Serial Monitor or another script has the port | Close Serial Monitor / stop the other script |
| Upload fails | Wrong board or port, or Python is using the port | Select Uno + correct port; stop Python |
| Motors never move | Battery off, ENA/ENB caps still on and D5/D10 not wired, missing common GND | Check in that order |
| Motors only twitch for 0.5 s | Commands sent only once (Serial Monitor) | Expected; use `motor_test.py` |
| One side never moves | Loose motor wire or EN pin wire | Re-seat wires on that side |
| Motors stutter while following | FPS too low (watchdog stops between frames) | Use `--imgsz 320`, close other apps |
| Arduino resets when motors start | Battery voltage drops (weak/low battery) or no common GND | Charge battery; check GND |
| Car never moves forward | Ultrasonic reading < 20 cm all the time | Check TRIG/ECHO wiring, nothing touching the sensor |
| Car turns away from the owner | Left/right motor pairs swapped | Swap the pairs at the L298N |

---

## Results sheet

Date: __________  Testers: __________________

| Item | Value |
| --- | --- |
| Arduino port | |
| Battery voltage (full) | V |
| Phase A wiring | pass / fail |
| Phase B power-up | pass / fail |
| Phase C upload | pass / fail |
| Phase D motors (D1–D6) | pass / fail |
| D7 start PWM → `MIN_SPEED` | |
| Phase E safety (E1–E7) | pass / fail |
| E4 obstacle stop distance | cm |
| Phase F tethered (F1–F10) | pass / fail |
| `TARGET_TORSO` (at `TARGET_DIST`) | |
| `K_FWD` / `K_TURN` / `MAX_SPEED` final | |
| FPS during floor test | |
| Notes / problems | |
