#!/usr/bin/env python3
"""Offline tests for the safe-distance follow and dynamic speed helpers.
Run: python test_follow.py   (no camera, no robot)
"""
import numpy as np

from control import (BACKOFF_CLEAR_CM, BACKOFF_MAX_TIME, DEADBAND, MAX_SPEED, MIN_SPEED, OWNER_MOVING,
                     ROBOT_TOP_SPEED, SEARCH_LOOK, SEARCH_SPEED, SEARCH_STEP_DEG, SPIN_DEG_PER_S, STATES,
                     TARGET_DIST, TARGET_TORSO, TRANSITIONS, VEL_GAIN, FollowState, Search, arc,
                     est_distance, follow_command, lock_target, gap_rate, mix, owner_speed, pace_term, parse_distance, ramp_fwd,
                     transition_allowed, update_backoff)
import control
from control import forward_only
from perception import L_HIP, L_SHOULDER, R_HIP, R_SHOULDER, best_match, torso_hist, torso_ratio

W, H = 640, 500
CENTRED = (W / 2 - 50, 100, W / 2 + 50, 400)
FAR_RIGHT = (W - 80, 100, W - 10, 400)


def fake_person(shoulder_y, hip_y, conf=0.9, hip_conf=None):
    k = np.zeros((17, 3))
    k[L_SHOULDER] = (300, shoulder_y, conf)
    k[R_SHOULDER] = (340, shoulder_y, conf)
    k[L_HIP] = (305, hip_y, conf if hip_conf is None else hip_conf)
    k[R_HIP] = (335, hip_y, conf if hip_conf is None else hip_conf)
    return k


def drive(box, size, driving, v_owner=None):
    """follow_command + mix without ramping -> (left, right, driving)."""
    fwd, turn, driving = follow_command(box, W, size, driving, v_owner)
    return (*mix(fwd, turn), driving)


def close(a, b):
    return abs(a - b) < 1e-9


def check(name, cond):
    assert cond, f"{name} FAILED"
    print(f"{name} OK")


# ---- safe-distance follow ----
check("T1 torso ratio", close(torso_ratio(fake_person(100, 250), H), 0.30))
check("T2 hips low confidence", torso_ratio(fake_person(100, 250, hip_conf=0.1), H) is None)
check("T3 hips above shoulders", torso_ratio(fake_person(250, 100), H) is None)

L, R, d = drive(CENTRED, 0.15, False)
check("T4 far -> drive", L > 0 and R > 0 and d)

L, R, d = drive(CENTRED, TARGET_TORSO, True)
check("T5 at target -> stop", L == 0 and R == 0 and not d)

L, R, d = drive(CENTRED, TARGET_TORSO - 0.01, False)
check("T6 hysteresis stays stopped", L == 0 and R == 0 and not d)

L, R, d = drive(CENTRED, TARGET_TORSO - DEADBAND - 0.01, False)
check("T7 hysteresis drives", L > 0 and R > 0 and d)

L, R, d = drive(CENTRED, TARGET_TORSO - 0.01, True)
check("T8 keeps driving at MIN_SPEED", L == R == MIN_SPEED and d)

L, R, d = drive(FAR_RIGHT, None, True)
check("T9 torso hidden, owner right -> turn only", L > 0 and R < 0 and L == -R and not d)

L, R, d = drive(CENTRED, None, True)
check("T10 torso hidden, centred -> stop", L == 0 and R == 0 and not d)

check("T11 distance estimate", close(est_distance(TARGET_TORSO), TARGET_DIST)
      and close(est_distance(TARGET_TORSO / 2), 2 * TARGET_DIST))

# ---- dynamic speed ----
check("T12 gap rate", close(gap_rate([(0.0, 1.0), (0.2, 1.1), (0.4, 1.2)]), 0.5)
      and gap_rate([(0.0, 1.0), (0.1, 1.05), (0.2, 1.1)]) is None
      and gap_rate([(0.0, 1.0), (0.4, 1.2)]) is None)

check("T13 owner speed adds robot speed", close(owner_speed(0.0, MAX_SPEED), ROBOT_TOP_SPEED)
      and owner_speed(None, MAX_SPEED) is None)

check("T14 pace term", pace_term(OWNER_MOVING / 2) == 0 and pace_term(-0.5) == 0 and pace_term(None) == 0
      and close(pace_term(ROBOT_TOP_SPEED), VEL_GAIN * MAX_SPEED))

size = TARGET_TORSO - DEADBAND - 0.01
slow, _, _ = follow_command(CENTRED, W, size, False, None)
walk, _, _ = follow_command(CENTRED, W, size, False, 0.4)
fast, _, _ = follow_command(CENTRED, W, size, False, 5.0)
check("T15 walking owner -> faster, capped", walk > slow and fast == MAX_SPEED)

fwd, _, d = follow_command(CENTRED, W, TARGET_TORSO, True, 0.5)
check("T16 still stops at target while owner walks", fwd == 0 and not d)

first = ramp_fwd(0.0, 200, 0.1)
check("T17 ramp up from stop", first == MIN_SPEED and close(ramp_fwd(first, 200, 0.1), MIN_SPEED + control.RAMP_UP * 0.1))

check("T18 stop is instant", ramp_fwd(180, 0, 0.1) == 0)

check("T19 ramp down while driving", close(ramp_fwd(200, 100, 0.1), 150))

# ---- steering and minimum distance ----
_, turn, _ = follow_command((W / 2 - 40, 100, W / 2 + 60, 400), W, TARGET_TORSO, False)
check("T20 steering deadband", turn == 0)

L, R, _ = drive((10, 100, 80, 400), TARGET_TORSO, False)
check("T21 owner left -> turn left", R > L)

check("T22 parse distance", parse_distance("D,45\nD,4") == (45, "D,4")
      and parse_distance("junk\nD,x\nD,30\nhello\n") == (30, "")
      and parse_distance("D,999\n") == (999, "") and parse_distance("D,1") == (None, "D,1"))

fs = FollowState()
control.BACKOFF_ENABLED = True   # T23-T28 test the (currently disabled) back-off logic itself
NEAR_CM = control.MIN_DIST_CM - 10
check("T23 back-off starts", update_backoff(fs, NEAR_CM, True, None, 0.0))
check("T24 not centred -> no back-off", not update_backoff(FollowState(), NEAR_CM, False, None, 0.0))
check("T25 camera says far -> obstacle, no back-off", not update_backoff(FollowState(), NEAR_CM, True, 2.0, 0.0))
check("T26 no sonar reading -> no back-off", not update_backoff(FollowState(), None, True, None, 0.0))

fs = FollowState()
update_backoff(fs, NEAR_CM, True, None, 0.0)
check("T27 hysteresis", update_backoff(fs, BACKOFF_CLEAR_CM - 5, True, None, 0.5)
      and not update_backoff(fs, BACKOFF_CLEAR_CM + 5, True, None, 0.6))

fs = FollowState()
update_backoff(fs, NEAR_CM, True, None, 0.0)
t = BACKOFF_MAX_TIME + 0.1
check("T28 time cap, wait for step back",
      not update_backoff(fs, NEAR_CM, True, None, t)
      and not update_backoff(fs, NEAR_CM, True, None, t + 0.5)
      and not update_backoff(fs, BACKOFF_CLEAR_CM + 5, True, None, t + 1.0)
      and update_backoff(fs, NEAR_CM, True, None, t + 1.5))

# ---- owner search ----
TURN_T = SEARCH_STEP_DEG / SPIN_DEG_PER_S
EPS = 1e-3

s = Search(+1, 0.0)
check("T29 search starts with a still look", s.step(0.0) == (0, False) and s.step(SEARCH_LOOK - EPS) == (0, False))

right = Search(+1, 0.0).step(SEARCH_LOOK)
left = Search(-1, 0.0).step(SEARCH_LOOK)
check("T30 turn direction", right == (SEARCH_SPEED, False) and left == (-SEARCH_SPEED, False))

s = Search(+1, 0.0)
s.step(SEARCH_LOOK)                                   # look done -> turn
mid = s.step(SEARCH_LOOK + TURN_T - EPS)
end = s.step(SEARCH_LOOK + TURN_T + EPS)
check("T31 one step = SEARCH_STEP_DEG", mid[0] == SEARCH_SPEED and s.progress() == SEARCH_STEP_DEG and end[0] == 0)

s, t, results = Search(-1, 0.0), 0.0, []
for _ in range(360 // SEARCH_STEP_DEG):
    t += SEARCH_LOOK + EPS
    results.append(s.step(t))                         # start turning
    t += TURN_T + EPS
    results.append(s.step(t))                         # step done, look
t += SEARCH_LOOK + EPS
final = s.step(t)
check("T32 done after exactly 360 deg", not any(d for _, d in results) and s.progress() == 360
      and final == (0, True))

s = Search(+1, 0.0)
s.step(SEARCH_LOOK)
held = s.step(SEARCH_LOOK + TURN_T / 2, hold=True)
after = s.step(SEARCH_LOOK + TURN_T)                  # look restarted at the hold, so still looking
check("T33 hold stops the turn", held == (0, False) and s.phase == "look" and s.progress() == 0
      and after == (0, False))


def solid(bgr):
    return np.full((200, 200, 3), bgr, dtype=np.uint8)


owner_hist = torso_hist(solid((0, 0, 200)), (0, 0, 200, 200))     # red shirt
frame = np.hstack([solid((0, 0, 200)), solid((200, 0, 0))])         # red person left, blue person right
people = [(7, (200, 0, 400, 200), None), (8, (0, 0, 200, 200), None)]
best, score = best_match(frame, people, owner_hist)
check("T34 best clothing match", best[0] == 8 and score > 0.9
      and best_match(frame, [], owner_hist) == (None, None))

# ---- state machine ----
EDGES = [("IDLE", "FOLLOW"), ("FOLLOW", "IDLE"), ("FOLLOW", "SEARCH"), ("SEARCH", "FOLLOW"),
         ("SEARCH", "IDLE"), ("FOLLOW", "LOST"), ("LOST", "FOLLOW"), ("LOST", "SEARCH"), ("LOST", "IDLE")]
check("T35 every designed change is allowed", all(transition_allowed(a, b) for a, b in EDGES)
      and all(transition_allowed(s, s) for s in STATES) and set(TRANSITIONS) == set(EDGES))

check("T36 illegal changes rejected", not any(transition_allowed(a, b) for a, b in
      [("IDLE", "SEARCH"), ("IDLE", "LOST"), ("SEARCH", "LOST"), ("FOLLOW", "SPIN")]))


def reachable(start, edges):
    seen, todo = {start}, [start]
    while todo:
        s = todo.pop()
        for a, b in edges:
            if a == s and b not in seen:
                seen.add(b)
                todo.append(b)
    return seen


check("T37 all states reachable from IDLE, and IDLE reachable from all",
      reachable("IDLE", TRANSITIONS) == set(STATES)
      and all("IDLE" in reachable(s, TRANSITIONS) for s in STATES))

# ---- move towards the owner (lock-time target, curve instead of spin) ----
check("T38 lock target from owner's size, scaled to TARGET_DIST",
      close(lock_target(0.2), min(control.SIZE_CAP, 0.2 * control.LOCK_DIST / TARGET_DIST))
      and lock_target(None) == TARGET_TORSO and lock_target(0.95) == TARGET_TORSO)
fwd, _, d = follow_command(CENTRED, W, 0.40, False, None, target=0.55)
check("T39 owner farther than at lock -> drive forward", fwd >= MIN_SPEED and d)
fwd, _, d = follow_command(CENTRED, W, 0.56, True, None, target=0.55)
check("T40 owner at lock distance -> stop", fwd == 0 and not d)
check("T41 distance uses lock target", close(est_distance(0.275, 0.55), 2 * TARGET_DIST))
fwd, turn, _ = follow_command(FAR_RIGHT, W, 0.40, False, None, target=0.55)
L, R = mix(fwd, arc(fwd, turn))
check("T42 owner off-centre while driving -> curve, no wheel reverses (STRAIGHT_DRIVE off)", L > R >= 0)
check("T43 not driving -> pivot still allowed", arc(0, 80) == 80)

# ---- forward-only following (no wheel ever reverses) ----
check("T44 pivot becomes one-side turn", forward_only(70, -70) == (control.TURN_MIN, 0)
      and forward_only(-70, 70) == (0, control.TURN_MIN))
check("T45 forward unchanged, reverse blocked", forward_only(150, 120) == (150, 120) and forward_only(-80, -80) == (0, 0))
control.BACKOFF_ENABLED = False
check("T46 back-off disabled -> never reverses", not update_backoff(FollowState(), 30, True, 0.5, 0.0))

# ---- drive towards the owner when the torso is hidden (box width), no wiggling ----
from control import BOX_TARGET, box_target, box_width
check("T47 box width", close(box_width((100, 0, 260, 480), W), 0.25))
check("T48 box target learned or fallback",
      close(box_target(0.3), min(control.SIZE_CAP, 0.3 * control.LOCK_DIST / TARGET_DIST)) and box_target(None) == BOX_TARGET
      and box_target(0.95) == BOX_TARGET)
fwd, turn, d = follow_command(CENTRED, W, 0.20, False, None, target=0.40)
check("T49 torso hidden but box small (owner far) -> drive forward", fwd >= MIN_SPEED and d)
_, turn, d = follow_command((W / 2 + 10, 100, W / 2 + 130, 400), W, 0.45, False, None, target=0.40)
check("T50 stopped, owner slightly off-centre -> no spin", turn == 0 and not d)
fwd, turn, d = follow_command((W / 2 + 10, 100, W / 2 + 130, 400), W, 0.20, False, None, target=0.40)
L, R = forward_only(*mix(fwd, arc(fwd, turn)))
check("T51 driving, owner slightly right -> curve right, both forward (STRAIGHT_DRIVE off)", L > R > 0)

from control import box_cut_off
check("T52 box cut off at frame edge -> width not trusted", box_cut_off((0, 0, 300, 480), W)
      and box_cut_off((400, 0, W, 480), W) and not box_cut_off(CENTRED, W))
check("T53 follow distance is short", TARGET_DIST <= 0.5)

# ---- straight drive: equal PWM both sides ----
from control import straight_or_turn
f, tr = straight_or_turn(100, 30, 0.15)
check("T54 driving, owner nearly ahead -> straight, equal PWM", f == 100 and tr == 0
      and mix(f, tr)[0] == mix(f, tr)[1])
f, tr = straight_or_turn(100, 60, 0.5)
check("T55 owner clearly to one side -> stop, then turn", f == 0 and tr == 60)
check("T56 stopped, small offset -> nothing", straight_or_turn(0, 0, 0.1) == (0, 0))

# ---- --straight-only hard mode: forward / stop / slow reverse, camera decides, sonar can veto ----
from control import REVERSE_MARGIN, REVERSE_MAX_TIME, straight_reverse
fs = FollowState()
check("T57 owner at target -> no reverse", not straight_reverse(fs, 0.60, 0.60, None, 0.0))
check("T58 owner clearly closer -> reverse", straight_reverse(fs, 0.60 * (1 + REVERSE_MARGIN) + 0.05, 0.60, None, 0.0))
fs = FollowState()
check("T59 sonar says nothing near -> veto", not straight_reverse(fs, 0.80, 0.60, 120, 0.0))
check("T60 sonar no echo (999) ignored", straight_reverse(fs, 0.80, 0.60, 999, 0.0))
fs = FollowState()
straight_reverse(fs, 0.80, 0.60, None, 0.0)
check("T61 reverse capped, then waits for step back",
      not straight_reverse(fs, 0.80, 0.60, None, REVERSE_MAX_TIME + 0.1)
      and not straight_reverse(fs, 0.80, 0.60, None, REVERSE_MAX_TIME + 1.0)
      and not straight_reverse(fs, 0.55, 0.60, None, REVERSE_MAX_TIME + 1.5)
      and straight_reverse(fs, 0.80, 0.60, None, REVERSE_MAX_TIME + 2.0))
check("T62 owner not measurable -> no reverse", not straight_reverse(FollowState(), None, 0.60, 30, 0.0))
check("T63 straight-only output is equal both sides", mix(-75, 0)[0] == mix(-75, 0)[1] == -75
      and mix(90, 0)[0] == mix(90, 0)[1] == 90)

# ---- calm turn: only after a full stop, short nudges ----
from control import STOP_BEFORE_TURN, TURN_LOOK, TURN_STEP_MAX, CalmTurn
c = CalmTurn()
check("T64 driving -> never turns", c.step(0.0, True, 0.8) == 0)
c = CalmTurn()
t = 0.0
while t < STOP_BEFORE_TURN - 0.05:
    assert c.step(t, False, 0.8) == 0, "turned before the full-stop wait"
    t += 0.05
check("T65 no turn during the first second of a stop", True)
turns = []
while t < STOP_BEFORE_TURN + TURN_LOOK + TURN_STEP_MAX + TURN_LOOK + 0.3:
    turns.append(c.step(t, False, 0.8))
    t += 0.05
check("T66 then nudges towards the owner (right), with still phases",
      any(v > 0 for v in turns) and turns.count(0) > 0 and not any(v < 0 for v in turns))
check("T67 owner roughly ahead -> no turn", CalmTurn().step(5.0, False, 0.1) == 0)
c = CalmTurn()
for i in range(60):
    c.step(i * 0.05, False, -0.8)
check("T68 owner left -> turns left", any(c.step(3.0 + i * 0.05, False, -0.8) < 0 for i in range(30)))

c = CalmTurn()
t, turning = 0.0, 0
while t < 6.0:
    turning += c.step(t, False, 1.0) != 0
    t += 0.01
check("T69 nudges are short (<= TURN_STEP_MAX each, mostly still)", turning * 0.01 <= 6.0 * TURN_STEP_MAX / TURN_LOOK)
check("T70 owner only slightly off (0.35) -> no new nudge", not any(CalmTurn().step(i * 0.05, False, 0.35)
                                                                     for i in range(100)))

print("All tests passed.")
