"""
Follow control for the owner-following robot: safe distance, steering and dynamic speed.
Pure functions (no camera, no serial) so they can be tested offline with test_follow.py.
"""
from collections import deque

import numpy as np

# ---- Tuning (adjust during testing) ----
TARGET_DIST = 1.0        # m, wanted gap from camera lens to owner (set in plans/distance_plan.md Phase 0)
TARGET_TORSO = 0.30      # shoulder-to-hip length / frame height at TARGET_DIST; fallback only, see LOCK_TARGET
LOCK_TARGET = True       # use the owner's torso size at lock time as the target (lock from ~TARGET_DIST away!)
TARGET_MIN, TARGET_MAX = 0.12, 0.80   # sane range for a lock-time target (outside it -> TARGET_TORSO)
DEADBAND = 0.03          # once stopped, start again only when the torso is this much smaller (~10 % of TARGET_TORSO)
SIZE_SMOOTH = 0.3        # 0..1, weight of the newest frame in the smoothed torso size
CENTER_DEADBAND = 0.10   # no turning when owner is this close to the frame centre
K_FWD = 1000             # forward gain (torso sizes are about half of box sizes)
K_TURN = 160             # turning gain
MAX_SPEED = 200          # PWM limit (0..255)
MIN_SPEED = 70           # PWM below which the motors stall
ROBOT_TOP_SPEED = 0.5    # m/s, robot ground speed at MAX_SPEED; measure it (plans/speed_plan.md step 1)!
VEL_WINDOW = 0.5         # s of gap history used to estimate the owner's speed
OWNER_MOVING = 0.15      # m/s, owner speeds below this count as standing still (noise)
VEL_GAIN = 0.8           # 0..1, share of the owner's pace matched by the speed term (below 1 = no overshoot)
RAMP_UP = 250            # PWM/s, max forward speed increase (MIN_SPEED -> 200 in ~0.5 s)
RAMP_DOWN = 500          # PWM/s, max forward speed decrease while still driving (full stops are instant)
OUTRUN_DIST = 2.5        # m, gap that counts as "owner got away"
OUTRUN_TIME = 1.5        # s at near-full speed with gap > OUTRUN_DIST before giving up
MIN_DIST_CM = 60         # cm, back off when the centred owner is closer than this (sonar)
BACKOFF_CLEAR_CM = 75    # cm, stop backing off once the owner is this far (hysteresis)
BACKOFF_SPEED = 80       # PWM, slow reverse (just above MIN_SPEED)
BACKOFF_MAX_TIME = 2.0   # s, longest reverse per episode: there is no rear sensor
BACKOFF_CENTER = 0.25    # |err_x| below this counts as "owner in front of the sonar"
SONAR_MAX_AGE = 0.3      # s, older sonar readings are ignored
SEARCH_SPEED = 90        # PWM per side for in-place rotation (slow; just above MIN_SPEED)
SPIN_DEG_PER_S = 90.0    # deg/s at SEARCH_SPEED on the demo floor; measure it (plans/search_plan.md S0)!
SEARCH_STEP_DEG = 30     # deg turned between looks (about half the camera's horizontal view)
SEARCH_LOOK = 0.7        # s standing still after each step so detection and re-ID see sharp frames
SEARCH_CONFIRM = 3       # consecutive frames a clothing match must hold before re-locking in SEARCH


def shape_speed(v):
    v = max(-MAX_SPEED, min(MAX_SPEED, v))
    if abs(v) < 20:
        return 0
    return int(np.sign(v) * max(abs(v), MIN_SPEED))


def est_distance(size, target=TARGET_TORSO):
    """Rough distance in metres from the torso size (size ∝ 1 / distance; `target` is the size at TARGET_DIST).
    Control only uses its rate of change: a calibration error scales the rate but never flips its sign."""
    return TARGET_DIST * target / size if size else None


def lock_target(size):
    """Follow target from the owner's torso size at lock time, or TARGET_TORSO if unusable / disabled."""
    if LOCK_TARGET and size is not None and TARGET_MIN <= size <= TARGET_MAX:
        return size
    return TARGET_TORSO


def gap_rate(history):
    """How fast the gap grows (m/s) from a list of (time, distance) pairs, or None if too little data."""
    history = list(history)
    if len(history) < 3:
        return None
    (t0, d0), (t1, d1) = history[0], history[-1]
    if t1 - t0 < 0.6 * VEL_WINDOW:
        return None
    return (d1 - d0) / (t1 - t0)


def owner_speed(rate, fwd_prev):
    """Owner's speed away from the camera (m/s): the camera sees owner minus robot, so add the robot's speed."""
    if rate is None:
        return None
    return rate + ROBOT_TOP_SPEED * fwd_prev / MAX_SPEED


def pace_term(v_owner):
    """Forward PWM that matches the owner's walking pace; 0 when standing still or walking towards us."""
    if v_owner is None or v_owner <= OWNER_MOVING:
        return 0.0
    return VEL_GAIN * MAX_SPEED * v_owner / ROBOT_TOP_SPEED


def ramp_fwd(prev, target, dt):
    """Limit how fast the forward speed changes. Stopping is never ramped."""
    if target <= 0:
        return 0.0
    if prev < MIN_SPEED:
        return float(MIN_SPEED)                     # start at MIN_SPEED so the motors don't stall
    return prev + max(-RAMP_DOWN * dt, min(RAMP_UP * dt, target - prev))


def mix(fwd, turn):
    """Forward + turn -> (left, right) motor values."""
    return shape_speed(fwd + turn), shape_speed(fwd - turn)


def arc(fwd, turn):
    """While driving forward, limit the turn so the inner wheels slow down but never reverse:
    the robot curves towards the owner instead of spinning on the spot."""
    if fwd > 0:
        return max(-fwd, min(fwd, turn))
    return turn


def centre_error(box, W):
    """Owner's horizontal offset from the frame centre: -1 (left) .. +1 (right)."""
    x1, _, x2, _ = box
    return ((x1 + x2) / 2 - W / 2) / (W / 2)


def follow_command(box, W, size, driving, v_owner=None, target=TARGET_TORSO):
    """Steer toward the owner; drive until the torso is `target` tall, faster when the owner walks away.
    `driving` is the previous decision: once stopped, start again only when the owner is DEADBAND farther.
    Returns (forward target, turn, driving)."""
    err_x = centre_error(box, W)
    deadband = DEADBAND * target / TARGET_TORSO     # same ~10 % hysteresis for any target
    if size is None:
        driving = False                             # torso not visible: steer only, never drive blind
    elif driving:
        driving = size < target
    else:
        driving = size < target - deadband
    fwd = 0.0
    if driving:
        fwd = min(MAX_SPEED, max(K_FWD * (target - size), MIN_SPEED) + pace_term(v_owner))
    turn = K_TURN * err_x if abs(err_x) > CENTER_DEADBAND else 0.0
    return fwd, turn, driving


def parse_distance(buf):
    """Pull "D,<cm>" lines out of received serial text. Returns (last cm or None, unfinished last line)."""
    *lines, rest = buf.split("\n")
    cm = None
    for line in lines:
        line = line.strip()
        if line.startswith("D,") and line[2:].isdigit():
            cm = int(line[2:])
    return cm, rest


def update_backoff(fs, us_cm, centred, cam_dist, now):
    """Decide whether to reverse away from an owner who is too close. Returns True while reversing.
    Only when the owner is centred (in the sonar beam) and the camera doesn't say they are far
    (then the sonar sees an obstacle). Reversing is blind, so it lasts at most BACKOFF_MAX_TIME;
    after that it waits until the owner has stepped back past BACKOFF_CLEAR_CM."""
    if fs.backoff_wait:
        if us_cm is not None and us_cm >= BACKOFF_CLEAR_CM:
            fs.backoff_wait = False
        return False
    near = us_cm is not None and centred and (cam_dist is None or cam_dist < TARGET_DIST)
    if fs.backing:
        if near and us_cm < BACKOFF_CLEAR_CM:
            if now - fs.backoff_start < BACKOFF_MAX_TIME:
                return True
            fs.backoff_wait = True
        fs.backing = False
        return False
    if near and us_cm < MIN_DIST_CM:
        fs.backing, fs.backoff_start = True, now
        return True
    return False


class FollowState:
    """Follow memory for the current owner track; replaced on lock, re-lock, lost and unlock."""

    def __init__(self):
        self.size_s = None          # smoothed torso size
        self.driving = False        # hysteresis flag
        self.dist_hist = deque()    # (time, estimated gap in m) over the last VEL_WINDOW s
        self.fwd = 0.0              # ramped forward PWM actually commanded
        self.v_owner = None         # m/s, owner's speed away from the camera
        self.outrun_since = None    # time the outrun condition started
        self.backing = False        # reversing away from an owner who is too close
        self.backoff_start = 0.0    # time the current back-off started
        self.backoff_wait = False   # back-off time limit hit: wait until the owner steps back


class Search:
    """Stop-and-look scan for a lost owner: turn SEARCH_STEP_DEG in place, stand still SEARCH_LOOK s, repeat
    until 360 deg. No encoders, so the angle comes from time x SPIN_DEG_PER_S."""

    def __init__(self, direction, now):
        self.dir = 1 if direction >= 0 else -1     # +1 = turn right (clockwise from above)
        self.turned = 0
        self.phase, self.phase_start = "look", now  # look first: check the current view while stopped

    def step(self, now, hold=False):
        """Returns (turn PWM, done). `hold` = a possible owner is being confirmed: stand still and look."""
        if hold:
            self.phase, self.phase_start = "look", now
            return 0, False
        if self.phase == "look" and now - self.phase_start >= SEARCH_LOOK:
            if self.turned >= 360:
                return 0, True
            self.phase, self.phase_start = "turn", now
        elif self.phase == "turn" and now - self.phase_start >= SEARCH_STEP_DEG / SPIN_DEG_PER_S:
            self.turned += SEARCH_STEP_DEG
            self.phase, self.phase_start = "look", now
        return (self.dir * SEARCH_SPEED if self.phase == "turn" else 0), False

    def progress(self):
        return self.turned


# ---- State machine: the only allowed state changes (follower.change_state checks every change) ----
STATES = ("IDLE", "FOLLOW", "SEARCH", "LOST")
TRANSITIONS = {
    ("IDLE", "FOLLOW"): "lock",
    ("FOLLOW", "IDLE"): "unlock",
    ("FOLLOW", "SEARCH"): "owner lost",
    ("SEARCH", "FOLLOW"): "owner found",
    ("SEARCH", "IDLE"): "search gave up",
    ("FOLLOW", "LOST"): "outrun / lost with --no-search",
    ("LOST", "FOLLOW"): "owner back",
    ("LOST", "SEARCH"): "outran owner left view",
    ("LOST", "IDLE"): "unlock",
}


def transition_allowed(old, new):
    return old == new or (old, new) in TRANSITIONS
