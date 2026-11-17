#!/usr/bin/env python3
"""
Owner-following robot (DesiPachinos)
Features: owner LOCK, UNLOCK, RE-LOCK, SAFE-DISTANCE FOLLOW with dynamic speed and back-off,
OWNER SEARCH (rotate in place to find a lost owner; disable with --no-search).

Gestures (hold them still):
  - Raise ONE hand above your head for ~1 s  -> robot locks onto you
  - Raise BOTH hands above your head for ~1.5 s -> robot unlocks and goes idle

States (allowed changes: TRANSITIONS in control.py; every change is logged as "[STATE] old -> new (reason)"):
  IDLE    waiting for a lock gesture, motors stopped
  FOLLOW  following the owner
  SEARCH  owner lost: turning in place to find them, then back to FOLLOW or IDLE
  LOST    stopped: owner outran the robot (or lost with --no-search)

Run on a laptop (no robot):    python follower.py
Run with the Arduino attached: python follower.py --port /dev/tty.usbmodem1101   (Mac)
                               python follower.py --port /dev/ttyACM0            (Pi / Jetson)
Press q in the video window to quit. Logs go to the terminal and to follower.log.
"""
import argparse
import logging
import time

import cv2
from ultralytics import YOLO

from control import (BACKOFF_CENTER, BACKOFF_SPEED, MAX_SPEED, MIN_DIST_CM, OUTRUN_DIST, OUTRUN_TIME,
                     SEARCH_CONFIRM, SIZE_SMOOTH, SONAR_MAX_AGE, TARGET_DIST, TARGET_TORSO, VEL_WINDOW,
                     FollowState, Search, centre_error, est_distance, follow_command, gap_rate, mix,
                     owner_speed, parse_distance, ramp_fwd, transition_allowed, update_backoff)
from perception import best_match, hands_up, torso_hist, torso_ratio

# ---- Tuning (adjust during testing) ----
LOCK_HOLD = 1.0          # s, one hand up to lock
UNLOCK_HOLD = 1.5        # s, both hands up to unlock
COOLDOWN = 2.0           # s after unlock before anyone can lock again
LOST_STOP = 0.3          # s owner missing before the robot stops
REID_THRESH = 0.75       # clothing-colour similarity needed to re-lock (0..1)
NOT_FOUND_SHOW = 3.0     # s the "OWNER NOT FOUND" banner stays up after a failed search
# Search speed, step and timing live in control.py.
# Distance, steering and speed tuning lives in control.py.

# ---- Display and logging ----
UI_SCALE = 1.0           # multiplier for all on-screen text; raise it if text is still too small
LOG_EVERY = 1.0          # s between status lines in the terminal
LOG_FILE = "follower.log"  # log file, overwritten on every run

FONT = cv2.FONT_HERSHEY_SIMPLEX
STATE_COLOURS = {"IDLE": (230, 230, 230), "FOLLOW": (0, 230, 0), "LOST": (0, 0, 255),
                 "SEARCH": (0, 200, 255)}  # BGR

log = logging.getLogger("follower")


def setup_logging():
    fmt = logging.Formatter("%(asctime)s.%(msecs)03d  %(levelname)-7s %(message)s", "%H:%M:%S")
    log.setLevel(logging.INFO)
    for handler in (logging.StreamHandler(), logging.FileHandler(LOG_FILE, mode="w")):
        handler.setFormatter(fmt)
        log.addHandler(handler)


def on_outrun(gap):
    """Owner got away at full speed but is still visible: wait in LOST. If they then leave view, SEARCH starts."""
    log.warning(f"[OUTRUN] owner faster than robot (gap ~{gap:.1f} m at full speed) -> stopped")
    return "LOST"


def change_state(old, new, reason):
    """The only way the state changes: checks TRANSITIONS, logs it, and is the hook for the status LED/buzzer.
    An illegal change is still applied (so the other variables stay consistent) but logged as ERROR."""
    if new == old:
        return old
    if not transition_allowed(old, new):
        log.error(f"[STATE] ILLEGAL {old} -> {new} ({reason}) - bug, please report the log")
    log.info(f"[STATE] {old} -> {new} ({reason})")
    return new


class Motors:
    """Sends 'M,<left>,<right>\\n' to the Arduino and reads its 'D,<cm>' sonar lines.
    Without a port it only prints/overlays."""

    def __init__(self, port):
        self.ser = None
        self.rx = ""                     # unfinished received line
        self.us_cm, self.us_time = None, 0.0
        if port:
            import serial
            self.ser = serial.Serial(port, 115200, timeout=0)
            time.sleep(2)  # Arduino resets when the port opens
            log.info(f"[SERIAL] connected to Arduino on {port}")
        else:
            log.info("[SERIAL] no --port given: laptop test mode, motor commands are only displayed")

    def send(self, left, right):
        if self.ser:
            self.ser.write(f"M,{left},{right}\n".encode())

    def read_distance(self):
        """Latest front sonar distance in cm, or None (no port, or no reading for SONAR_MAX_AGE).
        Call every frame so the receive buffer never fills up."""
        if not self.ser:
            return None
        waiting = self.ser.in_waiting
        if waiting:
            cm, self.rx = parse_distance(self.rx + self.ser.read(waiting).decode(errors="ignore"))
            self.rx = self.rx[-32:]      # junk without newlines must not grow forever
            if cm is not None:
                self.us_cm, self.us_time = cm, time.time()
        return self.us_cm if time.time() - self.us_time <= SONAR_MAX_AGE else None

    def close(self):
        self.send(0, 0)
        if self.ser:
            self.ser.close()


# ---------------------------------------------------------------- on-screen display

def text_size(text, scale, thick):
    (tw, th), base = cv2.getTextSize(text, FONT, scale, thick)
    pad = int(6 * scale) + 2
    return tw + 2 * pad, th + base + 2 * pad, pad, th


def draw_text(img, text, x, y, scale, colour, bg=(0, 0, 0), thick=2):
    """Text on a filled background box; (x, y) is the box's top-left corner. Returns box height."""
    w, h, pad, th = text_size(text, scale, thick)
    cv2.rectangle(img, (x, y), (x + w, y + h), bg, -1)
    cv2.putText(img, text, (x + pad, y + pad + th), FONT, scale, colour, thick, cv2.LINE_AA)
    return h


def draw_overlay(frame, people, owner_id, state, left, right, fps, hint, banner=None):
    """`banner` = (text, BGR background) shown big at the bottom, or None."""
    H, W = frame.shape[:2]
    s = UI_SCALE * W / 900               # text grows with the camera resolution
    t = max(2, int(round(2 * s)))

    # Boxes for every person
    for pid, box, _ in people:
        x1, y1, x2, y2 = map(int, box)
        is_owner = pid == owner_id
        colour = (0, 230, 0) if is_owner else (170, 170, 170)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, t + 2 if is_owner else t)

    # Status panel, top-left
    owner_txt = f"OWNER: id {owner_id}" if owner_id is not None else "OWNER: none"
    lines = [(f"{state}   {owner_txt}", 1.3 * s, STATE_COLOURS[state], t + 1),
             (f"L={left:+4d}  R={right:+4d}   people: {len(people)}   {fps:4.1f} FPS",
              0.9 * s, (255, 255, 255), t)]
    if hint:
        lines.append((hint, 0.8 * s, (0, 220, 255), t))
    panel_w, panel_h = 0, 0
    for text, scale, colour, thick in lines:
        panel_w = max(panel_w, text_size(text, scale, thick)[0])
        panel_h += draw_text(frame, text, 0, panel_h, scale, colour, thick=thick)

    # Name tags above each box (pushed below the status panel if they would hide under it)
    for pid, box, _ in people:
        x1, y1 = int(box[0]), int(box[1])
        is_owner = pid == owner_id
        tag = f"OWNER (id {pid})" if is_owner else f"id {pid}"
        scale = 1.1 * s if is_owner else 0.8 * s
        _, h, _, _ = text_size(tag, scale, t)
        ty = max(0, y1 - h)
        if ty < panel_h and x1 < panel_w:
            ty = panel_h
        draw_text(frame, tag, max(0, x1), ty, scale, (0, 0, 0) if is_owner else (255, 255, 255),
                  bg=(0, 230, 0) if is_owner else (170, 170, 170), thick=t)

    # Big warning at the bottom (owner lost / searching / not found)
    if banner:
        msg, bg = banner
        w, h, _, _ = text_size(msg, 1.6 * s, t + 2)
        draw_text(frame, msg, max(0, (W - w) // 2), H - h - int(20 * s), 1.6 * s, (255, 255, 255),
                  bg=bg, thick=t + 2)


# ---------------------------------------------------------------- main loop

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0", help="camera index or video file")
    ap.add_argument("--port", default=None, help="Arduino serial port (omit for laptop testing)")
    ap.add_argument("--model", default="yolov8n-pose.pt")
    ap.add_argument("--imgsz", type=int, default=320, help="320 is fast; 640 is more accurate")
    ap.add_argument("--no-show", action="store_true", help="no video window (headless robot)")
    ap.add_argument("--no-search", action="store_true",
                    help="stop and wait instead of rotating to find a lost owner")
    args = ap.parse_args()

    setup_logging()
    log.info(f"[START] model={args.model} imgsz={args.imgsz} source={args.source} "
             f"show={'no' if args.no_show else 'yes'} log file={LOG_FILE}")

    model = YOLO(args.model)
    cap = cv2.VideoCapture(int(args.source) if args.source.isdigit() else args.source)
    if not cap.isOpened():
        log.error("[CAMERA] could not open the camera. On a Mac: System Settings > Privacy & "
                  "Security > Camera > allow Terminal / VS Code, then restart it. "
                  "Or try --source 1.")
        return
    log.info(f"[CAMERA] opened {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
             f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    motors = Motors(args.port)

    state = "IDLE"            # see TRANSITIONS in control.py; change it only with change_state()
    owner_id, owner_hist = None, None
    lock_start = {}           # track id -> time one hand went up
    unlock_start = None
    last_seen = 0.0
    cooldown_until = 0.0
    seen_ids = set()
    fps, prev_t, last_log = 0.0, time.time(), 0.0
    fs = FollowState()
    outran = False            # owner outran the robot: stay stopped until they are back within OUTRUN_DIST
    reid_best = None
    last_sonar_warn = 0.0
    search = None             # Search scan while the owner is lost
    candidate = (None, 0)     # (track id, consecutive frames) of a possible owner during SEARCH
    last_err = 0.0            # owner's last horizontal offset: which way to search first
    not_found_until = 0.0     # show "OWNER NOT FOUND" until this time
    frames = 0

    log.info("[IDLE] waiting: raise ONE hand above your head to lock")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                log.error("[CAMERA] no frame from camera (camera busy, unplugged or end of video)")
                break
            H, W = frame.shape[:2]
            now = time.time()
            dt = now - prev_t
            prev_t = now
            if dt > 0:
                fps = 1 / dt if fps == 0 else 0.9 * fps + 0.1 / dt
            frames += 1
            us = motors.read_distance()

            res = model.track(frame, persist=True, tracker="botsort.yaml",
                              classes=[0], imgsz=args.imgsz, verbose=False)[0]
            people = []
            if res.boxes is not None and res.boxes.id is not None and res.keypoints is not None:
                people = list(zip(res.boxes.id.int().tolist(),
                                  res.boxes.xyxy.tolist(),
                                  res.keypoints.data.cpu().numpy()))
            for pid, _, _ in people:
                if pid not in seen_ids:
                    seen_ids.add(pid)
                    log.info(f"[TRACK] new person in view: id {pid}")

            left = right = 0
            hint = ""

            if state == "IDLE":
                if now > cooldown_until:
                    raised = set()
                    for pid, box, k in people:
                        if hands_up(k) == 1:
                            raised.add(pid)
                            if pid not in lock_start:
                                lock_start[pid] = now
                                log.info(f"[GESTURE] id {pid}: one hand up, hold {LOCK_HOLD:.1f}s to lock")
                            if now - lock_start[pid] >= LOCK_HOLD:
                                hist = torso_hist(frame, box)
                                if hist is None:      # box unusable this frame, try again next frame
                                    continue
                                owner_id, owner_hist = pid, hist
                                state, last_seen = change_state(state, "FOLLOW", f"lock id {pid}"), now
                                fs, outran = FollowState(), False
                                lock_start.clear()
                                log.info(f"[LOCK] owner = id {pid}  -> FOLLOW")
                                break
                    for pid in list(lock_start):
                        if pid not in raised:
                            del lock_start[pid]
                            log.info(f"[GESTURE] id {pid}: hand down, lock cancelled")
                    if state == "IDLE":
                        if lock_start:
                            pid, t0 = next(iter(lock_start.items()))
                            pct = min(100, int(100 * (now - t0) / LOCK_HOLD))
                            hint = f"Locking id {pid}... {pct}%"
                        else:
                            hint = "Raise ONE hand above your head to lock"
                else:
                    hint = f"Cooldown {cooldown_until - now:.1f}s"
            elif state == "SEARCH":
                # Only in-place rotation here: the owner is not visible, so never drive forward or back.
                owner = next((p for p in people if p[0] == owner_id), None)
                hold = False
                if owner is not None:
                    log.info(f"[FOUND] owner id {owner_id} after {search.progress()} deg -> FOLLOW")
                    state = change_state(state, "FOLLOW",
                                         f"owner id {owner_id} back after {search.progress()} deg")
                    last_seen = now
                else:
                    best, reid_best = best_match(frame, people, owner_hist)
                    if best is not None and reid_best >= REID_THRESH:
                        n = candidate[1] + 1 if candidate[0] == best[0] else 1
                        candidate, hold = (best[0], n), True    # stand still while confirming
                        if n >= SEARCH_CONFIRM:
                            log.info(f"[RE-LOCK] owner id {owner_id} -> id {best[0]} during search "
                                     f"(clothing match {reid_best:.2f} for {n} frames)")
                            state = change_state(state, "FOLLOW", f"clothing match id {best[0]}")
                            owner_id, last_seen = best[0], now
                    else:
                        candidate = (None, 0)
                if state == "SEARCH":
                    turn, done = search.step(now, hold)
                    left, right = mix(0, turn)
                    hint = f"Searching... {search.progress()}/360 deg"
                    if reid_best is not None:
                        hint += f"  (best match {reid_best:.2f}, need {REID_THRESH})"
                    if done:
                        log.warning(f"[SEARCH] 360 deg scanned, owner not found -> IDLE "
                                    f"(cooldown {COOLDOWN:.1f}s)")
                        state = change_state(state, "IDLE", "360 deg scanned, not found")
                        owner_id, owner_hist, outran = None, None, False
                        cooldown_until, not_found_until = now + COOLDOWN, now + NOT_FOUND_SHOW
                        left = right = 0
                if state != "SEARCH":
                    search, candidate, reid_best = None, (None, 0), None
            else:
                owner = next((p for p in people if p[0] == owner_id), None)

                # Owner's track ID disappeared: try to recognise them by clothing colour.
                if owner is None and owner_hist is not None:
                    best, reid_best = best_match(frame, people, owner_hist)
                    if best is not None and reid_best >= REID_THRESH:
                        log.info(f"[RE-LOCK] owner id {owner_id} -> id {best[0]} "
                                 f"(clothing match {reid_best:.2f} >= {REID_THRESH})")
                        owner, owner_id = best, best[0]
                        fs = FollowState()

                if owner is None:
                    fs.fwd, fs.backing = 0.0, False
                    gone = now - last_seen > LOST_STOP
                    if gone and not args.no_search and (state == "FOLLOW" or outran):
                        direction = 1 if last_err >= 0 else -1
                        reason = "outran owner left view" if outran else f"owner not seen for {LOST_STOP}s"
                        state = change_state(state, "SEARCH", reason)
                        search, candidate = Search(direction, now), (None, 0)
                        fs = FollowState()
                        log.warning(f"[SEARCH] owner id {owner_id} not seen for {LOST_STOP}s -> scanning, "
                                    f"turning {'right' if direction > 0 else 'left'} first")
                    elif gone and state == "FOLLOW":
                        state = change_state(state, "LOST", "owner lost, --no-search")  # motors stay at 0
                        fs = FollowState()
                        log.warning(f"[LOST] owner id {owner_id} not seen for {LOST_STOP}s "
                                    f"-> motors stopped")
                    hint = f"Owner not seen for {now - last_seen:.1f}s"
                    if reid_best is not None:
                        hint += f"  (best match {reid_best:.2f}, need {REID_THRESH})"
                else:
                    pid, box, k = owner
                    last_err = centre_error(box, W)
                    raw = torso_ratio(k, H)
                    if raw is not None:
                        fs.size_s = raw if fs.size_s is None else (1 - SIZE_SMOOTH) * fs.size_s + SIZE_SMOOTH * raw
                        fs.dist_hist.append((now, est_distance(fs.size_s)))
                        while now - fs.dist_hist[0][0] > VEL_WINDOW:
                            fs.dist_hist.popleft()
                    else:
                        fs.dist_hist.clear()     # a window with a hole would give a fake speed
                    size = fs.size_s if raw is not None else None   # torso hidden this frame -> steer only
                    gap = est_distance(size)

                    if outran and gap is not None and gap < OUTRUN_DIST:
                        outran = False
                        log.info(f"[FOUND] owner back within {OUTRUN_DIST} m after outrun -> FOLLOW")
                    elif state == "LOST" and not outran:
                        log.info(f"[FOUND] owner id {pid} back after {now - last_seen:.1f}s -> FOLLOW")
                    last_seen, reid_best = now, None
                    state = change_state(state, "LOST" if outran else "FOLLOW", "owner visible")
                    h = torso_hist(frame, box)
                    if h is not None:            # adapt slowly to lighting changes
                        owner_hist = cv2.addWeighted(owner_hist, 0.95, h, 0.05, 0)

                    if hands_up(k) == 2:         # unlock gesture: pause while held
                        fs.driving, fs.fwd, fs.backing = False, 0.0, False
                        if unlock_start is None:
                            unlock_start = now
                            log.info(f"[GESTURE] both hands up, hold {UNLOCK_HOLD:.1f}s to unlock "
                                     f"(robot paused)")
                        pct = min(100, int(100 * (now - unlock_start) / UNLOCK_HOLD))
                        hint = f"Unlocking... {pct}%  (keep both hands up)"
                        if now - unlock_start >= UNLOCK_HOLD:
                            log.info(f"[UNLOCK] owner id {owner_id} released -> IDLE "
                                     f"(cooldown {COOLDOWN:.1f}s)")
                            state = change_state(state, "IDLE", f"unlock by id {owner_id}")
                            owner_id, owner_hist = None, None
                            unlock_start, cooldown_until = None, now + COOLDOWN
                            fs, outran = FollowState(), False
                    else:
                        if unlock_start is not None:
                            log.info("[GESTURE] hands down, unlock cancelled")
                        unlock_start = None
                        if outran:
                            hint = f"Too fast for me - come back within {OUTRUN_DIST} m"
                        else:
                            fs.v_owner = owner_speed(gap_rate(fs.dist_hist), fs.fwd)
                            fwd_target, turn, fs.driving = follow_command(box, W, size, fs.driving, fs.v_owner)
                            centred = abs(centre_error(box, W)) < BACKOFF_CENTER
                            if us is not None and centred and us < TARGET_DIST * 100:
                                fwd_target, fs.driving = 0.0, False      # sonar stop
                            was_backing = fs.backing
                            if update_backoff(fs, us, centred, gap, now):
                                fs.fwd = -BACKOFF_SPEED              # not ramped; owner_speed() sees the robot moving back
                                if not was_backing:
                                    log.info(f"[BACKOFF] owner at {us} cm -> reversing")
                            else:
                                fs.fwd = ramp_fwd(fs.fwd, fwd_target, dt)
                                if was_backing and fs.backoff_wait:
                                    log.warning("[BACKOFF] time limit, waiting for owner to step back")
                                elif was_backing:
                                    log.info(f"[BACKOFF] done at {us if us is not None else '-'} cm")
                                elif (us is not None and us < MIN_DIST_CM and centred and gap is not None
                                      and gap >= TARGET_DIST and now - last_sonar_warn >= LOG_EVERY):
                                    last_sonar_warn = now
                                    log.warning(f"[SONAR] obstacle at {us} cm but camera says owner "
                                                f"~{gap:.1f} m -> stop, no reverse")
                            left, right = mix(fs.fwd, turn)
                            if fs.backing:
                                hint = f"Too close ({us} cm) - backing off"
                            elif size is not None:
                                hint = (f"Torso {size:.2f} / {TARGET_TORSO:.2f}  ~{gap:.1f} m"
                                        f"   both hands up = unlock")
                                if fs.v_owner is not None:
                                    hint += f"  owner {fs.v_owner:+.1f} m/s"
                            else:
                                hint = "Torso not visible - steering only"

                            if fs.fwd >= 0.9 * MAX_SPEED and gap is not None and gap > OUTRUN_DIST:
                                if fs.outrun_since is None:
                                    fs.outrun_since = now
                                if now - fs.outrun_since >= OUTRUN_TIME:
                                    state = change_state(state, on_outrun(gap), f"outrun, gap ~{gap:.1f} m")
                                    outran = True
                                    fs = FollowState()
                                    left = right = 0
                            else:
                                fs.outrun_since = None

            motors.send(left, right)
            if now - last_log >= LOG_EVERY:
                last_log = now
                ids = [p[0] for p in people]
                torso = f"{fs.size_s:.3f}" if fs.size_s is not None else "-"
                dist = f"{est_distance(fs.size_s):.2f}" if fs.size_s is not None else "-"
                ov = f"{fs.v_owner:+.2f}" if fs.v_owner is not None else "-"
                log.info(f"[STATUS] {state:<6} owner={owner_id if owner_id is not None else '-'} "
                         f"people={len(ids)} {ids} L={left:+4d} R={right:+4d} "
                         f"torso={torso}/{TARGET_TORSO:.2f} dist~{dist}m drive={fs.driving} "
                         f"ov={ov} fwd={fs.fwd:.0f} us={us if us is not None else '-'} "
                         f"search={f'{search.progress()}/360' if search else '-'} fps={fps:.1f}")

            if not args.no_show:
                banner = None
                if state == "LOST":
                    banner = ("OWNER LOST - STOPPED", (0, 0, 200))
                elif state == "SEARCH":
                    banner = ("SEARCHING FOR OWNER", (0, 120, 220))
                elif state == "IDLE" and now < not_found_until:
                    banner = ("OWNER NOT FOUND", (0, 0, 200))
                draw_overlay(frame, people, owner_id, state, left, right, fps, hint, banner)
                cv2.imshow("Owner Follower", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    log.info("[QUIT] q pressed")
                    break
    except KeyboardInterrupt:
        log.info("[QUIT] Ctrl+C")
    finally:
        motors.close()
        cap.release()
        cv2.destroyAllWindows()
        log.info(f"[STOP] motors stopped, {frames} frames processed")


if __name__ == "__main__":
    main()
