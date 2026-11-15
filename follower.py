#!/usr/bin/env python3
"""
Owner-following robot (DesiPachinos)
Phase 1 features: owner LOCK, UNLOCK, RE-LOCK after losing the owner, SAFE-DISTANCE FOLLOW.

Gestures (hold them still):
  - Raise ONE hand above your head for ~1 s  -> robot locks onto you
  - Raise BOTH hands above your head for ~1.5 s -> robot unlocks and goes idle

Run on a laptop (no robot):    python follower.py
Run with the Arduino attached: python follower.py --port /dev/tty.usbmodem1101   (Mac)
                               python follower.py --port /dev/ttyACM0            (Pi / Jetson)
Press q in the video window to quit. Logs go to the terminal and to follower.log.
"""
import argparse
import logging
import time

import cv2
import numpy as np
from ultralytics import YOLO

# ---- COCO keypoint indices used by YOLOv8-pose ----
NOSE, L_SHOULDER, R_SHOULDER, L_WRIST, R_WRIST = 0, 5, 6, 9, 10
KP_CONF = 0.5            # ignore keypoints below this confidence

# ---- Tuning (adjust during testing) ----
LOCK_HOLD = 1.0          # s, one hand up to lock
UNLOCK_HOLD = 1.5        # s, both hands up to unlock
COOLDOWN = 2.0           # s after unlock before anyone can lock again
LOST_STOP = 0.3          # s owner missing before the robot stops
REID_THRESH = 0.75       # clothing-colour similarity needed to re-lock (0..1)
TARGET_H = 0.55          # owner box height / frame height at the wanted distance (~1 m); tune!
DEADBAND = 0.05          # no forward motion when this close to the target size
CENTER_DEADBAND = 0.10   # no turning when owner is this close to the frame centre
K_FWD = 500              # forward gain
K_TURN = 160             # turning gain
MAX_SPEED = 200          # PWM limit (0..255)
MIN_SPEED = 70           # PWM below which the motors stall

# ---- Display and logging ----
UI_SCALE = 1.0           # multiplier for all on-screen text; raise it if text is still too small
LOG_EVERY = 1.0          # s between status lines in the terminal
LOG_FILE = "follower.log"  # log file, overwritten on every run

FONT = cv2.FONT_HERSHEY_SIMPLEX
STATE_COLOURS = {"IDLE": (230, 230, 230), "FOLLOW": (0, 230, 0), "LOST": (0, 0, 255)}  # BGR

log = logging.getLogger("follower")


def setup_logging():
    fmt = logging.Formatter("%(asctime)s.%(msecs)03d  %(levelname)-7s %(message)s", "%H:%M:%S")
    log.setLevel(logging.INFO)
    for handler in (logging.StreamHandler(), logging.FileHandler(LOG_FILE, mode="w")):
        handler.setFormatter(fmt)
        log.addHandler(handler)


def keypoint(k, i):
    x, y, c = k[i]
    return (x, y) if c > KP_CONF else None


def hands_up(k):
    """Number of wrists raised above the head (0, 1 or 2)."""
    nose = keypoint(k, NOSE)
    if nose is not None:
        ref = nose[1]
    else:
        shoulders = [p for p in (keypoint(k, L_SHOULDER), keypoint(k, R_SHOULDER)) if p]
        if not shoulders:
            return 0
        ref = min(p[1] for p in shoulders) - 40  # a bit above the shoulders
    return sum(1 for w in (keypoint(k, L_WRIST), keypoint(k, R_WRIST)) if w and w[1] < ref)


def torso_hist(frame, box):
    """Hue/saturation histogram of the upper body, used to recognise the owner's clothes."""
    H, W = frame.shape[:2]
    x1, y1, x2, y2 = box
    h, w = y2 - y1, x2 - x1
    xa, xb = int(max(0, x1 + 0.2 * w)), int(min(W, x2 - 0.2 * w))
    ya, yb = int(max(0, y1 + 0.2 * h)), int(min(H, y1 + 0.6 * h))
    crop = frame[ya:yb, xa:xb]
    if crop.size == 0:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
    cv2.normalize(hist, hist)
    return hist


def shape_speed(v):
    v = max(-MAX_SPEED, min(MAX_SPEED, v))
    if abs(v) < 20:
        return 0
    return int(np.sign(v) * max(abs(v), MIN_SPEED))


def follow_command(box, W, H):
    """Proportional control: steer toward the owner, drive forward until the box is TARGET_H tall."""
    x1, y1, x2, y2 = box
    err_x = ((x1 + x2) / 2 - W / 2) / (W / 2)      # -1 (left) .. +1 (right)
    height_ratio = (y2 - y1) / H                    # bigger = closer
    fwd = 0.0 if height_ratio > TARGET_H - DEADBAND else K_FWD * (TARGET_H - height_ratio)
    turn = K_TURN * err_x if abs(err_x) > CENTER_DEADBAND else 0.0
    return shape_speed(fwd + turn), shape_speed(fwd - turn)


class Motors:
    """Sends 'M,<left>,<right>\\n' to the Arduino. Without a port it only prints/overlays."""

    def __init__(self, port):
        self.ser = None
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


def draw_overlay(frame, people, owner_id, state, left, right, fps, hint):
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

    # Big warning when the owner is lost
    if state == "LOST":
        msg = "OWNER LOST - STOPPED"
        w, h, _, _ = text_size(msg, 1.6 * s, t + 2)
        draw_text(frame, msg, max(0, (W - w) // 2), H - h - int(20 * s), 1.6 * s, (255, 255, 255),
                  bg=(0, 0, 200), thick=t + 2)


# ---------------------------------------------------------------- main loop

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0", help="camera index or video file")
    ap.add_argument("--port", default=None, help="Arduino serial port (omit for laptop testing)")
    ap.add_argument("--model", default="yolov8n-pose.pt")
    ap.add_argument("--imgsz", type=int, default=320, help="320 is fast; 640 is more accurate")
    ap.add_argument("--no-show", action="store_true", help="no video window (headless robot)")
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

    state = "IDLE"            # IDLE -> FOLLOW <-> LOST -> (unlock) IDLE
    owner_id, owner_hist = None, None
    lock_start = {}           # track id -> time one hand went up
    unlock_start = None
    last_seen = 0.0
    cooldown_until = 0.0
    seen_ids = set()
    fps, prev_t, last_log = 0.0, time.time(), 0.0
    owner_ratio, reid_best = None, None
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
                                state, last_seen = "FOLLOW", now
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
            else:
                owner = next((p for p in people if p[0] == owner_id), None)

                # Owner's track ID disappeared: try to recognise them by clothing colour.
                if owner is None and owner_hist is not None:
                    best, best_score = None, 0.0
                    for p in people:
                        h = torso_hist(frame, p[1])
                        if h is not None:
                            s = cv2.compareHist(owner_hist, h, cv2.HISTCMP_CORREL)
                            if s > best_score:
                                best, best_score = p, s
                    reid_best = best_score if best is not None else None
                    if best is not None and best_score >= REID_THRESH:
                        log.info(f"[RE-LOCK] owner id {owner_id} -> id {best[0]} "
                                 f"(clothing match {best_score:.2f} >= {REID_THRESH})")
                        owner, owner_id = best, best[0]

                if owner is None:
                    owner_ratio = None
                    if state == "FOLLOW" and now - last_seen > LOST_STOP:
                        state = "LOST"           # motors stay at 0: never follow a stranger
                        log.warning(f"[LOST] owner id {owner_id} not seen for {LOST_STOP}s "
                                    f"-> motors stopped")
                    hint = f"Owner not seen for {now - last_seen:.1f}s"
                    if reid_best is not None:
                        hint += f"  (best match {reid_best:.2f}, need {REID_THRESH})"
                else:
                    pid, box, k = owner
                    if state == "LOST":
                        log.info(f"[FOUND] owner id {pid} back after {now - last_seen:.1f}s -> FOLLOW")
                    last_seen, state, reid_best = now, "FOLLOW", None
                    owner_ratio = (box[3] - box[1]) / H
                    h = torso_hist(frame, box)
                    if h is not None:            # adapt slowly to lighting changes
                        owner_hist = cv2.addWeighted(owner_hist, 0.95, h, 0.05, 0)

                    if hands_up(k) == 2:         # unlock gesture: pause while held
                        if unlock_start is None:
                            unlock_start = now
                            log.info(f"[GESTURE] both hands up, hold {UNLOCK_HOLD:.1f}s to unlock "
                                     f"(robot paused)")
                        pct = min(100, int(100 * (now - unlock_start) / UNLOCK_HOLD))
                        hint = f"Unlocking... {pct}%  (keep both hands up)"
                        if now - unlock_start >= UNLOCK_HOLD:
                            log.info(f"[UNLOCK] owner id {owner_id} released -> IDLE "
                                     f"(cooldown {COOLDOWN:.1f}s)")
                            state, owner_id, owner_hist = "IDLE", None, None
                            unlock_start, cooldown_until = None, now + COOLDOWN
                            owner_ratio = None
                    else:
                        if unlock_start is not None:
                            log.info("[GESTURE] hands down, unlock cancelled")
                        unlock_start = None
                        left, right = follow_command(box, W, H)
                        hint = f"Size {owner_ratio:.2f} / target {TARGET_H:.2f}   both hands up = unlock"

            motors.send(left, right)

            if now - last_log >= LOG_EVERY:
                last_log = now
                ids = [p[0] for p in people]
                size = f"{owner_ratio:.2f}" if owner_ratio is not None else "-"
                log.info(f"[STATUS] {state:<6} owner={owner_id if owner_id is not None else '-'} "
                         f"people={len(ids)} {ids} L={left:+4d} R={right:+4d} "
                         f"size={size}/{TARGET_H:.2f} fps={fps:.1f}")

            if not args.no_show:
                draw_overlay(frame, people, owner_id, state, left, right, fps, hint)
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
