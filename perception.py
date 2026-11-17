"""
Perception helpers for the owner-following robot: keypoints, hand gestures, torso size and
clothing-colour re-identification. Pure functions on YOLOv8-pose output (no state, no serial).
"""
import cv2
import numpy as np

# ---- COCO keypoint indices used by YOLOv8-pose ----
NOSE, L_SHOULDER, R_SHOULDER, L_WRIST, R_WRIST = 0, 5, 6, 9, 10
L_HIP, R_HIP = 11, 12
KP_CONF = 0.5            # ignore keypoints below this confidence


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


def torso_ratio(k, H):
    """Shoulder-to-hip length / frame height, or None if shoulders or hips are not visible."""
    sh = [p for p in (keypoint(k, L_SHOULDER), keypoint(k, R_SHOULDER)) if p]
    hp = [p for p in (keypoint(k, L_HIP), keypoint(k, R_HIP)) if p]
    if not sh or not hp:
        return None
    length = np.mean([p[1] for p in hp]) - np.mean([p[1] for p in sh])
    return length / H if length > 0 else None


def best_match(frame, people, owner_hist):
    """Person whose clothing looks most like the owner's: (person, correlation), or (None, None)."""
    best, best_score = None, 0.0
    for p in people:
        h = torso_hist(frame, p[1])
        if h is not None:
            s = cv2.compareHist(owner_hist, h, cv2.HISTCMP_CORREL)
            if s > best_score:
                best, best_score = p, s
    return best, (best_score if best is not None else None)
