"""
Camera input for the owner-following robot. Both camera types give the same interface:
read() -> (ok, BGR frame) and release().
  --source picam   Raspberry Pi Camera Module (ribbon cable) through picamera2/libcamera
  --source 0       USB webcam / laptop webcam (OpenCV), or a video file path
"""
import cv2

PICAM_SIZE = (640, 480)  # px, Pi camera frame size (YOLO resizes to --imgsz anyway)


class PiCamera:
    def __init__(self):
        from picamera2 import Picamera2   # apt package python3-picamera2, only on the Pi
        self.cam = Picamera2()
        # libcamera's "RGB888" is stored as B,G,R bytes, which is exactly what OpenCV expects
        self.cam.configure(self.cam.create_video_configuration(
            main={"size": PICAM_SIZE, "format": "RGB888"}))
        self.cam.start()

    def read(self):
        return True, self.cam.capture_array()

    def release(self):
        self.cam.stop()
        self.cam.close()


def open_camera(source):
    """Returns (camera, description), or (None, error message)."""
    if source == "picam":
        try:
            return PiCamera(), f"Pi camera {PICAM_SIZE[0]}x{PICAM_SIZE[1]}"
        except Exception as e:
            return None, (f"could not open the Pi camera ({e}). Check the ribbon cable, run "
                          "'rpicam-hello --list-cameras', and create the venv with --system-site-packages.")
    cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
    if not cap.isOpened():
        return None, ("could not open the camera. On a Mac: System Settings > Privacy & Security > "
                      "Camera > allow Terminal / VS Code, then restart it. Or try --source 1, "
                      "or --source picam for the Pi Camera Module.")
    return cap, (f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                 f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
