import cv2
import time
import numpy as np
import mediapipe as mp

# ===== MediaPipe Tasks imports =====
BaseOptions = mp.tasks.BaseOptions
VisionRunningMode = mp.tasks.vision.RunningMode
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions

MODEL_PATH = "/home/nadeesha/final-project-flysky/models/hand_landmarker.task"

TIP_IDS = [4, 8, 12, 16, 20]
PIP_IDS = [3, 6, 10, 14, 18]


def fingers_up(landmarks, handedness_name):
    """
    landmarks: list of 21 NormalizedLandmark objects
    handedness_name: 'Left' or 'Right'
    returns [thumb, index, middle, ring, pinky]
    """
    fingers = []

    # Thumb logic depends on hand side
    thumb_tip_x = landmarks[TIP_IDS[0]].x
    thumb_pip_x = landmarks[PIP_IDS[0]].x

    if handedness_name == "Right":
        fingers.append(1 if thumb_tip_x < thumb_pip_x else 0)
    else:
        fingers.append(1 if thumb_tip_x > thumb_pip_x else 0)

    # Other fingers: tip higher than PIP means finger is up
    for i in range(1, 5):
        tip_y = landmarks[TIP_IDS[i]].y
        pip_y = landmarks[PIP_IDS[i]].y
        fingers.append(1 if tip_y < pip_y else 0)

    return fingers


def detect_gesture(fingers):
    if fingers == [0, 0, 0, 0, 0]:
        return "FIST"
    elif fingers == [1, 1, 1, 1, 1]:
        return "OPEN PALM"
    elif fingers == [0, 1, 0, 0, 0]:
        return "POINTING"
    elif fingers == [1, 0, 0, 0, 0]:
        return "THUMBS UP"
    elif fingers == [0, 1, 1, 0, 0]:
        return "PEACE"
    else:
        return "UNKNOWN"


def draw_landmarks_and_connections(frame, landmarks):
    h, w, _ = frame.shape

    # Standard hand connections
    connections = [
        (0,1), (1,2), (2,3), (3,4),
        (0,5), (5,6), (6,7), (7,8),
        (5,9), (9,10), (10,11), (11,12),
        (9,13), (13,14), (14,15), (15,16),
        (13,17), (17,18), (18,19), (19,20),
        (0,17)
    ]

    for a, b in connections:
        x1 = int(landmarks[a].x * w)
        y1 = int(landmarks[a].y * h)
        x2 = int(landmarks[b].x * w)
        y2 = int(landmarks[b].y * h)
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    for lm in landmarks:
        x = int(lm.x * w)
        y = int(lm.y * h)
        cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)


def draw_finger_mask(frame, landmarks):
    """
    Mask all fingers with thick black lines and circles.
    """
    h, w, _ = frame.shape
    overlay = frame.copy()

    finger_chains = [
        [0, 1, 2, 3, 4],      # thumb
        [0, 5, 6, 7, 8],      # index
        [0, 9, 10, 11, 12],   # middle
        [0, 13, 14, 15, 16],  # ring
        [0, 17, 18, 19, 20]   # pinky
    ]

    for chain in finger_chains:
        pts = []
        for idx in chain:
            x = int(landmarks[idx].x * w)
            y = int(landmarks[idx].y * h)
            pts.append((x, y))

        for i in range(len(pts) - 1):
            cv2.line(overlay, pts[i], pts[i + 1], (0, 0, 0), 30)

        for p in pts:
            cv2.circle(overlay, p, 16, (0, 0, 0), -1)

    return overlay


def main():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    if not cap.isOpened():
        print("Cannot open webcam")
        return

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=VisionRunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.7,
        min_hand_presence_confidence=0.7,
        min_tracking_confidence=0.7
    )

    with HandLandmarker.create_from_options(options) as landmarker:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read webcam")
                break

            frame = cv2.flip(frame, 1)

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=rgb_frame
            )

            timestamp_ms = int(time.time() * 1000)
            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            gesture_text = "No Hand"

            if result.hand_landmarks and len(result.hand_landmarks) > 0:
                landmarks = result.hand_landmarks[0]

                handedness_name = "Unknown"
                if result.handedness and len(result.handedness) > 0 and len(result.handedness[0]) > 0:
                    handedness_name = result.handedness[0][0].category_name

                fingers = fingers_up(landmarks, handedness_name)
                gesture_text = detect_gesture(fingers)

                frame = draw_finger_mask(frame, landmarks)
                draw_landmarks_and_connections(frame, landmarks)

                cv2.putText(
                    frame,
                    f"Hand: {handedness_name}",
                    (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"Fingers: {fingers}",
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

            cv2.putText(
                frame,
                f"Gesture: {gesture_text}",
                (10, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 0, 0),
                2
            )

            cv2.imshow("New MediaPipe Hand Gestures", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()