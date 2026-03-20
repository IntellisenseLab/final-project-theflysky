import cv2
import time
import math
import collections
import mediapipe as mp

# ===== MediaPipe Tasks imports =====
BaseOptions = mp.tasks.BaseOptions
VisionRunningMode = mp.tasks.vision.RunningMode

HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions

HAND_MODEL_PATH = "/home/nadeesha/final-project-flysky/models/hand_landmarker.task"

TIP_IDS = [4, 8, 12, 16, 20]
PIP_IDS = [3, 6, 10, 14, 18]

# Thumb extension tuning (for flicker control)
THUMB_STRAIGHT_ANGLE_DEG = 150.0
THUMB_WRIST_EXTENSION_MARGIN = 0.04


def fingers_up(landmarks, handedness_name):
    """
    landmarks: list of 21 NormalizedLandmark objects
    handedness_name: 'Left' or 'Right'
    returns [thumb, index, middle, ring, pinky]
    """
    fingers = []

    # Thumb state from hand geometry (orientation-independent):
    # 1) thumb is relatively straight at IP joint
    # 2) tip is farther from wrist than IP joint
    thumb_mcp = landmarks[2]
    thumb_ip = landmarks[3]
    thumb_tip = landmarks[4]
    wrist = landmarks[0]

    v1x = thumb_mcp.x - thumb_ip.x
    v1y = thumb_mcp.y - thumb_ip.y
    v1z = thumb_mcp.z - thumb_ip.z
    v2x = thumb_tip.x - thumb_ip.x
    v2y = thumb_tip.y - thumb_ip.y
    v2z = thumb_tip.z - thumb_ip.z

    n1 = math.sqrt(v1x * v1x + v1y * v1y + v1z * v1z)
    n2 = math.sqrt(v2x * v2x + v2y * v2y + v2z * v2z)

    thumb_straight = False
    if n1 > 1e-6 and n2 > 1e-6:
        cosang = (v1x * v2x + v1y * v2y + v1z * v2z) / (n1 * n2)
        cosang = max(-1.0, min(1.0, cosang))
        angle_deg = math.degrees(math.acos(cosang))
        thumb_straight = angle_deg > THUMB_STRAIGHT_ANGLE_DEG

    d_tip_wrist = math.sqrt(
        (thumb_tip.x - wrist.x) ** 2 +
        (thumb_tip.y - wrist.y) ** 2 +
        (thumb_tip.z - wrist.z) ** 2
    )
    d_ip_wrist = math.sqrt(
        (thumb_ip.x - wrist.x) ** 2 +
        (thumb_ip.y - wrist.y) ** 2 +
        (thumb_ip.z - wrist.z) ** 2
    )
    thumb_extended = d_tip_wrist > (d_ip_wrist + THUMB_WRIST_EXTENSION_MARGIN)

    fingers.append(1 if (thumb_straight and thumb_extended) else 0)

    for i in range(1, 5):
        tip_y = landmarks[TIP_IDS[i]].y
        pip_y = landmarks[PIP_IDS[i]].y
        fingers.append(1 if tip_y < pip_y else 0)

    return fingers


def draw_landmarks_and_connections(frame, landmarks):
    h, w, _ = frame.shape

    connections = [
        (0, 1), (1, 2), (2, 3), (3, 4),
        (0, 5), (5, 6), (6, 7), (7, 8),
        (5, 9), (9, 10), (10, 11), (11, 12),
        (9, 13), (13, 14), (14, 15), (15, 16),
        (13, 17), (17, 18), (18, 19), (19, 20),
        (0, 17)
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
    h, w, _ = frame.shape
    overlay = frame.copy()

    finger_chains = [
        [0, 1, 2, 3, 4],
        [0, 5, 6, 7, 8],
        [0, 9, 10, 11, 12],
        [0, 13, 14, 15, 16],
        [0, 17, 18, 19, 20]
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


def classify_gesture_from_fingers(fingers):
    """
    Heuristic gesture labels from [thumb, index, middle, ring, pinky].
    This keeps gesture labeling fully based on hand_landmarker landmarks.
    """
    patterns = {
        (0, 0, 0, 0, 0): "FIST",
        (1, 1, 1, 1, 1): "OPEN PALM",
        (0, 1, 0, 0, 0): "POINTING",
        (1, 0, 0, 0, 0): "THUMBS UP",
        (0, 1, 1, 0, 0): "PEACE",
        (1, 1, 0, 0, 1): "I LOVE YOU",
    }
    return patterns.get(tuple(fingers), "CUSTOM")


def get_index_direction(landmarks):
    """
    Estimate index-finger pointing direction using MCP->TIP vector.
    Returns one of: N, NE, E, SE, S, SW, W, NW, TOWARDS CAMERA
    """
    index_mcp = landmarks[5]
    index_tip = landmarks[8]

    dx = index_tip.x - index_mcp.x
    dy = index_tip.y - index_mcp.y
    dz = index_mcp.z - index_tip.z  # Positive if tip is closer to camera.

    xy_len = math.hypot(dx, dy)

    # If finger projection is short in image plane but depth difference is strong,
    # treat as pointing toward the camera.
    if dz > 0.12 and xy_len < 0.10:
        return "TOWARDS CAMERA"

    angle = math.degrees(math.atan2(-dy, dx))
    if angle < 0:
        angle += 360

    directions = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"]
    idx = int((angle + 22.5) // 45) % 8
    return directions[idx]


def main():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    if not cap.isOpened():
        print("Cannot open webcam")
        return

    gesture_history = collections.deque(maxlen=7)

    hand_options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=HAND_MODEL_PATH),
        running_mode=VisionRunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.7,
        min_hand_presence_confidence=0.7,
        min_tracking_confidence=0.7
    )

    with HandLandmarker.create_from_options(hand_options) as hand_landmarker:

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read webcam")
                break

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=rgb_frame
            )

            timestamp_ms = int(time.time() * 1000)

            hand_result = hand_landmarker.detect_for_video(mp_image, timestamp_ms)

            gesture_text = "No Hand"
            handedness_name = "Unknown"
            index_direction_text = "-"

            if hand_result.hand_landmarks and len(hand_result.hand_landmarks) > 0:
                landmarks = hand_result.hand_landmarks[0]

                if hand_result.handedness and len(hand_result.handedness) > 0 and len(hand_result.handedness[0]) > 0:
                    handedness_name = hand_result.handedness[0][0].category_name

                fingers = fingers_up(landmarks, handedness_name)

                # Direction detection is shown only when index finger is the sole finger up.
                if fingers == [0, 1, 0, 0, 0]:
                    index_direction_text = get_index_direction(landmarks)

                gesture_history.append(classify_gesture_from_fingers(fingers))
                gesture_text = max(set(gesture_history), key=gesture_history.count)

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
                    f"Index Dir: {index_direction_text}",
                    (10, 105),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2
                )
            else:
                gesture_history.clear()

            cv2.putText(
                frame,
                f"Gesture: {gesture_text}",
                (10, 140),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 0, 0),
                2
            )

            cv2.imshow("Hand Landmarker Only", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()