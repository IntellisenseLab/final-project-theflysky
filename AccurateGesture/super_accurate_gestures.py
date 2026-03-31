"""
Super Accurate Hand Gesture Recognition (v2)
============================================
Accurate Hand Gesture Recognition on Raspberry Pi 5.

This script replaces the old finger-counting landmarker logic with the
official MediaPipe Gesture Recognizer (CNN-based classification).

Key advantages:
- Deep learning-based gesture classification (no heuristics/angles).
- Recognises complex gestures: Thumb Up, Closed Fist, Open Palm, Victory, etc.
- Optimized for edge CPU (no Torch/Pytorch needed).
- Stable tracking (less landmarker jitter).

Recognised Gestures
-------------------
  • FIST
  • OPEN PALM
  • POINTING (Up)  + Direction detection (N, NE, E, etc.)
  • THUMBS UP
  • PEACE (Victory)
  • I LOVE YOU
  • THUMB DOWN

Run:
  .venv/bin/python "AccurateGesture/super_accurate_gestures.py"
"""

import collections
import math
import os
import sys
import time

import cv2
import mediapipe as mp

# ──────────────────────────────────────────────────────────────────────────────
# Setup
# ──────────────────────────────────────────────────────────────────────────────

# Path to the pre-packaged task file (CNN model for gestures)
MODEL_PATH = "/home/nadeesha/final-project-flysky/models/gesture_recognizer.task"

# MediaPipe Task aliases
BaseOptions       = mp.tasks.BaseOptions
GestureRecognizer = mp.tasks.vision.GestureRecognizer
Options           = mp.tasks.vision.GestureRecognizerOptions
RunningMode       = mp.tasks.vision.RunningMode

# ──────────────────────────────────────────────────────────────────────────────
# Gesture Mapping (Model Labels -> Display Labels)
# ──────────────────────────────────────────────────────────────────────────────
# The gesture_recognizer.task model outputs labels like:
# 'None', 'Closed_Fist', 'Open_Palm', 'Pointing_Up', 'Thumb_Down', 'Thumb_Up', 'Victory', 'ILoveYou'
GESTURE_MAP = {
    "Closed_Fist": "FIST",
    "Open_Palm":   "OPEN PALM",
    "Pointing_Up": "POINTING",
    "Thumb_Up":    "THUMBS UP",
    "Thumb_Down":  "THUMBS DOWN",
    "Victory":     "PEACE",
    "ILoveYou":    "I LOVE YOU",
}

# ──────────────────────────────────────────────────────────────────────────────
# Direction Logic (when POINTING)
# ──────────────────────────────────────────────────────────────────────────────

def get_pointing_direction(landmarks):
    """
    Estimate direction of the index finger when POINTING.
    landmarks: list of 21 NormalizedLandmark objects.
    """
    wrist = landmarks[0]
    index_mcp = landmarks[5]
    index_tip = landmarks[8]

    # Vector from index MCP to TIP
    dx = index_tip.x - index_mcp.x
    dy = index_tip.y - index_mcp.y
    dz = index_mcp.z - index_tip.z  # Positive if tip is closer to camera

    xy_len = math.hypot(dx, dy)

    # If tip is much closer to camera but doesn't move much in XY plane, it's pointing AT the camera.
    if dz > 0.12 and xy_len < 0.10:
        return "TOWARDS CAMERA"

    # Angle in degrees (0-360) – standard atan2 approach
    angle = math.degrees(math.atan2(-dy, dx))
    if angle < 0: angle += 360

    directions = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"]
    idx = int((angle + 22.5) // 45) % 8
    return directions[idx]

# ──────────────────────────────────────────────────────────────────────────────
# Main Application
# ──────────────────────────────────────────────────────────────────────────────

def main():
    if not os.path.exists(MODEL_PATH):
        print(f"[ERROR] Model file not found at {MODEL_PATH}")
        sys.exit(1)

    print("[INFO] Starting Super Accurate Gesture Recognizer...")
    print("[INFO] Running on Raspberry Pi 5 (Cortex-A76 Optimized).")

    # 1. Start camera capture
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("[ERROR] Cannot open webcam.")
        return

    # Reduce resolution for better FPS on RPi if needed (defaulting to 640x480)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 2. Configure Gesture Recognizer (CNN task)
    options = Options(
        base_options=BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.6,
        min_hand_presence_confidence=0.6,
        min_tracking_confidence=0.6
    )

    # 3. Use Context Manager for automatic cleanup
    with GestureRecognizer.create_from_options(options) as recognizer:
        
        # Stability: gesture history for smoothing
        gesture_history = collections.deque(maxlen=5)
        last_time = time.time()
        fps = 0

        while True:
            ret, frame = cap.read()
            if not ret: break

            # Prepare image for MediaPipe
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            # Detect gestures for current frame (Video mode requires timestamp)
            timestamp_ms = int(time.time() * 1000)
            result = recognizer.recognize_for_video(mp_image, timestamp_ms)

            current_gesture = "No Hand"
            confidence = 0.0
            pointing_dir = "-"
            hand_type = "-"

            # Process detection results
            if result.gestures and len(result.gestures) > 0:
                best_gesture = result.gestures[0][0]
                label = best_gesture.category_name
                confidence = best_gesture.score

                # Map to our display name
                current_gesture = GESTURE_MAP.get(label, "CUSTOM")
                
                # Handedness (Left/Right)
                if result.handedness:
                    hand_type = result.handedness[0][0].category_name

                # Direction Detection (Special case for Pointing)
                if current_gesture == "POINTING" and result.hand_landmarks:
                    landmarks = result.hand_landmarks[0]
                    pointing_dir = get_pointing_direction(landmarks)

                # Stabilize: Smoothed label choice
                gesture_history.append(current_gesture)
                stable_gesture = max(set(gesture_history), key=gesture_history.count)
            else:
                gesture_history.clear()
                stable_gesture = "No Hand"

            # ───────────────────────────────────────────────────────────────────
            # Drawing / HUD
            # ───────────────────────────────────────────────────────────────────
            
            # FPS Calculation
            curr_time = time.time()
            dt = curr_time - last_time
            last_time = curr_time
            if dt > 0: fps = 0.9 * fps + 0.1 * (1.0 / dt)

            # HUD Background
            cv2.rectangle(frame, (0, 0), (280, 180), (0, 0, 0), -1)
            cv2.addWeighted(frame[:180, :280], 0.3, frame[:180, :280], 0, 0, frame[:180, :280])

            # Draw Output Info
            text_color = (0, 255, 0) if stable_gesture != "No Hand" else (200, 200, 200)
            cv2.putText(frame, f"Gesture: {stable_gesture}", (15, 35), cv2.FONT_HERSHEY_DUPLEX, 0.8, text_color, 2)
            cv2.putText(frame, f"Conf: {confidence:.2f}", (15, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(frame, f"Hand: {hand_type}", (15, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(frame, f"P. Dir: {pointing_dir}", (15, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            cv2.putText(frame, f"FPS: {fps:.1f}", (15, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

            # Optional: Draw Landmarks if needed
            if result.hand_landmarks:
                h, w, _ = frame.shape
                for lm in result.hand_landmarks[0]:
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)

            # ───────────────────────────────────────────────────────────────────
            # Display Window
            # ───────────────────────────────────────────────────────────────────
            cv2.imshow("Super Accurate Gestures (CNN model)", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
