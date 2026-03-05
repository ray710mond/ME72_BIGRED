#!/usr/bin/env python3
import cv2
import time
import os

# Directory to save images
save_dir = "/home/bigred/captured_images"  # update path if needed
os.makedirs(save_dir, exist_ok=True)

# Open camera
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not cap.isOpened():
    print("Error: Cannot open camera")
    exit()

# Force MJPEG, 640x480, 30 FPS
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

try:
    for i in range(5):  # capture 5 images
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            continue  # try next frame

        # Optional: show the frame in a small window (can comment out)
        cv2.imshow("Preview", frame)
        cv2.waitKey(1)  # required to render window

        # Save frame as image
        file_path = os.path.join(save_dir, f"image_{i+1:03d}.jpg")
        cv2.imwrite(file_path, frame)
        print(f"Saved {file_path}")

        time.sleep(1)  # wait 1 second between captures

finally:
    cap.release()
    cv2.destroyAllWindows()
    print("Camera released")
    