import cv2
import os
from datetime import datetime
import time

# Folder to save images
SAVE_FOLDER = "captured_images"
os.makedirs(SAVE_FOLDER, exist_ok=True)

# Duration to capture images (seconds)
CAPTURE_DURATION = 5

# Open the Pi Camera (device 0)
cap = cv2.VideoCapture(0)

# Set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit(1)

print(f"Capturing images for {CAPTURE_DURATION} seconds...")

start_time = time.time()
frame_count = 0

while time.time() - start_time < CAPTURE_DURATION:
    ret, frame = cap.read()
    if not ret:
        continue

    # Rotate 90° counterclockwise
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Save frame with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = os.path.join(SAVE_FOLDER, f"image_{timestamp}.png")
    cv2.imwrite(filename, frame)
    frame_count += 1

cap.release()
print(f"Finished! Saved {frame_count} images to '{SAVE_FOLDER}'")
