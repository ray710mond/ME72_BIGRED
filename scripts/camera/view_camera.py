#!/usr/bin/env python3
import cv2

def main():
    # Open the Pi camera using V4L2 interface
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # 0 = /dev/video0

    if not cap.isOpened():
        print("Cannot open camera. Make sure the Pi camera is enabled and /dev/video0 exists.")
        return

    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Show the frame in a window
        cv2.imshow("Live Camera Feed", frame)

        # Quit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()