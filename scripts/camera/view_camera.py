#!/usr/bin/env python3

from picamera2 import Picamera2
import cv2

def main():

    picam2 = Picamera2()

    config = picam2.create_preview_configuration()
    picam2.configure(config)

    picam2.start()

    print("Press 'q' to quit.")

    while True:
        frame = picam2.capture_array()

        cv2.imshow("Live Camera Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    picam2.stop()

if __name__ == "__main__":
    main()
