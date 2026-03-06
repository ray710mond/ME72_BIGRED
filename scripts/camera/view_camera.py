#!/usr/bin/env python3

import cv2
import subprocess
import numpy as np

cmd = [
    "rpicam-vid",
    "--codec", "mjpeg",
    "--width", "640",
    "--height", "480",
    "--framerate", "30",
    "--timeout", "0",
    "--inline",
    "--stdout"
]

pipe = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)

buffer = b''

print("Press 'q' to quit")

while True:

    buffer += pipe.stdout.read(4096)

    a = buffer.find(b'\xff\xd8')  # JPEG start
    b = buffer.find(b'\xff\xd9')  # JPEG end

    if a != -1 and b != -1 and b > a:
        jpg = buffer[a:b+2]
        buffer = buffer[b+2:]

        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

        if frame is not None:

            # rotate 90° CCW
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

            cv2.imshow("Pi Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

pipe.terminate()
cv2.destroyAllWindows()
