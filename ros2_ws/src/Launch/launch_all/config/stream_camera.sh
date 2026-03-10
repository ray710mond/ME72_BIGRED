#!/bin/bash
# Launched by launch_all to stream the Pi camera over UDP.
# Usage: stream_camera.sh HOST_IP PORT WIDTH HEIGHT FPS [--no-save]

HOST_IP=$1
PORT=$2
WIDTH=$3
HEIGHT=$4
FPS=$5
NO_SAVE=$6

RPICAM_CMD="rpicam-vid -n -t 0 --width $WIDTH --height $HEIGHT --framerate $FPS --codec mjpeg --buffer-count 1"

if [ "$NO_SAVE" = "--no-save" ]; then
    echo "[stream_camera] Streaming to $HOST_IP:$PORT (no recording)"
    exec $RPICAM_CMD -o "udp://${HOST_IP}:${PORT}"
else
    REC_DIR="$HOME/recordings"
    mkdir -p "$REC_DIR"
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    REC_FILE="${REC_DIR}/${TIMESTAMP}.mjpeg"
    echo "[stream_camera] Streaming to $HOST_IP:$PORT + saving to $REC_FILE"

    exec $RPICAM_CMD -o - 2>/dev/null \
        | tee "$REC_FILE" \
        | python3 -c "
import sys, socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    d = sys.stdin.buffer.read(32768)
    if not d: break
    s.sendto(d, ('${HOST_IP}', ${PORT}))
"
fi
