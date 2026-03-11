#!/usr/bin/env python3
"""
Competition-ready visual servoing script.

Reads camera frames, confirms the hole contour, tracks it, computes motor
commands to align the hole with a saved target position, and streams 
annotated video back to a Mac for live viewing without freezing the Pi.

Usage on Pi (competition day):
  rpicam-vid -n -t 0 --width 1280 --height 720 --framerate 15 \
      --codec mjpeg --buffer-count 1 -o - 2>/dev/null \
    | python3 navigate.py --stdin --stream-to MAC_IP:5001

Usage on Mac (view the Pi stream):
  python3 live_feed.py --port 5001
"""

import json
import argparse
import socket
import sys
import os
import select
import threading
import termios
import tty
import cv2
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hole_detection import (
    process_frame,
    _can_show_windows,
    _bboxes_overlap,
    _BRIGHTNESS_THRESHOLDS,
    _MIN_OVERLAP_IOU,
    _OVERLAP_GRACE_AFTER,
    _MAX_BRIGHTNESS_DIFF,
    _MAX_HOLE_BRIGHTNESS,
    _MIN_AREA_RATIO,
    _MAX_AREA_RATIO,
    _REACQUIRE_CONFIRM,
    _MAX_CONSECUTIVE_FAILURES,
)

# ---------------------------------------------------------------------------
# Controller gains  (tune on the real robot)
# ---------------------------------------------------------------------------
KP_STEER = 1.0
KP_THROTTLE = 0.6
KP_ANGLE = 0.005

DEADBAND_X_PX = 20
DEADBAND_Y_PX = 20
DEADBAND_ANGLE_DEG = 3.0

MAX_STEER = 0.4
MAX_THROTTLE = 0.3

ARRIVED_X_PX = 10
ARRIVED_Y_PX = 10
ARRIVED_ANGLE_DEG = 2.0

LEFT_ROI = 0.10

_DEFAULT_TARGET = Path(__file__).with_name("target.json")

# ---------------------------------------------------------------------------
# MJPEG stdin reader  (frame-dropping for real-time)
# ---------------------------------------------------------------------------
_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"

def mjpeg_stdin_frames(raw_streamer=None):
    """Yield the LATEST frame from an MJPEG stream on stdin."""
    buf = bytearray()
    fd = sys.stdin.buffer.fileno()

    while True:
        try:
            chunk = os.read(fd, 131072)
        except OSError:
            return
        if not chunk:
            return
        buf.extend(chunk)

        while select.select([fd], [], [], 0)[0]:
            try:
                more = os.read(fd, 131072)
            except OSError:
                break
            if not more:
                break
            buf.extend(more)

        last_frame = None
        while True:
            start = buf.find(_SOI)
            if start == -1:
                buf = buf[-1:]
                break
            end = buf.find(_EOI, start + 2)
            if end == -1:
                break
            jpeg = bytes(buf[start : end + 2])
            buf = buf[end + 2 :]

            if raw_streamer is not None:
                raw_streamer.send_raw(jpeg)

            frame = cv2.imdecode(
                np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if frame is not None:
                last_frame = frame

        if last_frame is not None:
            yield last_frame

# ---------------------------------------------------------------------------
# UDP frame streamer  (Pi -> Mac)
# ---------------------------------------------------------------------------
class UDPStreamer:
    """Send JPEG-encoded frames over UDP for live_feed.py."""
    def __init__(self, host: str, port: int):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, frame: np.ndarray, quality=35, scale=0.5):
        if scale != 1.0:
            h, w = frame.shape[:2]
            frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
        _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        self.send_raw(jpeg.tobytes())

    def send_raw(self, data: bytes):
        chunk_size = 32768
        for i in range(0, len(data), chunk_size):
            self.sock.sendto(data[i : i + chunk_size], self.addr)

    def close(self):
        self.sock.close()

# ---------------------------------------------------------------------------
# Threaded Keyboard Listener
# ---------------------------------------------------------------------------
shared_key = None

def keyboard_listener():
    """Runs in the background, waiting for 'y', 'n', or 'q' without blocking video."""
    global shared_key
    try:
        with open("/dev/tty", "r") as tty_file:
            fd = tty_file.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                while True:
                    char = tty_file.read(1)
                    if char:
                        shared_key = char.lower()
                    if shared_key == 'q':
                        break
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    except Exception as e:
        print(f"\n[Warning] Keyboard listener failed to start: {e}")

# ---------------------------------------------------------------------------
# Target I/O & Controller logic
# ---------------------------------------------------------------------------
def load_target(path: Path) -> dict:
    with open(path) as f:
        return json.load(f)

def compute_command(current_result: dict, target: dict):
    if current_result is None or current_result.get("coordinates") is None:
        return 0.0, 0.0, "no detection"

    img_w, img_h = target["image_size"]
    cur_cx, cur_cy = current_result["rotated_rect"]["center"]
    tgt_cx, tgt_cy = target["center"]
    cur_angle = current_result["rotated_rect"]["angle"]
    tgt_angle = target["angle"]

    x_err = tgt_cx - cur_cx
    y_err = tgt_cy - cur_cy
    angle_err = tgt_angle - cur_angle
    if angle_err > 180: angle_err -= 360
    elif angle_err < -180: angle_err += 360

    if (abs(x_err) < ARRIVED_X_PX and
            abs(y_err) < ARRIVED_Y_PX and
            abs(angle_err) < ARRIVED_ANGLE_DEG):
        return 0.0, 0.0, "ARRIVED"

    steer = 0.0
    if abs(x_err) >= DEADBAND_X_PX: steer = KP_STEER * (x_err / img_w)
    throttle = 0.0
    if abs(y_err) >= DEADBAND_Y_PX: throttle = KP_THROTTLE * (y_err / img_h)
    if abs(angle_err) >= DEADBAND_ANGLE_DEG: steer += KP_ANGLE * angle_err

    steer = max(-MAX_STEER, min(MAX_STEER, steer))
    throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, throttle))

    status = (f"x_err={x_err:+.0f} y_err={y_err:+.0f} "
              f"ang_err={angle_err:+.1f} "
              f"-> thr={throttle:+.3f} str={steer:+.3f}")
    return throttle, steer, status

def draw_nav_overlay(vis, current_result, target, throttle, steer, status):
    tgt_corners = target["corners"]
    pts = np.array(tgt_corners, dtype=np.int32)

    tgt_cx, tgt_cy = target["center"]
    cv2.drawMarker(vis, (int(tgt_cx), int(tgt_cy)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

    if current_result and current_result.get("coordinates"):
        cur_corners = current_result["coordinates"]
        pts = np.array(cur_corners, dtype=np.int32)
        cv2.polylines(vis, [pts], isClosed=True, color=(0, 0, 255), thickness=2)
        for pt in cur_corners: cv2.circle(vis, tuple(pt), 6, (0, 0, 255), -1)
        cur_cx, cur_cy = current_result["rotated_rect"]["center"]
        cv2.drawMarker(vis, (int(cur_cx), int(cur_cy)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.arrowedLine(vis, (int(cur_cx), int(cur_cy)), (int(tgt_cx), int(tgt_cy)),
                        (255, 255, 0), 2, tipLength=0.15)

    label = f"throttle={throttle:+.3f} steer={steer:+.3f}"
    color = (0, 255, 0) if status == "ARRIVED" else (0, 255, 255)
    cv2.putText(vis, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    if status == "ARRIVED":
        cv2.putText(vis, "ARRIVED", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

def draw_confirm_overlay(vis, coords, thresh_val):
    if coords is not None:
        pts = np.array(coords, dtype=np.int32)
        cv2.polylines(vis, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
        for pt in coords: cv2.circle(vis, tuple(pt), 8, (0, 255, 0), -1)
        label = f"thresh={thresh_val} | CONTOUR FOUND  [y/n/q]"
    else:
        label = f"thresh={thresh_val} | no contour  [n/q]"
    cv2.putText(vis, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

_ROTATE_MAP = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}

def frame_source(args, raw_streamer=None):
    rot = _ROTATE_MAP.get(args.rotate)
    def _maybe_rotate(f): return cv2.rotate(f, rot) if rot is not None else f

    if args.stdin:
        for f in mjpeg_stdin_frames(raw_streamer=raw_streamer):
            yield _maybe_rotate(f)
    else:
        source = int(args.source) if args.source.isdigit() else args.source
        cap = cv2.VideoCapture(source)
        if not cap.isOpened(): raise SystemExit(f"Error: cannot open source: {args.source}")
        try:
            while True:
                ok, frame = cap.read()
                if not ok: return
                yield _maybe_rotate(frame)
        finally: cap.release()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    global shared_key
    parser = argparse.ArgumentParser(description="Visual servoing: confirm hole, track, navigate.")
    src = parser.add_mutually_exclusive_group()
    src.add_argument("--source", default="0", help='Video source: path or camera index (default "0").')
    src.add_argument("--stdin", action="store_true", help="Read MJPEG from stdin (pipe from rpicam-vid).")
    parser.add_argument("--target", default=str(_DEFAULT_TARGET), help="Path to target.json.")
    parser.add_argument("--every", type=int, default=1, help="Process every Nth frame.")
    parser.add_argument("--display", action="store_true", help="Show cv2 windows (Mac only).")
    parser.add_argument("--stream-to", help="Stream annotated frames over UDP: HOST:PORT")
    parser.add_argument("--raw-stream-to", help="Also forward the raw camera stream")
    parser.add_argument("--skip-confirm", action="store_true", help="Skip interactive confirmation.")
    parser.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270])
    parser.add_argument("--left-roi", type=float, default=LEFT_ROI)
    args = parser.parse_args()

    show_windows = args.display and _can_show_windows()
    streamer = None
    if args.stream_to:
        host, port = args.stream_to.rsplit(":", 1)
        streamer = UDPStreamer(host, int(port))
        print(f"Streaming annotated frames to {host}:{port}")

    raw_streamer = None
    if args.raw_stream_to:
        host, port = args.raw_stream_to.rsplit(":", 1)
        raw_streamer = UDPStreamer(host, int(port))
        print(f"Forwarding raw stream to {host}:{port}")

    target = load_target(Path(args.target))
    print(f"Target loaded: center=({target['center'][0]:.0f}, {target['center'][1]:.0f})")

    frames = frame_source(args, raw_streamer=raw_streamer)

    # Start background keyboard listener if running on stdin
    if args.stdin:
        listener_thread = threading.Thread(target=keyboard_listener, daemon=True)
        listener_thread.start()

    reference_bbox = None
    reference_brightness = None
    reference_area = None
    loop_count = 0

    try:
        # ==============================================================
        # Phase 1  --  Interactive contour confirmation
        # ==============================================================
        if not args.skip_confirm:
            print("\n=== CONTOUR CONFIRMATION ===")
            print("Press  y = confirm    n = next threshold    q = quit")
            print()

            confirmed = False
            thresh_idx = 0
            vis = None
            coords = None

            for frame in frames:
                loop_count += 1
                
                # Check keyboard
                key = shared_key
                if key: shared_key = None # Consume keypress
                
                if show_windows: # Fallback for Mac viewing
                    key_raw = cv2.waitKey(1) & 0xFF
                    if key_raw != 255: key = chr(key_raw)

                if key == 'q':
                    print("User quit during confirmation.")
                    return
                elif key == 'y' and coords is not None:
                    confirmed = True
                    print(f"  CONFIRMED  brightness={reference_brightness:.1f} area={reference_area:.0f}")
                    if show_windows: cv2.destroyWindow("Confirm Contour")
                    break
                elif key == 'n':
                    thresh_idx += 1
                    if thresh_idx >= len(_BRIGHTNESS_THRESHOLDS):
                        raise SystemExit("REPOSITION ROBOT -- all thresholds rejected.")
                    print(f"  Switched to threshold={_BRIGHTNESS_THRESHOLDS[thresh_idx]}")

                # Process every 2nd frame
                if loop_count % 2 == 0:
                    thresh_val = _BRIGHTNESS_THRESHOLDS[thresh_idx]
                    result = process_frame(
                        frame, left_roi_fraction=args.left_roi, display=False,
                        debug=False, verbose=False, brightness_threshold=thresh_val,
                    )
                    coords = result.get("coordinates") if result else None
                    if coords is not None:
                        reference_bbox = result["bbox"]
                        reference_brightness = result.get("mean_brightness")
                        reference_area = result.get("contour_area")

                    vis = frame.copy()
                    draw_confirm_overlay(vis, coords, thresh_val)

                    if show_windows: cv2.imshow("Confirm Contour", vis)

                # Stream every 4th frame
                if streamer and vis is not None and loop_count % 4 == 0:
                    streamer.send(vis, quality=35, scale=0.5)

            if not confirmed:
                raise SystemExit("Video ended before confirmation.")

        # ==============================================================
        # Phase 2  --  Tracking + Navigation
        # ==============================================================
        print("\n=== NAVIGATING ===\n")
        consecutive_failures = 0
        pending_bbox = None
        pending_count = 0
        vis = None

        for frame in frames:
            loop_count += 1

            # Check keyboard to allow quitting mid-navigation
            key = shared_key
            if key: shared_key = None
            if show_windows:
                key_raw = cv2.waitKey(1) & 0xFF
                if key_raw == ord('q'): key = 'q'

            if key == 'q':
                print("User quit during navigation.")
                break

            # Process every 2nd frame
            if loop_count % 2 == 0:
                result = process_frame(
                    frame, left_roi_fraction=args.left_roi,
                    display=False, debug=False, verbose=False,
                )
                coords = result.get("coordinates") if result else None

                # overlap gate
                if coords is not None and reference_bbox is not None:
                    if consecutive_failures < _OVERLAP_GRACE_AFTER:
                        if not _bboxes_overlap(reference_bbox, result["bbox"]):
                            coords = None

                # shadow rejection
                if coords is not None:
                    nb = result.get("mean_brightness")
                    na = result.get("contour_area")
                    if nb is not None and nb > _MAX_HOLE_BRIGHTNESS:
                        coords = None
                    elif reference_brightness is not None and nb is not None:
                        if abs(nb - reference_brightness) > _MAX_BRIGHTNESS_DIFF: coords = None
                    if coords is not None and reference_area is not None and na is not None:
                        ratio = na / reference_area
                        if ratio < _MIN_AREA_RATIO or ratio > _MAX_AREA_RATIO: coords = None

                # temporal confirmation
                if coords is not None and consecutive_failures == 0:
                    reference_bbox = result.get("bbox")
                    pending_bbox = None
                    pending_count = 0
                    consecutive_failures = 0
                elif coords is not None:
                    new_bbox = result["bbox"]
                    if pending_bbox is not None and _bboxes_overlap(pending_bbox, new_bbox):
                        pending_count += 1
                        pending_bbox = new_bbox
                    else:
                        pending_bbox = new_bbox
                        pending_count = 1

                    if pending_count >= _REACQUIRE_CONFIRM:
                        print(f"  Re-acquired hole after {consecutive_failures} frames")
                        reference_bbox = result.get("bbox")
                        consecutive_failures = 0
                        pending_bbox = None
                        pending_count = 0
                    else:
                        coords = None
                else:
                    pending_bbox = None
                    pending_count = 0
                    consecutive_failures += 1
                    if reference_bbox is None and consecutive_failures >= _MAX_CONSECUTIVE_FAILURES:
                        print(f"\nFATAL: No hole for {consecutive_failures} frames. Reposition robot.")
                        break

                # navigation command
                if coords is not None:
                    throttle, steer, status = compute_command(result, target)
                else:
                    throttle, steer, status = 0.0, 0.0, "no detection"

                print(f"frame={loop_count}  {status}")

                # TODO: publish Twist(linear.x=throttle, angular.z=steer) to /cmd_vel_auto

                vis = frame.copy()
                if coords is not None:
                    draw_nav_overlay(vis, result, target, throttle, steer, status)
                else:
                    draw_nav_overlay(vis, None, target, throttle, steer, status)

                if show_windows: cv2.imshow("Navigate", vis)

            # Stream every 4th frame
            if streamer and vis is not None and loop_count % 4 == 0:
                streamer.send(vis, quality=35, scale=0.5)

    finally:
        if streamer: streamer.close()
        if raw_streamer: raw_streamer.close()
        if show_windows: cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
