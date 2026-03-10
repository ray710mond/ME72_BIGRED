#!/usr/bin/env python3
"""
Competition-ready visual servoing script.

Reads camera frames, confirms the hole contour, tracks it, computes motor
commands to align the hole with a saved target position, and (optionally)
streams annotated video back to a Mac for live viewing.

Usage on Pi (competition day):
  rpicam-vid -n -t 0 --width 1280 --height 720 --framerate 30 \
      --codec mjpeg --buffer-count 1 -o - 2>/dev/null \
    | python3 navigate.py --stdin --stream-to MAC_IP:5000

Usage on Mac (testing with recorded video):
  python3 navigate.py --source test.mjpeg --display --every 1

On Mac (view the Pi stream):
  python3 live_feed.py
"""

import json
import argparse
import socket
import sys
import cv2
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hole_detection import (
    process_frame,
    _can_show_windows,
    _bboxes_overlap,
    _frame_changed,
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
#
# Sign convention (camera on robot, looking at hole in a wall):
#   positive angular_z  ->  robot turns LEFT  ->  hole moves RIGHT in frame
#   positive linear_x   ->  robot moves FORWARD -> hole moves UP in frame
#
# If the robot behaves opposite, flip the sign of the gain.
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

# JPEG quality for the UDP stream to the Mac (lower = smaller packets)
_STREAM_QUALITY = 50
_STREAM_SCALE = 0.5       # resize annotated frames before streaming (1.0 = full res)
_UDP_CHUNK = 32768


# ---------------------------------------------------------------------------
# MJPEG stdin reader  (frame-dropping for real-time)
# ---------------------------------------------------------------------------
_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"


def mjpeg_stdin_frames(raw_streamer=None):
    """Yield the LATEST frame from an MJPEG stream on stdin.

    Drains all buffered data first, parses every complete JPEG, but only
    yields the most recent one.  Earlier frames are dropped so processing
    never falls behind the camera.

    If *raw_streamer* is provided, every raw JPEG is forwarded to it at
    full framerate (before any frame is dropped).
    """
    import os
    import select as _sel

    buf = bytearray()
    fd = sys.stdin.buffer.fileno()

    while True:
        # Block until at least some data arrives
        try:
            chunk = os.read(fd, 131072)
        except OSError:
            return
        if not chunk:
            return
        buf.extend(chunk)

        # Non-blocking drain: grab everything the pipe has buffered
        while _sel.select([fd], [], [], 0)[0]:
            try:
                more = os.read(fd, 131072)
            except OSError:
                break
            if not more:
                break
            buf.extend(more)

        # Parse all complete JPEGs; forward raw bytes; keep only the last
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
    """Send JPEG-encoded frames over UDP for live_feed.py / ffplay."""

    def __init__(self, host: str, port: int):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, frame: np.ndarray):
        """Re-encode a numpy frame as JPEG and send (resized for speed)."""
        if _STREAM_SCALE != 1.0:
            h, w = frame.shape[:2]
            frame = cv2.resize(frame, (int(w * _STREAM_SCALE), int(h * _STREAM_SCALE)))
        _, jpeg = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, _STREAM_QUALITY]
        )
        self.send_raw(jpeg.tobytes())

    def send_raw(self, data: bytes):
        """Send raw JPEG bytes (no re-encoding)."""
        for i in range(0, len(data), _UDP_CHUNK):
            self.sock.sendto(data[i : i + _UDP_CHUNK], self.addr)

    def close(self):
        self.sock.close()


# ---------------------------------------------------------------------------
# Terminal keyboard helper  (works even when stdin is a pipe)
# ---------------------------------------------------------------------------
def _open_tty():
    """Return a file object for the real terminal, or None if unavailable."""
    try:
        return open("/dev/tty", "r")
    except OSError:
        return None


def _tty_key(tty_file):
    """Non-blocking single-char read from the terminal. Returns '' if nothing."""
    if tty_file is None:
        return ""
    import select
    if select.select([tty_file], [], [], 0)[0]:
        return tty_file.readline().strip()
    return ""


# ---------------------------------------------------------------------------
# Target I/O
# ---------------------------------------------------------------------------
def load_target(path: Path) -> dict:
    with open(path) as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------
def compute_command(current_result: dict, target: dict):
    """Return (linear_x, angular_z, status_str)."""
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
    if angle_err > 180:
        angle_err -= 360
    elif angle_err < -180:
        angle_err += 360

    if (abs(x_err) < ARRIVED_X_PX and
            abs(y_err) < ARRIVED_Y_PX and
            abs(angle_err) < ARRIVED_ANGLE_DEG):
        return 0.0, 0.0, "ARRIVED"

    steer = 0.0
    if abs(x_err) >= DEADBAND_X_PX:
        steer = KP_STEER * (x_err / img_w)
    throttle = 0.0
    if abs(y_err) >= DEADBAND_Y_PX:
        throttle = KP_THROTTLE * (y_err / img_h)
    if abs(angle_err) >= DEADBAND_ANGLE_DEG:
        steer += KP_ANGLE * angle_err

    steer = max(-MAX_STEER, min(MAX_STEER, steer))
    throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, throttle))

    status = (f"x_err={x_err:+.0f}  y_err={y_err:+.0f}  "
              f"angle_err={angle_err:+.1f}  "
              f"-> throttle={throttle:+.3f}  steer={steer:+.3f}")
    return throttle, steer, status


# ---------------------------------------------------------------------------
# Overlay drawing
# ---------------------------------------------------------------------------
def draw_nav_overlay(vis, current_result, target, throttle, steer, status):
    """Draw target (green), current detection (red), error arrow, commands."""
    tgt_corners = target["corners"]
    pts = np.array(tgt_corners, dtype=np.int32)
    cv2.polylines(vis, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
    for pt in tgt_corners:
        cv2.circle(vis, tuple(pt), 6, (0, 255, 0), -1)
    tgt_cx, tgt_cy = target["center"]
    cv2.drawMarker(vis, (int(tgt_cx), int(tgt_cy)), (0, 255, 0),
                   cv2.MARKER_CROSS, 20, 2)

    if current_result and current_result.get("coordinates"):
        cur_corners = current_result["coordinates"]
        pts = np.array(cur_corners, dtype=np.int32)
        cv2.polylines(vis, [pts], isClosed=True, color=(0, 0, 255), thickness=2)
        for pt in cur_corners:
            cv2.circle(vis, tuple(pt), 6, (0, 0, 255), -1)
        cur_cx, cur_cy = current_result["rotated_rect"]["center"]
        cv2.drawMarker(vis, (int(cur_cx), int(cur_cy)), (0, 0, 255),
                       cv2.MARKER_CROSS, 20, 2)
        cv2.arrowedLine(vis,
                        (int(cur_cx), int(cur_cy)),
                        (int(tgt_cx), int(tgt_cy)),
                        (255, 255, 0), 2, tipLength=0.15)

    label = f"throttle={throttle:+.3f}  steer={steer:+.3f}"
    color = (0, 255, 0) if status == "ARRIVED" else (0, 255, 255)
    cv2.putText(vis, label, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    if status == "ARRIVED":
        cv2.putText(vis, "ARRIVED", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)


def draw_confirm_overlay(vis, coords, thresh_val):
    if coords is not None:
        pts = np.array(coords, dtype=np.int32)
        cv2.polylines(vis, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
        for pt in coords:
            cv2.circle(vis, tuple(pt), 8, (0, 255, 0), -1)
        label = f"thresh={thresh_val} | CONTOUR FOUND  [y/n/q]"
    else:
        label = f"thresh={thresh_val} | no contour  [n/q]"
    cv2.putText(vis, label, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


# ---------------------------------------------------------------------------
# Frame source abstraction
# ---------------------------------------------------------------------------
_ROTATE_MAP = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


def frame_source(args, raw_streamer=None):
    """Yield frames from either stdin MJPEG or cv2.VideoCapture.

    Applies --rotate if set, so downstream code always sees upright frames.
    When using --stdin, drops buffered frames to stay real-time.
    """
    rot = _ROTATE_MAP.get(args.rotate)

    def _maybe_rotate(f):
        return cv2.rotate(f, rot) if rot is not None else f

    if args.stdin:
        for f in mjpeg_stdin_frames(raw_streamer=raw_streamer):
            yield _maybe_rotate(f)
    else:
        source = int(args.source) if args.source.isdigit() else args.source
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise SystemExit(f"Error: cannot open source: {args.source}")
        try:
            while True:
                ok, frame = cap.read()
                if not ok:
                    return
                yield _maybe_rotate(frame)
        finally:
            cap.release()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Visual servoing: confirm hole, track, navigate.")
    src = parser.add_mutually_exclusive_group()
    src.add_argument("--source", default="0",
                     help='Video source: path or camera index (default "0").')
    src.add_argument("--stdin", action="store_true",
                     help="Read MJPEG from stdin (pipe from rpicam-vid).")
    parser.add_argument("--target", default=str(_DEFAULT_TARGET),
                        help="Path to target.json.")
    parser.add_argument("--every", type=int, default=1,
                        help="Process every Nth frame.")
    parser.add_argument("--display", action="store_true",
                        help="Show cv2 windows (Mac only).")
    parser.add_argument("--stream-to",
                        help="Stream annotated frames over UDP: HOST:PORT "
                             "(e.g. 192.168.1.100:5001).")
    parser.add_argument("--raw-stream-to",
                        help="Also forward the raw camera stream (full framerate, "
                             "no overlays) to a separate HOST:PORT "
                             "(e.g. 192.168.1.100:5000). "
                             "Replaces stream_camera.sh.")
    parser.add_argument("--skip-confirm", action="store_true",
                        help="Skip interactive confirmation, go straight to navigation.")
    parser.add_argument("--no-motion-check", action="store_true",
                        help="Disable motion detection during confirmation (use if Pi camera "
                             "auto-exposure keeps triggering false positives).")
    parser.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270],
                        help="Rotate each frame CW by this many degrees before processing "
                             "(use if camera is mounted sideways). live_feed.py transpose=2 "
                             "= 90 CW, so use --rotate 270 to undo that and process upright.")
    parser.add_argument("--left-roi", type=float, default=LEFT_ROI)
    args = parser.parse_args()

    # --- output targets ---
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
        print(f"Forwarding raw camera stream to {host}:{port}")

    target = load_target(Path(args.target))
    print(f"Target loaded: center=({target['center'][0]:.0f}, {target['center'][1]:.0f})  "
          f"angle={target['angle']:.1f}  area={target['contour_area']:.0f}")

    # tty for keyboard input when stdin is the video pipe
    tty = _open_tty() if args.stdin else None

    frames = frame_source(args, raw_streamer=raw_streamer)
    frame_idx = 0

    # tracking state
    reference_bbox = None
    reference_brightness = None
    reference_area = None

    try:
        # ==============================================================
        # Phase 1  --  Interactive contour confirmation
        # ==============================================================
        if not args.skip_confirm:
            print("\n=== CONTOUR CONFIRMATION ===")
            if args.stdin:
                print("Type  y = confirm    n = next threshold    q = quit  (+ Enter)")
            else:
                print("Press  y = confirm    n = next threshold    q = quit")
            print()

            confirmed = False
            thresh_idx = 0
            reference_confirm_gray = None
            _SETTLE_FRAMES = 60  # let auto-exposure stabilize before checking motion (~2s @ 30fps)
            frames_seen = 0

            for frame in frames:
                frames_seen += 1
                frame_idx += 1

                cur_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if not args.no_motion_check:
                    if frame_idx <= _SETTLE_FRAMES:
                        reference_confirm_gray = cur_gray
                    elif _frame_changed(cur_gray, reference_confirm_gray, hole_mask=None):
                        raise SystemExit(
                            "Robot moved before contour was confirmed. "
                            "Reposition the robot and restart."
                        )

                thresh_val = _BRIGHTNESS_THRESHOLDS[thresh_idx]
                result = process_frame(
                    frame,
                    left_roi_fraction=args.left_roi,
                    display=False,
                    debug=False,
                    verbose=False,
                    brightness_threshold=thresh_val,
                )
                coords = result.get("coordinates") if result else None

                vis = frame.copy()
                draw_confirm_overlay(vis, coords, thresh_val)

                if show_windows:
                    cv2.imshow("Confirm Contour", vis)
                    key_raw = cv2.waitKey(30) & 0xFF
                    key = chr(key_raw) if key_raw < 128 else ""
                elif streamer:
                    streamer.send(vis)
                    key = _tty_key(tty)
                else:
                    key = ""

                if key == "q":
                    print("User quit during confirmation.")
                    return

                if key == "y" and coords is not None:
                    confirmed = True
                    reference_bbox = result["bbox"]
                    reference_brightness = result.get("mean_brightness")
                    reference_area = result.get("contour_area")
                    print(f"  CONFIRMED  brightness={reference_brightness:.1f}  "
                          f"area={reference_area:.0f}")
                    if show_windows:
                        cv2.destroyWindow("Confirm Contour")
                    break

                if key == "n":
                    thresh_idx += 1
                    if thresh_idx >= len(_BRIGHTNESS_THRESHOLDS):
                        raise SystemExit("REPOSITION ROBOT -- all thresholds rejected.")
                    print(f"  Switched to threshold={_BRIGHTNESS_THRESHOLDS[thresh_idx]}")

            if not confirmed:
                if frames_seen == 0:
                    raise SystemExit(
                        "No frames received. rpicam-vid likely failed to start.\n"
                        "  (Run this on the Pi, not the Mac. rpicam-vid is Pi-only.)\n"
                        "  Try: rpicam-vid -n -t 5 -o test.mjpeg   (run alone to see errors)\n"
                        "  If 'device busy': stop launch_all stream first."
                    )
                raise SystemExit(
                    "Video ended before confirmation.\n"
                    "  - Is rpicam-vid still running? (run without 2>/dev/null to see errors)\n"
                    "  - Is the camera in use by another process? (stop launch_all stream first)"
                )

        # ==============================================================
        # Phase 2  --  Tracking + Navigation
        # ==============================================================
        print("\n=== NAVIGATING ===\n")
        consecutive_failures = 0
        pending_bbox = None
        pending_count = 0
        last_coords = None

        for frame in frames:
            frame_idx += 1

            if args.every > 1 and (frame_idx % args.every) != 0:
                if show_windows:
                    cv2.imshow("Navigate", frame)
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        break
                continue

            result = process_frame(
                frame,
                left_roi_fraction=args.left_roi,
                display=False,
                debug=False,
                verbose=False,
            )
            coords = result.get("coordinates") if result else None

            # --- overlap gate ---
            if coords is not None and reference_bbox is not None:
                if consecutive_failures < _OVERLAP_GRACE_AFTER:
                    if not _bboxes_overlap(reference_bbox, result["bbox"]):
                        coords = None

            # --- shadow rejection (brightness + area) ---
            if coords is not None:
                nb = result.get("mean_brightness")
                na = result.get("contour_area")

                if nb is not None and nb > _MAX_HOLE_BRIGHTNESS:
                    coords = None
                elif reference_brightness is not None and nb is not None:
                    if abs(nb - reference_brightness) > _MAX_BRIGHTNESS_DIFF:
                        coords = None
                if coords is not None and reference_area is not None and na is not None:
                    ratio = na / reference_area
                    if ratio < _MIN_AREA_RATIO or ratio > _MAX_AREA_RATIO:
                        coords = None

            # --- accept / temporal confirmation / failure ---
            if coords is not None and consecutive_failures == 0:
                last_coords = coords
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
                    last_coords = coords
                    reference_bbox = result.get("bbox")
                    consecutive_failures = 0
                    pending_bbox = None
                    pending_count = 0
                else:
                    coords = None
            else:
                last_coords = None
                pending_bbox = None
                pending_count = 0
                consecutive_failures += 1
                if (reference_bbox is None and
                        consecutive_failures >= _MAX_CONSECUTIVE_FAILURES):
                    print(f"\nFATAL: No hole for {consecutive_failures} frames. "
                          "Reposition robot.")
                    break

            # --- navigation command ---
            if coords is not None:
                throttle, steer, status = compute_command(result, target)
            else:
                throttle, steer, status = 0.0, 0.0, "no detection"

            print(f"frame={frame_idx}  {status}")

            # TODO: publish Twist(linear.x=throttle, angular.z=steer) to /cmd_vel_auto

            # --- visual output ---
            vis = frame.copy()
            if coords is not None:
                draw_nav_overlay(vis, result, target, throttle, steer, status)
            else:
                draw_nav_overlay(vis, None, target, throttle, steer, status)

            if show_windows:
                cv2.imshow("Navigate", vis)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            if streamer:
                streamer.send(vis)

    finally:
        if tty:
            tty.close()
        if streamer:
            streamer.close()
        if raw_streamer:
            raw_streamer.close()
        if show_windows:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
