#!/usr/bin/env python3
"""
Visual servoing: drive the robot until the hole matches the target position.

Reads camera frames, detects the hole, compares to a saved target, and
outputs (throttle, steer) commands as a proportional controller.

Modes
-----
--test <video>   Dry-run with a recorded video.  Prints commands to terminal.
--live           Real camera (source 0).  Prints commands to terminal.
                 (ROS2 integration: swap print for Twist publish on /cmd_vel_auto)

Target
------
Loaded from target.json (same directory).  Generate it by running
hole_detection.py on the ideal frame:

    python3 hole_detection.py --source test.mjpeg --frame 1

then save the output, or use the --save-target flag here.
"""

import json
import argparse
import cv2
import numpy as np
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from hole_detection import process_frame

# ---------------------------------------------------------------------------
# Controller gains  (tune on the real robot)
#
# Sign convention (camera on robot, looking at hole in a wall):
#   positive angular_z  →  robot turns LEFT  →  hole moves RIGHT in frame
#   positive linear_x   →  robot moves FORWARD → hole moves UP in frame
#
# If the robot behaves opposite, flip the sign of the gain.
# ---------------------------------------------------------------------------
KP_STEER = 1.0           # px-error → angular.z (turn to center hole horizontally)
KP_THROTTLE = 0.6         # px-error → linear.x  (drive to center hole vertically)
KP_ANGLE = 0.005          # deg-error → angular.z addition (rotate to match hole angle)

DEADBAND_X_PX = 20        # ignore horizontal error smaller than this
DEADBAND_Y_PX = 20        # ignore vertical   error smaller than this
DEADBAND_ANGLE_DEG = 3.0  # ignore angle       error smaller than this

MAX_STEER = 0.4           # clamp angular.z output
MAX_THROTTLE = 0.3        # clamp linear.x  output

ARRIVED_X_PX = 10         # "close enough" thresholds
ARRIVED_Y_PX = 10
ARRIVED_ANGLE_DEG = 2.0

LEFT_ROI = 0.10           # must match hole_detection setting


# ---------------------------------------------------------------------------
# Target I/O
# ---------------------------------------------------------------------------
_DEFAULT_TARGET = Path(__file__).with_name("target.json")


def load_target(path: Path) -> dict:
    with open(path) as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------
def compute_command(current_result: dict, target: dict):
    """Return (linear_x, angular_z, status_str).

    Returns (0, 0, "no detection") when the hole is not visible.
    Returns (0, 0, "ARRIVED") when error is within thresholds.
    """
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
    # Wrap angle error to [-180, 180]
    if angle_err > 180:
        angle_err -= 360
    elif angle_err < -180:
        angle_err += 360

    if (abs(x_err) < ARRIVED_X_PX and
            abs(y_err) < ARRIVED_Y_PX and
            abs(angle_err) < ARRIVED_ANGLE_DEG):
        return 0.0, 0.0, "ARRIVED"

    # Proportional control with deadband
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
              f"→  throttle={throttle:+.3f}  steer={steer:+.3f}")
    return throttle, steer, status


# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------
def draw_nav_overlay(vis, current_result, target):
    """Draw current detection (red) and target outline (green) on the frame."""
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


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Visual servoing to align hole with target position.")
    parser.add_argument("--source", default="0",
                        help='Video source: path or camera index (default "0").')
    parser.add_argument("--target", default=str(_DEFAULT_TARGET),
                        help="Path to target.json.")
    parser.add_argument("--every", type=int, default=1,
                        help="Process every Nth frame.")
    parser.add_argument("--display", action="store_true",
                        help="Show live overlay (green=target, red=current, yellow=error arrow).")
    parser.add_argument("--left-roi", type=float, default=LEFT_ROI)
    args = parser.parse_args()

    target = load_target(Path(args.target))
    print(f"Target loaded: center=({target['center'][0]:.0f}, {target['center'][1]:.0f})  "
          f"angle={target['angle']:.1f}  area={target['contour_area']:.0f}")

    source = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f"Error: cannot open source: {args.source}")

    frame_idx = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame_idx += 1

            if args.every > 1 and (frame_idx % args.every) != 0:
                if args.display:
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

            throttle, steer, status = compute_command(result, target)

            print(f"frame={frame_idx}  {status}")

            if args.display:
                vis = frame.copy()
                draw_nav_overlay(vis, result, target)

                label = f"throttle={throttle:+.3f}  steer={steer:+.3f}"
                color = (0, 255, 0) if status == "ARRIVED" else (0, 255, 255)
                cv2.putText(vis, label, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                if status == "ARRIVED":
                    cv2.putText(vis, "ARRIVED", (10, 65),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

                cv2.imshow("Navigate", vis)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
