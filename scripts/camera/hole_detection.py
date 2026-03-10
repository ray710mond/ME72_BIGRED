#!/usr/bin/env python3

# Pixel input must be: height: 720 width: 1280

import os
import cv2
import sys
import argparse
import numpy as np
from pathlib import Path

# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------
_BRIGHTNESS_THRESHOLDS = list(range(40, 90, 5))  # [40, 45, 50, ..., 85]
_MIN_CONTOUR_AREA = 15000
_MAX_CONTOUR_AREA = 490000
_MAX_CONTOUR_DIM = 1000
_MAX_CONSECUTIVE_FAILURES = 10  # headless mode: terminate after this many frames w/o detection

_FRAME_CHANGE_THRESHOLD = 12.0  # mean diff after heavy blur; shadows < this < robot movement
_HOLE_MASK_MARGIN = 50          # px to expand hole mask for pre-confirmation stability check
_MIN_OVERLAP_IOU = 0.3          # minimum bbox IoU to accept a contour frame-to-frame
_OVERLAP_GRACE_AFTER = 15       # after this many failures, drop overlap requirement (re-acquire)
_MAX_BRIGHTNESS_DIFF = 10       # max brightness difference vs confirmed hole
_MAX_HOLE_BRIGHTNESS = 50       # absolute cap: a real void is never brighter than this
_MIN_AREA_RATIO = 0.25          # contour area must be >= 25% of confirmed area
_MAX_AREA_RATIO = 3.0           # contour area must be <= 300% of confirmed area
_REACQUIRE_CONFIRM = 7          # consecutive frames before accepting after a gap


def _can_show_windows() -> bool:
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        return False
    return True


def _order_corners(pts, angle=0.0):
    """Order 4 corners consistently relative to the rectangle's own axes.

    Returns [BL, TL, TR, BR] where the labels refer to the hole's
    physical edges (not the image frame).  Counter-rotates the points so
    the rectangle is upright before sorting, which prevents corner-label
    flipping at steep angles.
    """
    center = np.mean(pts, axis=0)
    rad = np.radians(-angle)
    cos_a, sin_a = np.cos(rad), np.sin(rad)

    unrotated = []
    for pt in pts:
        dx, dy = pt[0] - center[0], pt[1] - center[1]
        ux = dx * cos_a - dy * sin_a
        uy = dx * sin_a + dy * cos_a
        unrotated.append((ux, uy, pt))

    sorted_by_y = sorted(unrotated, key=lambda r: r[1])
    top = sorted(sorted_by_y[:2], key=lambda r: r[0])
    bot = sorted(sorted_by_y[2:], key=lambda r: r[0])
    return [list(top[0][2]), list(top[1][2]), list(bot[1][2]), list(bot[0][2])]


# ---------------------------------------------------------------------------
# Tracking helpers
# ---------------------------------------------------------------------------
def _bboxes_overlap(bbox1, bbox2, min_iou=_MIN_OVERLAP_IOU):
    """True when two axis-aligned bounding boxes share enough area."""
    x1, y1, w1, h1 = bbox1
    x2, y2, w2, h2 = bbox2
    ix = max(x1, x2)
    iy = max(y1, y2)
    iw = min(x1 + w1, x2 + w2) - ix
    ih = min(y1 + h1, y2 + h2) - iy
    if iw <= 0 or ih <= 0:
        return False
    intersection = iw * ih
    union = w1 * h1 + w2 * h2 - intersection
    return (intersection / union if union > 0 else 0) >= min_iou


def _make_hole_mask(shape, bbox, margin=_HOLE_MASK_MARGIN):
    """Binary mask covering the hole bbox + margin (excluded from frame-change diff)."""
    mask = np.zeros(shape[:2], dtype=np.uint8)
    x, y, w, h = bbox
    h_img, w_img = shape[:2]
    cv2.rectangle(
        mask,
        (max(0, x - margin), max(0, y - margin)),
        (min(w_img, x + w + margin), min(h_img, y + h + margin)),
        255, -1,
    )
    return mask


def _frame_changed(current_gray, reference_gray, hole_mask,
                   threshold=_FRAME_CHANGE_THRESHOLD):
    """Detect large scene change (robot moved) while ignoring the hole interior and shadows."""
    curr = cv2.GaussianBlur(current_gray, (31, 31), 0)
    ref = cv2.GaussianBlur(reference_gray, (31, 31), 0)
    compare_mask = 255 * np.ones(current_gray.shape[:2], dtype=np.uint8)
    if hole_mask is not None:
        compare_mask[hole_mask > 0] = 0
    diff = cv2.absdiff(curr, ref)
    return cv2.mean(diff, mask=compare_mask)[0] > threshold


# ---------------------------------------------------------------------------
# Core detection
# ---------------------------------------------------------------------------
def _detect_hole_contour(
    gray,
    global_orange_mask,
    width,
    height,
    left_roi_fraction,
    brightness_thresh,
    debug,
):
    """Core detection pipeline with configurable thresholds.

    Returns (target_contour, thresh_image, mean_brightness).
    contour and mean_brightness are None when nothing found.
    """
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    _, thresh = cv2.threshold(blur, brightness_thresh, 255, cv2.THRESH_BINARY_INV)

    thresh[global_orange_mask > 0] = 0

    mask_width = int(width * left_roi_fraction)
    hinge_x0 = mask_width
    hinge_x1 = hinge_x0 + int(width * 0.13)
    hinge_y0 = int(height * 0.45)
    hinge_y1 = hinge_y0 + int(height * 0.3)
    cv2.rectangle(thresh, (0, 0), (mask_width, height), 0, -1)
    cv2.rectangle(thresh, (hinge_x0, hinge_y0), (hinge_x1, hinge_y1), 0, -1)

    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, thresh, None

    valid_contours = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        _, _, w, h = cv2.boundingRect(cnt)

        if area < _MIN_CONTOUR_AREA or area > _MAX_CONTOUR_AREA or w > _MAX_CONTOUR_DIM or h > _MAX_CONTOUR_DIM:
            continue

        valid_contours.append(cnt)

    if not valid_contours:
        return None, thresh, None

    # Pick the darkest contour (lowest mean brightness = deepest void)
    target_contour = None
    lowest_brightness = 255.0
    for cnt in valid_contours:
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        mean_brightness = cv2.mean(gray, mask=mask)[0]
        if debug:
            print(f"    candidate  area={cv2.contourArea(cnt):.0f}  brightness={mean_brightness:.1f}")
        if mean_brightness < lowest_brightness:
            lowest_brightness = mean_brightness
            target_contour = cnt

    return target_contour, thresh, lowest_brightness


def process_frame(
    frame,
    *,
    left_roi_fraction: float,
    display: bool,
    debug: bool,
    verbose: bool = True,
    brightness_threshold: int | None = None,
):
    """Detect the hole and return 4 rotation-aligned corners.

    Parameters
    ----------
    brightness_threshold : int or None
        If set, only this single threshold is tried (used during interactive
        confirmation).  If *None*, all ``_BRIGHTNESS_THRESHOLDS`` are tried
        in order (automatic fallback).

    Return dict always contains:
        coordinates     list of [x, y] (BL, TL, TR, BR) or None
        orange_pixels   int
    On success, also:
        bbox, rotated_rect, fallback_level
    On failure, also:
        error           str
    """
    if frame is None:
        return None

    img = frame
    height, width = img.shape[:2]
    show_windows = debug and _can_show_windows()

    # ------------------------------------------------------------------
    # Camera calibration check (orange detection)
    # ------------------------------------------------------------------
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([30, 255, 255])
    global_orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

    left_region_limit = int(width * left_roi_fraction)
    orange_pixels = cv2.countNonZero(global_orange_mask[:, :left_region_limit])

    if debug and show_windows:
        roi_display = np.zeros_like(global_orange_mask)
        roi_display[:, :left_region_limit] = global_orange_mask[:, :left_region_limit]
        cv2.imshow("Orange Pixels (Left ROI)", roi_display)

    if verbose and (orange_pixels < 30000 or orange_pixels > 40000):
        print("WARNING: Robot outtake not detected! Camera may have shifted.")

    # ------------------------------------------------------------------
    # Hole detection (single threshold or full fallback)
    # ------------------------------------------------------------------
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    thresholds = (
        [brightness_threshold] if brightness_threshold is not None
        else list(_BRIGHTNESS_THRESHOLDS)
    )

    target_contour = None
    used_level = 0
    last_thresh = None

    for i, thresh_val in enumerate(thresholds):
        if debug:
            print(f"  [level {i}] thresh={thresh_val}")

        contour, last_thresh, contour_brightness = _detect_hole_contour(
            gray, global_orange_mask, width, height, left_roi_fraction,
            thresh_val, debug,
        )
        if contour is not None:
            target_contour = contour
            used_level = i
            break

    if debug and show_windows and last_thresh is not None:
        cv2.imshow("Thresholded Image with Mask", last_thresh)

    if target_contour is None:
        return {
            "coordinates": None,
            "orange_pixels": orange_pixels,
            "error": "REPOSITION ROBOT -- no hole detected after all attempts",
        }

    # ------------------------------------------------------------------
    # Rotation-aligned bounding rectangle
    # Convex hull removes concavities from the robot arm inside the hole,
    # so the fitted rectangle aligns with the hole geometry, not the arm.
    # ------------------------------------------------------------------
    hull = cv2.convexHull(target_contour)
    rect = cv2.minAreaRect(hull)
    box = cv2.boxPoints(rect)
    box = np.intp(box)
    center, (rw, rh), angle = rect

    # Normalize so rw is always the longer side and angle stays in [-90, 90]
    if rw < rh:
        rw, rh = rh, rw
        angle += 90
    if angle > 90:
        angle -= 180

    corners = _order_corners(box.tolist(), angle)
    ax, ay, aw, ah = cv2.boundingRect(target_contour)

    if debug and show_windows:
        vis = img.copy()
        cv2.drawContours(vis, [box], 0, (0, 255, 0), 2)
        cv2.drawContours(vis, [target_contour], -1, (255, 0, 0), 1)
        cv2.rectangle(vis, (ax, ay), (ax + aw, ay + ah), (0, 255, 255), 1)
        for pt in corners:
            cv2.circle(vis, tuple(pt), 6, (0, 0, 255), -1)
        cv2.imshow("Hole Detection", vis)

    return {
        "coordinates": corners,
        "orange_pixels": orange_pixels,
        "bbox": [ax, ay, aw, ah],
        "rotated_rect": {
            "center": [float(center[0]), float(center[1])],
            "size": [float(rw), float(rh)],
            "angle": float(angle),
        },
        "fallback_level": used_level,
        "mean_brightness": contour_brightness,
        "contour_area": float(cv2.contourArea(target_contour)),
    }


# ==========================================
# CLI helpers
# ==========================================
def _parse_source(source: str):
    s = source.strip()
    if s.isdigit():
        return int(s)
    return s


def _fmt_corners(corners):
    return "  ".join(f"({c[0]},{c[1]})" for c in corners)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--source",
        default=str(Path(__file__).with_name("test.mjpeg")),
        help='Video source: path, URL, or camera index (e.g. "0").',
    )
    parser.add_argument("--every", type=int, default=1, help="Process every Nth frame.")
    parser.add_argument(
        "--frame",
        type=int,
        default=None,
        help="Process only this 1-indexed frame number (e.g. 815).",
    )
    parser.add_argument("--left-roi", type=float, default=0.10, help="Left ROI fraction (0-1).")

    viz = parser.add_mutually_exclusive_group()
    viz.add_argument(
        "--display",
        action="store_true",
        help="Show only detections overlay. Press 'q' to quit.",
    )
    viz.add_argument(
        "--debug",
        action="store_true",
        help="Show debug windows + masking boxes. Press 'q' to quit.",
    )
    args = parser.parse_args()

    source = _parse_source(args.source)
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f"Error: Could not open source: {args.source}")

    frame_idx = 0
    show_windows = (args.display or args.debug) and _can_show_windows()
    last_coords = None
    last_bbox = None

    def _draw_overlay(vis):
        if last_coords and len(last_coords) == 4:
            pts = np.array(last_coords, dtype=np.int32)
            cv2.polylines(vis, [pts], isClosed=True, color=(0, 0, 255), thickness=2)
            for pt in last_coords:
                cv2.circle(vis, tuple(pt), 8, (0, 0, 255), -1)
        if args.debug and last_bbox:
            bx, by, bw, bh = last_bbox
            cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), (0, 255, 255), 1)
        cv2.imshow("Detections", vis)

    try:
        # ==============================================================
        # Single-frame mode (no confirmation, no tracking)
        # ==============================================================
        if args.frame is not None:
            target = args.frame
            if target < 1:
                raise SystemExit("--frame must be >= 1")

            cap.set(cv2.CAP_PROP_POS_FRAMES, float(target - 1))
            while frame_idx < target:
                ok, frame = cap.read()
                if not ok:
                    raise SystemExit(f"Error: Could not read frame {target}.")
                frame_idx += 1

            result = process_frame(
                frame,
                left_roi_fraction=args.left_roi,
                display=args.display,
                debug=args.debug,
                verbose=args.debug,
            )
            coords = result.get("coordinates") if result else None
            if coords is None:
                print(f"ERROR: frame={frame_idx}  {result.get('error', 'detection failed')}")
            else:
                print(f"frame={frame_idx}  corners={_fmt_corners(coords)}")
                last_coords = coords
                last_bbox = result.get("bbox")

            if show_windows:
                _draw_overlay(frame.copy())
                cv2.waitKey(0)
            return

        # ==============================================================
        # Streaming mode
        # ==============================================================
        reference_bbox = None        # rolling: updated every accepted frame
        reference_brightness = None  # mean brightness of confirmed hole contour
        reference_area = None        # contour area of confirmed hole

        # ----------------------------------------------------------
        # Phase 1  --  Interactive contour confirmation
        # Requires a display window; skipped in headless mode.
        # ----------------------------------------------------------
        if show_windows:
            # Live confirmation loop: video streams continuously while the
            # user watches.  Press 'y' to lock in the current contour,
            # 'n' to try the next brightness threshold, 'q' to quit.
            # If the robot moves before the user confirms, the script
            # cancels (frame-change detection against the first frame).
            print("\n=== CONTOUR CONFIRMATION (live) ===")
            print("Press  y = confirm    n = next threshold    q = quit\n")

            confirmed = False
            thresh_idx = 0
            reference_confirm_gray = None
            last_confirm_result = None

            while not confirmed:
                ok, frame = cap.read()
                if not ok:
                    raise SystemExit("Error: video ended before confirmation.")
                frame_idx += 1

                cur_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                if reference_confirm_gray is None:
                    reference_confirm_gray = cur_gray
                elif _frame_changed(cur_gray, reference_confirm_gray, hole_mask=None):
                    cv2.destroyAllWindows()
                    raise SystemExit(
                        "Robot moved before contour was confirmed. "
                        "Reposition the robot and restart."
                    )

                thresh_val = _BRIGHTNESS_THRESHOLDS[thresh_idx]
                result = process_frame(
                    frame,
                    left_roi_fraction=args.left_roi,
                    display=args.display,
                    debug=args.debug,
                    verbose=True,
                    brightness_threshold=thresh_val,
                )
                coords = result.get("coordinates") if result else None
                last_confirm_result = result if coords is not None else last_confirm_result

                vis = frame.copy()
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
                cv2.imshow("Confirm Contour", vis)

                key = cv2.waitKey(30) & 0xFF

                if key == ord("q"):
                    print("User quit during confirmation.")
                    return

                if key == ord("y") and coords is not None:
                    confirmed = True
                    reference_bbox = result["bbox"]
                    reference_brightness = result.get("mean_brightness")
                    reference_area = result.get("contour_area")
                    last_coords = coords
                    last_bbox = reference_bbox
                    print(f"  CONFIRMED  brightness={reference_brightness:.1f}  "
                          f"area={reference_area:.0f}  corners={_fmt_corners(coords)}\n")
                    cv2.destroyWindow("Confirm Contour")

                elif key == ord("n"):
                    thresh_idx += 1
                    if thresh_idx >= len(_BRIGHTNESS_THRESHOLDS):
                        cv2.destroyAllWindows()
                        raise SystemExit(
                            "REPOSITION ROBOT -- all thresholds rejected."
                        )
                    print(f"  Switched to threshold={_BRIGHTNESS_THRESHOLDS[thresh_idx]}")

        # ----------------------------------------------------------
        # Phase 2  --  Tracking loop
        #
        # After confirmation the robot is free to move.  The hole may
        # temporarily leave the frame; we keep searching and re-acquire
        # it when it reappears near its last known position (overlap
        # check).  No frame-change detection here -- movement is
        # expected.
        #
        # Headless (no confirmation): plain detection with a strict
        # failure limit since there is no reference to re-acquire against.
        # ----------------------------------------------------------
        consecutive_failures = 0
        pending_bbox = None
        pending_count = 0

        while True:
            ok, frame = cap.read()
            if not ok:
                break

            frame_idx += 1
            if args.every > 1 and (frame_idx % args.every) != 0:
                if show_windows:
                    _draw_overlay(frame.copy())
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        break
                continue

            # --- detect ---
            result = process_frame(
                frame,
                left_roi_fraction=args.left_roi,
                display=args.display,
                debug=args.debug,
                verbose=args.debug,
            )
            coords = result.get("coordinates") if result else None

            # --- overlap gate (only when we have a confirmed reference) ---
            # After a long gap the hole may return at a different position/angle,
            # so the overlap requirement is dropped to allow re-acquisition.
            if coords is not None and reference_bbox is not None:
                if consecutive_failures < _OVERLAP_GRACE_AFTER:
                    if not _bboxes_overlap(reference_bbox, result["bbox"]):
                        if args.debug:
                            print(f"  frame={frame_idx}  rejected: no overlap with reference")
                        coords = None

            # --- shadow rejection gates (brightness + area) ---
            if coords is not None:
                new_brightness = result.get("mean_brightness")
                new_area = result.get("contour_area")

                # Absolute brightness cap: a void is never this bright
                if new_brightness is not None and new_brightness > _MAX_HOLE_BRIGHTNESS:
                    if args.debug:
                        print(f"  frame={frame_idx}  rejected: brightness {new_brightness:.1f} "
                              f"> absolute cap {_MAX_HOLE_BRIGHTNESS}")
                    coords = None

                # Relative brightness: must be similar to confirmed hole
                elif reference_brightness is not None and new_brightness is not None:
                    if abs(new_brightness - reference_brightness) > _MAX_BRIGHTNESS_DIFF:
                        if args.debug:
                            print(f"  frame={frame_idx}  rejected: brightness {new_brightness:.1f} "
                                  f"vs reference {reference_brightness:.1f}")
                        coords = None

                # Area similarity: reject contours far from confirmed size
                if coords is not None and reference_area is not None and new_area is not None:
                    ratio = new_area / reference_area
                    if ratio < _MIN_AREA_RATIO or ratio > _MAX_AREA_RATIO:
                        if args.debug:
                            print(f"  frame={frame_idx}  rejected: area ratio {ratio:.2f} "
                                  f"(area={new_area:.0f} vs ref={reference_area:.0f})")
                        coords = None

            # --- accept or count failure ---
            if coords is not None and consecutive_failures == 0:
                # Normal tracking: accept immediately
                last_coords = coords
                last_bbox = result.get("bbox")
                reference_bbox = last_bbox
                pending_bbox = None
                pending_count = 0
                lvl = result.get("fallback_level", 0)
                lvl_str = f"  [fallback={lvl}]" if lvl > 0 else ""
                print(f"frame={frame_idx}{lvl_str}  corners={_fmt_corners(coords)}")

            elif coords is not None:
                # Re-acquiring after a gap: require temporal confirmation
                # to avoid accepting a 1-2 frame shadow flicker.
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
                    last_bbox = result.get("bbox")
                    reference_bbox = last_bbox
                    consecutive_failures = 0
                    pending_bbox = None
                    pending_count = 0
                    lvl = result.get("fallback_level", 0)
                    lvl_str = f"  [fallback={lvl}]" if lvl > 0 else ""
                    print(f"frame={frame_idx}{lvl_str}  corners={_fmt_corners(coords)}")
                else:
                    print(f"frame={frame_idx}  Confirming contour ({pending_count}/{_REACQUIRE_CONFIRM})...")

            else:
                last_coords = None
                last_bbox = None
                pending_bbox = None
                pending_count = 0
                consecutive_failures += 1
                print(f"frame={frame_idx}  No contour detected")
                if reference_bbox is None and consecutive_failures >= _MAX_CONSECUTIVE_FAILURES:
                    print(
                        f"\nFATAL: No hole detected for {consecutive_failures} "
                        f"consecutive processed frames. Reposition the robot and restart."
                    )
                    break

            if show_windows:
                _draw_overlay(frame.copy())
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    finally:
        cap.release()
        if show_windows:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

# python3 scripts/camera/hole_detection.py --source scripts/camera/test.mjpeg --frame 815 --display
# python3 scripts/camera/hole_detection.py --display
# Live Script: python3 scripts/camera/hole_detection.py --source 0 --display
# python3 scripts/camera/hole_detection.py --source scripts/camera/test2.mjpeg --display


# TODO Add color detection tolerance for other bot outtake.
# TODO what outputs should we print for the robot movements? (center and length?)
