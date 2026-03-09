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
_BRIGHTNESS_THRESHOLDS = [45, 60, 75]  # tried in order; first to produce a valid contour wins
_MIN_CONTOUR_AREA = 16000
_MAX_CONTOUR_AREA = 490000
_MAX_CONTOUR_DIM = 1000
_MAX_SHADOW_BRIGHTNESS = 80     # contours brighter than this are shadows, not void
_MIN_SOLIDITY = 0.5             # reject very irregular shapes (amorphous shadows)
_MAX_CONSECUTIVE_FAILURES = 10  # headless mode: terminate after this many frames w/o detection

_FRAME_CHANGE_THRESHOLD = 12.0  # mean diff after heavy blur; shadows < this < robot movement
_HOLE_MASK_MARGIN = 50          # px to expand hole mask for pre-confirmation stability check
_MIN_OVERLAP_IOU = 0.3          # minimum bbox IoU to accept a contour frame-to-frame
_SEARCH_STATUS_INTERVAL = 30    # print "Searching..." every N failed processed frames


def _can_show_windows() -> bool:
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        return False
    return True


def _order_corners(pts):
    """Order 4 points clockwise: top-left, top-right, bottom-right, bottom-left."""
    sorted_by_y = sorted(pts, key=lambda p: p[1])
    top = sorted(sorted_by_y[:2], key=lambda p: p[0])
    bot = sorted(sorted_by_y[2:], key=lambda p: p[0])
    return [top[0], top[1], bot[1], bot[0]]


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

    Returns (target_contour, thresh_image) -- contour is None when nothing found.
    """
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    _, thresh = cv2.threshold(blur, brightness_thresh, 255, cv2.THRESH_BINARY_INV)

    thresh[global_orange_mask > 0] = 0

    mask_width = int(width * left_roi_fraction)
    hinge_x0 = mask_width
    hinge_x1 = hinge_x0 + int(width * 0.23)
    hinge_y0 = int(height * 0.45)
    hinge_y1 = hinge_y0 + int(height * 0.3)
    cv2.rectangle(thresh, (0, 0), (mask_width, height), 0, -1)
    cv2.rectangle(thresh, (hinge_x0, hinge_y0), (hinge_x1, hinge_y1), 0, -1)

    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, thresh

    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        _, _, w, h = cv2.boundingRect(cnt)

        if area < _MIN_CONTOUR_AREA or area > _MAX_CONTOUR_AREA or w > _MAX_CONTOUR_DIM or h > _MAX_CONTOUR_DIM:
            continue

        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        solidity = area / hull_area if hull_area > 0 else 0
        if solidity < _MIN_SOLIDITY:
            if debug:
                print(f"    rejected solidity={solidity:.2f}  area={area:.0f}")
            continue

        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        mean_brightness = cv2.mean(gray, mask=mask)[0]
        if mean_brightness > _MAX_SHADOW_BRIGHTNESS:
            if debug:
                print(f"    rejected brightness={mean_brightness:.1f}  area={area:.0f}")
            continue

        candidates.append((cnt, mean_brightness, area, solidity))

    if not candidates:
        return None, thresh

    candidates.sort(key=lambda c: c[1])
    if debug:
        for cnt, bright, area, sol in candidates:
            tag = " <-- selected" if cnt is candidates[0][0] else ""
            print(f"    candidate  area={area:.0f}  brightness={bright:.1f}  solidity={sol:.2f}{tag}")

    return candidates[0][0], thresh


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
        coordinates     list of [x, y] (TL, TR, BR, BL) or None
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

    if verbose and orange_pixels < 30000:
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

        contour, last_thresh = _detect_hole_contour(
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
    # ------------------------------------------------------------------
    rect = cv2.minAreaRect(target_contour)
    box = cv2.boxPoints(rect)
    box = np.intp(box)
    center, (rw, rh), angle = rect

    corners = _order_corners(box.tolist())
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
    parser.add_argument("--every", type=int, default=5, help="Process every Nth frame.")
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
        reference_bbox = None   # rolling: updated every accepted frame

        # ----------------------------------------------------------
        # Phase 1  --  Interactive contour confirmation
        # Requires a display window; skipped in headless mode.
        # ----------------------------------------------------------
        if show_windows:
            # Stability check: read two frames and make sure the scene
            # isn't actively changing (robot still moving into position).
            prev_frame = None
            frame = None
            for _ in range(2):
                ok, f = cap.read()
                if not ok:
                    raise SystemExit("Error: could not read frames for confirmation.")
                frame_idx += 1
                prev_frame = frame
                frame = f

            if prev_frame is not None:
                g1 = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
                g2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if _frame_changed(g2, g1, hole_mask=None):
                    raise SystemExit(
                        "Scene is changing -- robot may still be moving. "
                        "Wait for it to settle and restart."
                    )

            print("\n=== CONTOUR CONFIRMATION ===")
            print("Press  y = confirm    n = next threshold    q = quit\n")

            confirmed = False
            for thresh_val in _BRIGHTNESS_THRESHOLDS:
                result = process_frame(
                    frame,
                    left_roi_fraction=args.left_roi,
                    display=args.display,
                    debug=args.debug,
                    verbose=False,
                    brightness_threshold=thresh_val,
                )
                coords = result.get("coordinates") if result else None

                if coords is None:
                    print(f"  No contour at threshold={thresh_val}, trying next...")
                    continue

                vis = frame.copy()
                pts = np.array(coords, dtype=np.int32)
                cv2.polylines(vis, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
                for pt in coords:
                    cv2.circle(vis, tuple(pt), 8, (0, 255, 0), -1)
                cv2.imshow("Confirm Contour", vis)

                print(f"  Contour found (threshold={thresh_val}). Is this the hole?  [y / n / q]")

                while True:
                    key = cv2.waitKey(0) & 0xFF
                    if key in (ord("y"), ord("n"), ord("q")):
                        break

                if key == ord("q"):
                    print("User quit during confirmation.")
                    return

                if key == ord("y"):
                    confirmed = True
                    reference_bbox = result["bbox"]
                    last_coords = coords
                    last_bbox = reference_bbox
                    print(f"  CONFIRMED  corners={_fmt_corners(coords)}\n")
                    cv2.destroyWindow("Confirm Contour")
                    break

                print("  Rejected, trying next threshold...")

            if not confirmed:
                raise SystemExit(
                    "REPOSITION ROBOT -- no valid hole confirmed after all thresholds."
                )

            # Flush stale frames that buffered while the user was deciding
            for _ in range(30):
                cap.grab()

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
            if coords is not None and reference_bbox is not None:
                if not _bboxes_overlap(reference_bbox, result["bbox"]):
                    if args.debug:
                        print(f"  frame={frame_idx}  rejected: no overlap with reference")
                    coords = None

            # --- accept or count failure ---
            if coords is not None:
                if consecutive_failures > 0:
                    print(f"  Re-acquired hole after {consecutive_failures} frames")
                last_coords = coords
                last_bbox = result.get("bbox")
                reference_bbox = last_bbox  # rolling update
                consecutive_failures = 0
                lvl = result.get("fallback_level", 0)
                lvl_str = f"  [fallback={lvl}]" if lvl > 0 else ""
                print(f"frame={frame_idx}{lvl_str}  corners={_fmt_corners(coords)}")
            else:
                consecutive_failures += 1
                if reference_bbox is None and consecutive_failures >= _MAX_CONSECUTIVE_FAILURES:
                    # Headless mode (no confirmation): give up quickly
                    print(
                        f"\nFATAL: No hole detected for {consecutive_failures} "
                        f"consecutive processed frames. Reposition the robot and restart."
                    )
                    break
                if consecutive_failures % _SEARCH_STATUS_INTERVAL == 0:
                    print(f"  Searching for hole... ({consecutive_failures} frames)")

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

# python3 scripts/camera/train.py --source scripts/camera/test.mjpeg --frame 815 --display
# python3 scripts/camera/train.py --display
# Live Script: python3 scripts/camera/train.py --source 0 --display
# python3 scripts/camera/train.py --source scripts/camera/test2.mjpeg --display


# TODO Add upper bound for tolderance of box outtake detection
# TODO Add color detection tolerance for other bot outtake.
