#!/usr/bin/env python3
"""
Core hole-detection algorithm extracted from scripts/camera/hole_detection.py.

Only the stateless `process_frame` function and its helpers are kept here so
the ROS2 node (and any future consumer) can import them cleanly.
"""

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------
BRIGHTNESS_THRESHOLDS = list(range(40, 90, 5))  # [40, 45, 50, ..., 85]
MIN_CONTOUR_AREA = 15000
MAX_CONTOUR_AREA = 490000
MAX_CONTOUR_DIM = 1000


def _order_corners(pts, angle=0.0):
    """Return [BL, TL, TR, BR] relative to the rectangle's own axes."""
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


def _detect_hole_contour(gray, global_orange_mask, width, height,
                         left_roi_fraction, brightness_thresh, debug):
    """Core pipeline for a single brightness threshold.

    Returns (target_contour, thresh_image, mean_brightness).
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
        if (area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA
                or w > MAX_CONTOUR_DIM or h > MAX_CONTOUR_DIM):
            continue
        valid_contours.append(cnt)

    if not valid_contours:
        return None, thresh, None

    target_contour = None
    lowest_brightness = 255.0
    for cnt in valid_contours:
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        mean_brightness = cv2.mean(gray, mask=mask)[0]
        if mean_brightness < lowest_brightness:
            lowest_brightness = mean_brightness
            target_contour = cnt

    return target_contour, thresh, lowest_brightness


def process_frame(frame, *, left_roi_fraction: float, debug: bool = False):
    """Detect the hole and return a result dict.

    Always contains:
        detected        bool
        orange_pixels   int
    On success also:
        corners         list of [x, y] (BL, TL, TR, BR)
        center          [float, float]
        size            [float, float]  (width, height of rotated rect)
        angle           float
        bbox            [x, y, w, h]
        contour_area    float
        mean_brightness float
        fallback_level  int
    On failure also:
        error           str
    """
    if frame is None:
        return {"detected": False, "orange_pixels": 0, "error": "no frame"}

    height, width = frame.shape[:2]

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([30, 255, 255])
    global_orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

    left_region_limit = int(width * left_roi_fraction)
    orange_pixels = int(cv2.countNonZero(global_orange_mask[:, :left_region_limit]))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    target_contour = None
    used_level = 0

    for i, thresh_val in enumerate(BRIGHTNESS_THRESHOLDS):
        contour, _, contour_brightness = _detect_hole_contour(
            gray, global_orange_mask, width, height,
            left_roi_fraction, thresh_val, debug,
        )
        if contour is not None:
            target_contour = contour
            used_level = i
            break

    if target_contour is None:
        return {
            "detected": False,
            "orange_pixels": orange_pixels,
            "error": "no hole detected",
        }

    hull = cv2.convexHull(target_contour)
    rect = cv2.minAreaRect(hull)
    box = cv2.boxPoints(rect)
    box = np.intp(box)
    center, (rw, rh), angle = rect

    if rw < rh:
        rw, rh = rh, rw
        angle += 90
    if angle > 90:
        angle -= 180

    corners = _order_corners(box.tolist(), angle)
    ax, ay, aw, ah = cv2.boundingRect(target_contour)

    return {
        "detected": True,
        "orange_pixels": orange_pixels,
        "corners": corners,
        "center": [float(center[0]), float(center[1])],
        "size": [float(rw), float(rh)],
        "angle": float(angle),
        "bbox": [ax, ay, aw, ah],
        "contour_area": float(cv2.contourArea(target_contour)),
        "mean_brightness": contour_brightness,
        "fallback_level": used_level,
    }
