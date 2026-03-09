#!/usr/bin/env python3
"""
ROS2 node that captures frames from a Pi camera (or any OpenCV source) and
publishes hole-detection results.

Published topics
----------------
  hole/detected       std_msgs/Bool              True when a hole is visible
  hole/center         geometry_msgs/PointStamped  Hole centre in image coords (px)
  hole/corners        geometry_msgs/PolygonStamped  4 ordered corners (BL, TL, TR, BR)

Parameters
----------
  camera_source   str    "0" (camera index) or a file/URL/GStreamer pipeline
  frame_width     int    1280
  frame_height    int    720
  process_hz      float  10.0   How often to run detection
  left_roi        float  0.10   Left-ROI fraction for orange calibration
  debug           bool   False  Print extra diagnostics to stdout
"""

import threading

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped, Polygon

from hole_detector.hole_detection import process_frame


class HoleDetectorNode(Node):

    def __init__(self):
        super().__init__("hole_detector")

        self.declare_parameter("camera_source", "0")
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)
        self.declare_parameter("process_hz", 10.0)
        self.declare_parameter("left_roi", 0.10)
        self.declare_parameter("debug", False)

        source_str = self.get_parameter("camera_source").value
        self.frame_width = self.get_parameter("frame_width").value
        self.frame_height = self.get_parameter("frame_height").value
        process_hz = float(self.get_parameter("process_hz").value)
        self.left_roi = float(self.get_parameter("left_roi").value)
        self.debug = bool(self.get_parameter("debug").value)

        source = int(source_str) if source_str.isdigit() else source_str

        self.cap = cv2.VideoCapture(source)
        if self.frame_width and self.frame_height:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera source: {source_str}")
            raise RuntimeError(f"Cannot open camera: {source_str}")

        self.get_logger().info(
            f"Camera opened: source={source_str}  "
            f"{self.frame_width}x{self.frame_height}  "
            f"process_hz={process_hz}"
        )

        self.pub_detected = self.create_publisher(Bool, "hole/detected", 10)
        self.pub_center = self.create_publisher(PointStamped, "hole/center", 10)
        self.pub_corners = self.create_publisher(PolygonStamped, "hole/corners", 10)

        self._frame = None
        self._frame_lock = threading.Lock()
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self._timer = self.create_timer(1.0 / process_hz, self._process)

    # ------------------------------------------------------------------
    # Capture thread — grabs frames as fast as the camera delivers them
    # so we always process the *latest* image, not a stale buffer.
    # ------------------------------------------------------------------
    def _capture_loop(self):
        while rclpy.ok():
            ok, frame = self.cap.read()
            if ok:
                with self._frame_lock:
                    self._frame = frame
            else:
                self.get_logger().warn("Camera read failed", throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # Processing timer
    # ------------------------------------------------------------------
    def _process(self):
        with self._frame_lock:
            frame = self._frame

        if frame is None:
            return

        result = process_frame(
            frame,
            left_roi_fraction=self.left_roi,
            debug=self.debug,
        )

        now = self.get_clock().now().to_msg()
        detected = result.get("detected", False)

        det_msg = Bool()
        det_msg.data = detected
        self.pub_detected.publish(det_msg)

        if not detected:
            if self.debug:
                self.get_logger().info(
                    f"No hole: {result.get('error', '?')}",
                    throttle_duration_sec=2.0,
                )
            return

        center = result["center"]
        center_msg = PointStamped()
        center_msg.header.stamp = now
        center_msg.header.frame_id = "camera"
        center_msg.point.x = center[0]
        center_msg.point.y = center[1]
        center_msg.point.z = 0.0
        self.pub_center.publish(center_msg)

        corners = result["corners"]
        poly_msg = PolygonStamped()
        poly_msg.header.stamp = now
        poly_msg.header.frame_id = "camera"
        poly_msg.polygon = Polygon()
        for c in corners:
            pt = Point32()
            pt.x = float(c[0])
            pt.y = float(c[1])
            pt.z = 0.0
            poly_msg.polygon.points.append(pt)
        self.pub_corners.publish(poly_msg)

        if self.debug:
            cx, cy = center
            w, h = result["size"]
            self.get_logger().info(
                f"Hole detected: center=({cx:.0f},{cy:.0f})  "
                f"size=({w:.0f}x{h:.0f})  angle={result['angle']:.1f}  "
                f"brightness={result['mean_brightness']:.1f}  "
                f"fallback={result['fallback_level']}",
                throttle_duration_sec=0.5,
            )

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HoleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
