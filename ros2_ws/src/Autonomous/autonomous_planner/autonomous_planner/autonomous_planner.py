#!/usr/bin/env python3
import time
from typing import List

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class TimedVelocityAutonomy(Node):
    """
    Subscribes:
      /autonomous_active (std_msgs/Bool)

    Publishes:
      /cmd_vel_auto (geometry_msgs/Twist)

    Behavior:
      - When /autonomous_active == True:
          Executes a sequence of velocity "segments".
          Each segment is (vx, vy, wz, duration_sec).
      - When /autonomous_active == False:
          Immediately publishes zero Twist and stops/resets the plan.
    """

    def __init__(self):
        super().__init__("autonomous_planner")

        # ---- Parameters ----
        # Publish rate for cmd_vel_des while active (Hz)
        self.declare_parameter("publish_hz", 20.0)

        # Loop the plan when done
        self.declare_parameter("loop", False)

        # Segments are flattened groups of 4:
        #   [vx0, vy0, wz0, t0,  vx1, vy1, wz1, t1,  ...]
        # Units: m/s, m/s, rad/s, seconds
        self.declare_parameter(
            "segments",
            [
                0.30, 0.00, 0.00, 1.5,   # forward 1.5s
                0.00, 0.00, 1.00, 0.8,   # rotate left 0.8s
                0.30, 0.00, 0.00, 1.0,   # forward 1.0s
                0.00, 0.00, -1.00, 0.8,  # rotate right 0.8s
                0.25, 0.00, 0.00, 1.2,   # forward 1.2s
            ],
        )

        self.publish_hz: float = float(self.get_parameter("publish_hz").value)
        self.loop: bool = bool(self.get_parameter("loop").value)

        flat = list(self.get_parameter("segments").value)
        self.segments: List[List[float]] = self._parse_segments(flat)

        if not self.segments:
            self.get_logger().warn("No valid segments provided; autonomy will publish zero only.")

        # ---- ROS I/O ----
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_auto", 10)
        self.active_sub = self.create_subscription(Bool, "/autonomous_active", self._on_active, 10)

        # Timer for periodic publishing / state machine
        period = 1.0 / max(self.publish_hz, 1e-6)
        self.timer = self.create_timer(period, self._tick)

        # ---- State ----
        self.autonomous_active = False
        self.running = False
        self.seg_idx = 0
        self.seg_start_t = 0.0

        self.get_logger().info(
            f"autonomous_planner ready. publish_hz={self.publish_hz}, loop={self.loop}, segments={len(self.segments)}"
        )

    def _parse_segments(self, flat: List[float]) -> List[List[float]]:
        segs: List[List[float]] = []
        if len(flat) % 4 != 0:
            self.get_logger().warn(
                f"'segments' length is {len(flat)} (not divisible by 4). "
                "Expected [vx, vy, wz, t] repeated. Extra values will be ignored."
            )

        n = (len(flat) // 4) * 4
        for i in range(0, n, 4):
            vx, vy, wz, dur = float(flat[i]), float(flat[i + 1]), float(flat[i + 2]), float(flat[i + 3])
            if dur <= 0.0:
                self.get_logger().warn(f"Skipping segment {i//4}: duration must be > 0 (got {dur}).")
                continue
            segs.append([vx, vy, wz, dur])
        return segs

    def _on_active(self, msg: Bool) -> None:
        new_active = bool(msg.data)

        # Rising edge: start plan
        if new_active and not self.autonomous_active:
            self.autonomous_active = True
            self._start_plan()

        # Falling edge: stop immediately
        elif (not new_active) and self.autonomous_active:
            self.autonomous_active = False
            self._stop_plan(publish_zero=True)

        # No change: do nothing
        else:
            self.autonomous_active = new_active

    def _start_plan(self) -> None:
        if not self.segments:
            self.get_logger().warn("Autonomy activated but segments list is empty; publishing zero.")
            self.running = False
            self._publish_zero()
            return

        self.running = True
        self.seg_idx = 0
        self.seg_start_t = time.monotonic()
        self.get_logger().info("Autonomy START")

        # Publish first command immediately
        self._publish_current_segment()

    def _stop_plan(self, publish_zero: bool = True) -> None:
        if self.running:
            self.get_logger().info("Autonomy STOP")

        self.running = False
        self.seg_idx = 0
        self.seg_start_t = 0.0

        if publish_zero:
            self._publish_zero()

    def _tick(self) -> None:
        # Always publish zero if inactive (helps downstream controllers settle)
        if not self.autonomous_active or not self.running:
            # Don’t spam logs; just publish zero
            self._publish_zero()
            return

        # Check if current segment duration elapsed
        now = time.monotonic()
        vx, vy, wz, dur = self.segments[self.seg_idx]
        if (now - self.seg_start_t) >= dur:
            self.seg_idx += 1

            # Finished all segments
            if self.seg_idx >= len(self.segments):
                if self.loop:
                    self.seg_idx = 0
                    self.seg_start_t = now
                    self.get_logger().info("Autonomy LOOP -> restarting plan")
                else:
                    self.get_logger().info("Autonomy DONE")
                    self._stop_plan(publish_zero=True)
                    return
            else:
                self.seg_start_t = now

        # Publish current segment command at publish_hz
        self._publish_current_segment()

    def _publish_current_segment(self) -> None:
        vx, vy, wz, _dur = self.segments[self.seg_idx]
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)   # for holonomic; leave 0 for diff-drive
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        msg = Twist()
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = TimedVelocityAutonomy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure we stop motion on shutdown
        node._publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()