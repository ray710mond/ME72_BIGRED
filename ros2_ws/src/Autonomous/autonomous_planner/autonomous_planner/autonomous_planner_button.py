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
      (none)

    Publishes:
      /cmd_vel_auto (geometry_msgs/Twist)
      /intake_running (std_msgs/Bool)

    Behavior:
      - Starts executing a sequence of velocity "segments" immediately on
        startup and continues until the plan completes (or loops if
        configured).
      - After the plan finishes the node becomes idle.  Control may still be
        overridden by higher-priority topics such as `cmd_vel_teleop` via a
        mux.
    """
    def __init__(self):
        super().__init__("autonomous_planner")

        # ---- Parameters ----
        # Publish rate for cmd_vel_des while active (Hz)
        self.declare_parameter("publish_hz", 20.0)

        # Loop the plan when done
        self.declare_parameter("loop", False)

        # Segments are flattened groups of 4:
        #   [vx0, wz0, intake_flag0, duration_sec,  vx1, wz1, intake_flag1, duration_sec,  ...]
        # Units: m/s, rad/s, bool-as-0/1, seconds
        self.declare_parameter(
            "segments",
            [
                0.1, 0.0, 0.0, 20.0,   # forward 20s, intake off
                0.0, 0.0, 0.0, 1.0,      # stop (duration ignored since it's last)
            ],
        )

        self.publish_hz: float = float(self.get_parameter("publish_hz").value)
        self.loop: bool = bool(self.get_parameter("loop").value)

        flat = list(self.get_parameter("segments").value)
        self.segments: List[List[float]] = self._parse_segments(flat)

        if not self.segments:
            self.get_logger().warn("No valid segments provided; autonomy will publish zero only.")

        # ---- ROS I/O ----
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_des", 10)
        self.intake_pub = self.create_publisher(Bool, "intake_running", 10)

        # # we watch for teleop input so we can abort the autonomous plan
        # self.sub_teleop = self.create_subscription(Twist, "cmd_vel_teleop", self._on_teleop, 10)

        # Timer for periodic publishing / state machine
        period = 1.0 / max(self.publish_hz, 1e-6)
        self.timer = self.create_timer(period, self._tick)

        # ---- State ----
        self.running = False
        self.seg_idx = 0
        self.seg_start_t = 0.0

        self.declare_parameter("start_delay", 0.0)
        self.start_delay: float = float(self.get_parameter("start_delay").value)

        self.get_logger().info(
            f"autonomous_planner ready. publish_hz={self.publish_hz}, loop={self.loop}, segments={len(self.segments)}, start_delay={self.start_delay}"
        )

        if self.start_delay > 0.0:
            self.start_timer = self.create_timer(self.start_delay, self._delayed_start)
        else:
            self._start_plan()

    def _parse_segments(self, flat: List[float]) -> List[List[float]]:
        segs: List[List[float]] = []
        if len(flat) % 4 != 0:
            self.get_logger().warn(
                f"'segments' length is {len(flat)} (not divisible by 4). "
                "Expected [vx, wz, intake, t] repeated. Extra values will be ignored."
            )

        n = (len(flat) // 4) * 4
        for i in range(0, n, 4):
            vx = float(flat[i])
            wz = float(flat[i + 1])
            intake_flag = bool(flat[i + 2])
            dur = float(flat[i + 3])
            if dur <= 0.0:
                self.get_logger().warn(f"Skipping segment {i//4}: duration must be > 0 (got {dur}).")
                continue
            segs.append([vx, wz, intake_flag, dur])
        return segs


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

    def _delayed_start(self) -> None:
        try:
            self.start_timer.cancel()
        except Exception:
            pass
        self._start_plan()

    def _stop_plan(self, publish_zero: bool = False) -> None:
        if self.running:
            self.get_logger().info("Autonomy STOP")

        self.running = False
        self.seg_idx = 0
        self.seg_start_t = 0.0

        if publish_zero:
            self._publish_zero()

    def _tick(self) -> None:
        # Do nothing if plan is not running
        if not self.running:
            return

        # Check if current segment duration elapsed
        now = time.monotonic()
        vx, wz, intake_flag, dur = self.segments[self.seg_idx]
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
        vx, wz, intake_flag, _dur = self.segments[self.seg_idx]
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

        imsg = Bool()
        imsg.data = bool(intake_flag)
        self.intake_pub.publish(imsg)

    def _publish_zero(self) -> None:
        # helper used on shutdown or exceptional cases
        msg = Twist()
        self.cmd_pub.publish(msg)
        imsg = Bool()
        imsg.data = False
        self.intake_pub.publish(imsg)

    # def _on_teleop(self, msg: Twist) -> None:
    #     # teleop override received; stop the autonomous plan permanently
    #     # if not self.running:
    #     #     return

    #     # # ignore zero commands that may be delivered from QoS history when the
    #     # # teleop node starts or if no input is being sent yet
    #     # if (
    #     #     abs(msg.linear.x) < 1e-3
    #     #     and abs(msg.linear.y) < 1e-3
    #     #     and abs(msg.angular.z) < 1e-3
    #     # ):
    #     #     return

    #     # self.get_logger().info("Teleop received – aborting autonomous plan")
    #     # self._stop_plan(publish_zero=True)


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