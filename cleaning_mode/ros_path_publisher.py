"""ROS 2 nav_msgs/Path 发布节点 (A6 cleaning_mode).

只发布 nav_msgs/Path,不接管 /cmd_vel;MPC 是 /cmd_vel 唯一写者。
接收 cleaning_mode.make_plan() 输出的 CleaningPlan,转 nav_msgs/Path 发布。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


@dataclass(frozen=True)
class WorldXY:
    """A point in the world frame."""

    x: float
    y: float


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def to_ros_path(
    xy: list[tuple[float, float]],
    frame_id: str = "map",
    stamp=None,
) -> Path:
    """Convert a sequence of (x, y) tuples to a nav_msgs/Path message.

    Each pose's yaw is computed from the segment direction.
    """
    msg = Path()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    for index, (x, y) in enumerate(xy):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = msg.header.stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        if len(xy) == 1:
            yaw = 0.0
        elif index + 1 < len(xy):
            nx, ny = xy[index + 1]
            yaw = math.atan2(ny - y, nx - x)
        else:
            px, py = xy[index - 1]
            yaw = math.atan2(y - py, x - px)
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        msg.poses.append(pose)
    return msg


class CleaningPathPublisher(Node):
    """Publishes a single nav_msgs/Path derived from a cleaning plan.

    Lifecycle:
    - on_configure(): create publisher
    - publish_plan(xy, frame_id): publish once and store
    - on_cleanup(): destroy publisher
    """

    def __init__(self, node_name: str = "cleaning_path_publisher") -> None:
        super().__init__(node_name)
        self._publisher = None
        self._last_path: Optional[Path] = None

    def on_configure(
        self,
        topic: str = "/cleaning/path",
        frame_id: str = "map",
        qos_depth: int = 10,
    ) -> None:
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(Path, topic, qos)
        self._frame_id = frame_id
        self.get_logger().info(f"cleaning path publisher configured on {topic}")

    def publish_plan(self, xy: list[tuple[float, float]]) -> bool:
        if self._publisher is None:
            self.get_logger().error("publisher not configured; call on_configure first")
            return False
        if not xy:
            self.get_logger().warn("empty plan, skipping publish")
            return False
        stamp = self.get_clock().now().to_msg()
        path = to_ros_path(xy, frame_id=self._frame_id, stamp=stamp)
        self._publisher.publish(path)
        self._last_path = path
        return True

    def on_cleanup(self) -> None:
        if self._publisher is not None:
            self.destroy_publisher(self._publisher)
            self._publisher = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleaningPathPublisher()
    node.on_configure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
