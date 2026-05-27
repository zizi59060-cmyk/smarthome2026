from __future__ import annotations

import rclpy
from rclpy.node import Node

from smarthome_common_interfaces.msg import ObjectTarget
from smarthome_vision.msg import DetectedTarget


class VisionDetectedTargetAdapter(Node):
    def __init__(self) -> None:
        super().__init__("vision_detected_target_adapter")
        self.declare_parameter("input_topic", "/detected_target")
        self.declare_parameter("output_target_topic", "/smarthome/object_target")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("publish_untracked", False)
        self.declare_parameter("label_prefix", "vision")

        self.pub = self.create_publisher(
            ObjectTarget,
            str(self.get_parameter("output_target_topic").value),
            10,
        )
        self.sub = self.create_subscription(
            DetectedTarget,
            str(self.get_parameter("input_topic").value),
            self.on_target,
            10,
        )

    def on_target(self, msg: DetectedTarget) -> None:
        publish_untracked = bool(self.get_parameter("publish_untracked").value)
        if not msg.tracking and not publish_untracked:
            return

        out = ObjectTarget()
        out.stamp = msg.stamp if msg.stamp.sec or msg.stamp.nanosec else self.get_clock().now().to_msg()
        out.class_id = int(msg.class_id)
        out.pose.header.stamp = out.stamp
        out.pose.header.frame_id = str(self.get_parameter("frame_id").value)
        out.pose.pose.position.x = float(msg.x)
        out.pose.pose.position.y = float(msg.y)
        out.pose.pose.position.z = float(msg.z)
        out.pose.pose.orientation.w = 1.0
        out.score = float(msg.score) if msg.tracking else 0.0
        out.source = ObjectTarget.SOURCE_QR if int(msg.mode) == 2 else ObjectTarget.SOURCE_OBJECT

        prefix = str(self.get_parameter("label_prefix").value)
        source = "qr" if out.source == ObjectTarget.SOURCE_QR else "object"
        out.label = f"{prefix}:{source}:{out.class_id}"
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionDetectedTargetAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
