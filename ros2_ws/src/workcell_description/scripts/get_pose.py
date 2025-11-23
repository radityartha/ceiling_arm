#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros  # <-- NEW IMPORT
from tf2_ros import TransformException  # <-- NEW IMPORT


class PoseListenerNode(Node):
    def __init__(self):
        super().__init__("pose_listener_node")

        # --- 1. Initialize the TF2 Buffer and Listener ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to check for the transform
        self.timer = self.create_timer(1.0, self.get_pose)

    def get_pose(self):
        from_frame = "world"
        to_frame = "t1_a1_end_effector_link"  # The link you want to find

        try:
            # --- 2. Look up the transform ---
            t = self.tf_buffer.lookup_transform(
                from_frame, to_frame, rclpy.time.Time()
            )  # Get the latest available transform

            self.get_logger().info(f"--- Current Pose for {to_frame} ---")
            self.get_logger().info(
                f"Position (x,y,z): {t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z}"
            )
            self.get_logger().info(
                f"Orientation (x,y,z,w): {t.transform.rotation.x}, {t.transform.rotation.y}, {t.transform.rotation.z}, {t.transform.rotation.w}"
            )

        except TransformException as ex:
            self.get_logger().warn(f"Could not get transform: {ex}")


def main():
    rclpy.init()
    node = PoseListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
