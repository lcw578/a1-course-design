#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GazeboGroundTruthOdom(Node):
    def __init__(self):
        super().__init__("gazebo_ground_truth_odom")

        self.declare_parameter("source_odom_topic", "/ground_truth/odom_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("zero_z", False)

        source_odom_topic = self.get_parameter("source_odom_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.zero_z = bool(self.get_parameter("zero_z").value)

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Odometry, source_odom_topic, self._odom_callback, 10)

        self.get_logger().info(
            f"Republishing Gazebo ground-truth odom from '{source_odom_topic}' "
            f"to '{odom_topic}' with TF {self.odom_frame} -> {self.base_frame}"
        )

    def _odom_callback(self, msg):
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose = msg.pose
        odom.twist = msg.twist

        if self.zero_z:
            odom.pose.pose.position.z = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0

        # Ground truth has no estimator covariance; keep a small nonzero diagonal
        # so consumers that inspect covariance do not see an unknown estimate.
        odom.pose.covariance[0] = 1e-4
        odom.pose.covariance[7] = 1e-4
        odom.pose.covariance[14] = 1e-4
        odom.pose.covariance[21] = 1e-4
        odom.pose.covariance[28] = 1e-4
        odom.pose.covariance[35] = 1e-4
        odom.twist.covariance[0] = 1e-4
        odom.twist.covariance[7] = 1e-4
        odom.twist.covariance[14] = 1e-4
        odom.twist.covariance[21] = 1e-4
        odom.twist.covariance[28] = 1e-4
        odom.twist.covariance[35] = 1e-4

        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = odom.pose.pose.position.x
            tf_msg.transform.translation.y = odom.pose.pose.position.y
            tf_msg.transform.translation.z = odom.pose.pose.position.z
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboGroundTruthOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
