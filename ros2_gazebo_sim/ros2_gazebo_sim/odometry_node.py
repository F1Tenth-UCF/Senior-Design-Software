#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

# For time synchronization of the two encoder topics:
from message_filters import ApproximateTimeSynchronizer, Subscriber

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Store latest position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Publisher: Odometry
        self.odom_broadcaster = TransformBroadcaster(self)
        
        # Subscribers: we assume each JointState has velocity[0] in rad/s for that wheel
        self.ips_sub = Subscriber(self, Point, '/autodrive/f1tenth_1/ips')
        self.imu_sub = Subscriber(self, Imu, '/autodrive/f1tenth_1/imu')

        # Synchronize messages that arrive within 0.01 seconds of each other
        self.ts = ApproximateTimeSynchronizer(
            [self.ips_sub, self.imu_sub],
            queue_size=10,
            slop=0.01,
            allow_headerless=True
        )
        self.ts.registerCallback(self.callback)

        # Create timer to publish transform at 50Hz
        self.create_timer(0.02, self.publish_transform)

    def callback(self, ips_msg, imu_msg):
        """
        Callback for synchronized ips and imu messages.
        Updates the stored position and orientation.
        """
        self.current_x = ips_msg.x
        self.current_y = ips_msg.y
        self.current_yaw = imu_msg.orientation.z

    def publish_transform(self):
        """
        Publishes the transform using the most recent position and orientation.
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'       # The reference frame
        transform.child_frame_id = 'f1tenth_1'   # Robot's (moving) frame
        
        # Position
        transform.transform.translation.x = self.current_x
        transform.transform.translation.y = self.current_y
        transform.transform.translation.z = 0.0

        # Orientation
        q = quaternion_from_euler(0, 0, self.current_yaw)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        # Publish the transform
        self.odom_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
