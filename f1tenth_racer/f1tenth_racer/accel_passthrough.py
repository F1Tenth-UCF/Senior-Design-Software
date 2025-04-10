#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
import csv
import time
import os
import uuid

SPEED_TO_ERPM_GAIN = 4614.0
SPEED_TO_ERPM_OFFSET = 0.0

DEBUG_KEY = str(uuid.uuid4())

class TwistToAckermannNode(Node):
    """
    Node that converts Twist messages to AckermannDriveStamped messages.
    Subscribes to /key_vel (Twist) and publishes to /teleop (AckermannDriveStamped).
    """

    def __init__(self):
        super().__init__('twist_to_ackermann_node')
        
        # Create subscription to /key_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/key_vel',
            self.twist_callback,
            10
        )
        
        # Create publisher for /teleop topic
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/teleop',
            10
        )

        # create the log file
        debug_key = str(uuid.uuid4())
        self.log_path = f'/home/cavrel/f1tenth_ws/src/Senior-Design-Software/experiment_data/accel_times_closed/{debug_key}.csv'

        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['time', 'speed'])

        self.subscription = self.create_subscription(VescStateStamped, "/sensors/core", self.vesc_state_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())        
        
        self.get_logger().info('Twist to Ackermann node initialized')

    def vesc_state_callback(self, msg: VescStateStamped):
        with open(self.log_path, 'a') as f:

            # calculate the speed in m/s
            speed = -(-msg.state.speed - SPEED_TO_ERPM_OFFSET) / SPEED_TO_ERPM_GAIN

            writer = csv.writer(f)
            writer.writerow([time.time(), speed])

            if speed >= 6.49:
                self.get_logger().info('Target speed reached! Shutting down...')
                # Close the file properly
                f.close()
                # Kill the node and shutdown ROS
                self.destroy_node()
                rclpy.shutdown()

    def twist_callback(self, msg):
        """
        Callback function for the /key_vel subscription.
        Converts Twist messages to AckermannDriveStamped messages.
        
        Args:
            msg (Twist): The incoming Twist message
        """
        # Create AckermannDriveStamped message
        ackermann_msg = AckermannDriveStamped()
        
        # Set header timestamp to current time
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = "base_link"
        
        # Convert linear.x to speed and angular.z to steering angle
        ackermann_msg.drive.speed = msg.linear.x
        ackermann_msg.drive.steering_angle = msg.angular.z
        
        # Publish the converted message
        self.publisher.publish(ackermann_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermannNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
