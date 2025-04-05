import rclpy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, TransformStamped, Point
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import rclpy.time
from sensor_msgs.msg import NavSatFix, Imu
from mavros_msgs.msg import AttitudeTarget, PositionTarget
from std_msgs.msg import Float64, Bool, Header
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
import threading
import tf2_ros
import numpy as np
from scipy.spatial import cKDTree
from time import time, sleep
import csv
import os
import math
from datetime import datetime
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

"""
Because we are using a PixHawk FCU to send signals to the ESC, we must go through initialization steps to arm the motors.
When initialized, this class goes through these steps.

This code is a modified version of the ROS2Control class from the OpenConvoy paper written by me (Owen Burns)"""

EXPLORATION = 0
RACELINE = 1

class Nav2Intermediary(Node):

	def __init__(self):
		super().__init__('nav2_intermediary')

		# pubsub setup
		self.nav2_velo_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.gap_follow_subscriber = self.create_subscription(AckermannDriveStamped, '/gap_follow/cmd_vel', self.gap_follow_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.killswitch_subscriber = self.create_subscription(Bool, '/f1tenth_racer/killswitch', self.killswitch_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.pose_graph_subscriber = self.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization', self.pose_graph_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.vel_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 20)

		self.state = EXPLORATION

	def gap_follow_callback(self, msg: AckermannDriveStamped):
		"""Callback for the gap follow topic."""
		self.vel_publisher.publish(msg)

	def pose_graph_callback(self, msg: MarkerArray):
		"""Callback for the pose graph topic."""
		
		marker_array = np.zeros((len(msg.markers), 2))

		for marker in msg.markers:
			marker_array[marker.id-1, :] = [marker.pose.position.x, marker.pose.position.y]

		tree = cKDTree(marker_array)

		idxs = tree.query_ball_point(marker_array[0, :], 0.1)
		closures = []

		for i in idxs:
			if i <= 25:
				continue

			closures.append(i)

		if len(closures) > 0:
			self.destroy_node()
			self.shutdown_flag = True

	def cmd_vel_callback(self, msg: Twist):
		"""Callback for the cmd_vel topic from nav2."""
		
		vel_msg = AckermannDriveStamped()
		vel_msg.drive.steering_angle = msg.angular.z
		vel_msg.drive.speed = msg.linear.x
		
		# Check if this is the first non-zero velocity
		if not self.first_nonzero_published and vel_msg.drive.speed > 0:
			self.first_nonzero_published = True
			self.get_logger().info("First non-zero velocity published. Starting odometry error recording.")
            
		self.vel_publisher.publish(vel_msg)

	def killswitch_callback(self, msg):
		"""Callback for the killswitch topic."""
		if msg.data:
			self.get_logger().info("Car is shut down.")
			self.destroy_node()
			self.shutdown_flag = True

def main(args=None):
    rclpy.init(args=args)
    nav2_intermediary = Nav2Intermediary()
    
    # Add a flag to check for shutdown
    nav2_intermediary.shutdown_flag = False
    
    # Spin until shutdown is requested
    while rclpy.ok() and not nav2_intermediary.shutdown_flag:
        rclpy.spin_once(nav2_intermediary)
        
    # Continuously attempt to shut down the node until it succeeds
    while True:
        try:
            rclpy.shutdown()
            break
        finally:
            sleep(1)

if __name__ == '__main__':
    main()
