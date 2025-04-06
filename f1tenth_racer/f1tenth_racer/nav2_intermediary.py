import rclpy
from scipy.spatial import cKDTree
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, TransformStamped, Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import rclpy.time
from sensor_msgs.msg import NavSatFix, Imu
from visualization_msgs.msg import MarkerArray
from mavros_msgs.msg import AttitudeTarget, PositionTarget
from std_msgs.msg import Float64, Bool, Header
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
import threading
import tf2_ros
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

class Nav2Intermediary(Node):

	def __init__(self):
		super().__init__('nav2_intermediary')

		# pubsub setup
		self.velo_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.wall_follow_subscriber = self.create_subscription(AckermannDriveStamped, '/wall_follower/cmd_vel', self.wall_follow_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.killswitch_subscriber = self.create_subscription(Bool, '/f1tenth_racer/killswitch', self.killswitch_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.pose_graph_subscriber = self.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization', self.pose_graph_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.vel_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 20)

		# TF2 setup for transform listening
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		
		# CSV file setup
		self.csv_file_path = "/home/cavrel/f1tenth_ws/src/Senior-Design-Software/odometry_errors.csv"
		self.csv_file = None
		self.csv_writer = None
		self.initialize_csv_file()
		
		# Flag to track if we've published non-zero velocity
		self.first_nonzero_published = False
		
		# Timer for recording odometry errors (runs at 10Hz)
		self.timer = self.create_timer(0.1, self.record_odometry_error)

	def pose_graph_callback(self, msg: MarkerArray):
		"""During the exploraiton phase, checks for loop closure. During raceline computation and execution, does nothing."""

		# order the markers by their id, ensuring they're in the order they were added to the graph
		marker_array = np.zeros((len(msg.markers), 2))
		for marker in msg.markers:
			marker_array[marker.id-1, :] = [marker.pose.position.x, marker.pose.position.y]

		# find the closest points to the original scan
		tree = cKDTree(marker_array)
		idxs = tree.query_ball_point(marker_array[0, :], 0.1)
		closures = []

		# determine if any of those closest points are more than 25 scans later. This would indicate that the car has looped back around to the starting point
		for i in idxs:
			if i <= 25:
				continue

			closures.append(i)

		self.get_logger().info(f"Loop closure detected at {len(closures)} points.")

	def wall_follow_callback(self, msg: AckermannDriveStamped):
		"""Callback for the wall follower topic."""
		self.vel_publisher.publish(msg)

	def initialize_csv_file(self):
		"""Initialize the CSV file with headers."""
		file_exists = os.path.isfile(self.csv_file_path)
		
		self.csv_file = open(self.csv_file_path, 'a')
		self.csv_writer = csv.writer(self.csv_file)
		
		if not file_exists:
			self.csv_writer.writerow(['timestamp', 'distance_cm'])
			self.csv_file.flush()

	def cmd_vel_callback(self, msg: Twist):
		"""Callback for the cmd_vel topic."""
		
		vel_msg = AckermannDriveStamped()
		vel_msg.drive.steering_angle = msg.angular.z
		vel_msg.drive.speed = msg.linear.x
		
		# Check if this is the first non-zero velocity
		if not self.first_nonzero_published and vel_msg.drive.speed > 0:
			self.first_nonzero_published = True
			self.get_logger().info("First non-zero velocity published. Starting odometry error recording.")
            
		self.vel_publisher.publish(vel_msg)

	def record_odometry_error(self):
		"""Record the distance between map and odom frames."""
		if not self.first_nonzero_published:
			return
			
		try:
			# Look up the transform between map and odom
			transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
			
			# Calculate distance in centimeters
			x = transform.transform.translation.x
			y = transform.transform.translation.y
			z = transform.transform.translation.z
			distance_cm = math.sqrt(x*x + y*y + z*z) * 100.0  # Convert meters to centimeters
			
			# Get current timestamp
			timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
			
			# Write to CSV
			self.csv_writer.writerow([timestamp, f"{distance_cm:.2f}"])
			self.csv_file.flush()
			
		except Exception as e:
			self.get_logger().warning(f"Failed to get transform between map and odom: {e}")

	def killswitch_callback(self, msg):
		"""Callback for the killswitch topic."""
		if msg.data:
			self.get_logger().info("Car is shut down.")
			if self.csv_file:
				self.csv_file.close()
			self.destroy_node()
			self.shutdown_flag = True

	def __del__(self):
		if self.csv_file:
			self.csv_file.close()

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