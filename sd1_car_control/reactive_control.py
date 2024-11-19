import math
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.executors import MultiThreadedExecutor
import threading
from time import time, sleep

MISSIONSTART = 0
DRIVING = 1
ABORT = 2

class ROS2Control(Node):
	def __init__(self):
		super().__init__('udp_publisher')

		# pubsub setup
		self.scan_subscription = self.create_subscription(LaserScan, '/car_1/scan', self._telem_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.velocity_publisher = self.create_publisher(TwistStamped, '/ackermann_controller/reference', 20)
		
		self.scan = None
		self.mission_status = MISSIONSTART

		# setup kill switch
		self.stop_thread = threading.Thread(target=self.__listen_for_stop)
		self.stop_thread.daemon = True
		self.stop_thread.start()

		# Timed components
		self.mission_timer = self.create_timer(0.1, self.__mission_timer_callback)

	def _telem_listener_callback(self, msg):
		self.scan = msg

	def __listen_for_stop(self):
		"""Kills the mission if the user presses ENTER."""

		while True:
			input()
			self._disarm()
	
	def __get_wall_distance(self, scan, side):
		"""Calculate wall distance for the specified side."""
		# Assuming 360-degree laser scan
		ranges = scan.ranges
		
		if side == 'left':
			# Left side ranges (first 90 degrees)
			wall_ranges = ranges[:len(ranges)//4]
		else:  # right side
			# Right side ranges (last 90 degrees)
			wall_ranges = ranges[3*len(ranges)//4:]
		
		# Remove inf and nan values, then take minimum
		valid_ranges = [r for r in wall_ranges if r != float('inf') and not math.isnan(r)]
		
		return min(valid_ranges) if valid_ranges else float('inf')
	


	def __mission_timer_callback(self):
		"""Main loop for vehicle control. Handles the arming, moving, and disarming of the rover."""

		# start the chain of events that arms the rover
		if self.mission_status == MISSIONSTART:
            # ensure that the laser scan is being published
			if self.scan is not None:
				self.mission_status = DRIVING
			return
		
		if self.mission_status == DRIVING:
			# px4 requires a stream of setpoint messages to be sent to the rover in order to arm
			# Wall following logic
			left_wall = self.__get_wall_distance(self.scan, 'left')
			right_wall = self.__get_wall_distance(self.scan, 'right')
			
			# Proportional control for steering
			steering_error = left_wall - right_wall
			kp = 0.5  # Proportional gain for steering
			
			# Publish velocity message
			msg = TwistStamped()
			msg.twist.linear.x = 1.0  # Constant forward velocity of 1 m/s
			msg.twist.angular.z = -kp * steering_error  # Negative to correct heading
			
			print(f"(publishing) Left: {left_wall}, Right: {right_wall}, Error: {steering_error}")
			self.velocity_publisher.publish(msg)
			return
	
		if self.mission_status == ABORT:
			print("MISSION COMPLETE")
			rclpy.shutdown()

	def _disarm(self):
		"""Disarm the vehicle"""

		self.mission_status = ABORT

rclpy.init(args=None)
control = ROS2Control()
executor = MultiThreadedExecutor()
executor.add_node(control)
executor.spin()