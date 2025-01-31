import math
import rclpy
import signal
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
import time
import sys
import tty
import termios
import math
import select

MISSIONSTART = 0
DRIVING = 1
ABORT = 2

# Store the original terminal settings
original_settings = None

def cleanup_terminal():
	"""Restore terminal settings."""
	global original_settings
	if original_settings:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)

def signal_handler(signum, frame):
	"""Handle interrupt signals."""
	cleanup_terminal()
	sys.exit(0)

class ROS2Control(Node):
	def __init__(self):
		super().__init__('keyboard_control')

		# pubsub setup
		self.scan_subscription = self.create_subscription(LaserScan, '/car_1/scan', self._telem_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.velocity_publisher = self.create_publisher(TwistStamped, '/ackermann_controller/reference', 20)
		
		self.scan = None
		self.mission_status = MISSIONSTART
		self.key_pressed = " "

		# setup keyboard input thread
		self.keyboard_thread = threading.Thread(target=self.__get_key)
		self.keyboard_thread.daemon = True
		self.keyboard_thread.start()

		# Timed components
		self.mission_timer = self.create_timer(0.1, self.__mission_timer_callback)

	def _telem_listener_callback(self, msg):
		self.scan = msg

	def __get_key(self):
		"""Listen for keyboard input in a separate thread."""
		global original_settings

		if not self.mission_status == ABORT:
			# Save terminal settings
			original_settings = termios.tcgetattr(sys.stdin)
			try:
				# Set terminal to raw mode
				tty.setraw(sys.stdin.fileno())
				while True:
					# Check if there's input available
					if select.select([sys.stdin], [], [], 0)[0]:
						key = sys.stdin.read(1)
						self.__process_key(key)
					sleep(0.1)
			finally:
				# Restore terminal settings
				cleanup_terminal()

	def __process_key(self, key):
		"""Process keyboard input and update velocities."""
		self.key_pressed = key

	def __mission_timer_callback(self):
		"""Main loop for vehicle control. Handles the publishing of control commands."""
		if self.mission_status == MISSIONSTART:
			self.mission_status = DRIVING
			return
		
		if self.mission_status == DRIVING:
			
			msg = TwistStamped()

			if self.key_pressed == 'w':
				msg.twist.linear.x = 2.0
				msg.twist.angular.z = 0.0
			elif self.key_pressed == 's':
				msg.twist.linear.x = -2.0
				msg.twist.angular.z = 0.0
			elif self.key_pressed == 'a':
				msg.twist.linear.x = 0.5
				msg.twist.angular.z = 0.7
			elif self.key_pressed == 'd':
				msg.twist.linear.x = 0.5
				msg.twist.angular.z = -0.7
			elif self.key_pressed == ' ':
				msg.twist.linear.x = 0.0
				msg.twist.angular.z = 0.0
			elif self.key_pressed == 'q':
				# ensure the vehicle stops
				msg.twist.linear.x = 0.0
				msg.twist.angular.z = 0.0
			
			msg.twist.linear.y = 0.0
			msg.twist.linear.z = 0.0
			msg.twist.angular.x = 0.0
			msg.twist.angular.y = 0.0

			ns = time.time_ns()
			seconds = ns // 1000000000
			nanoseconds = ns % 1000000000
			msg.header.stamp.sec = seconds
			msg.header.stamp.nanosec = nanoseconds
			msg.header.frame_id = 'base_link'
			self.velocity_publisher.publish(msg)

			if self.key_pressed == 'q':
				self.mission_status = ABORT

			return
	
		if self.mission_status == ABORT:
			print("MISSION COMPLETE")
			self.destroy_node()
			rclpy.shutdown()

if __name__ == '__main__':
	# Set up signal handlers
	signal.signal(signal.SIGINT, signal_handler)
	signal.signal(signal.SIGTERM, signal_handler)

	try:
		rclpy.init(args=None)
		control = ROS2Control()
		executor = MultiThreadedExecutor()
		executor.add_node(control)
		executor.spin()
	finally:
		cleanup_terminal()