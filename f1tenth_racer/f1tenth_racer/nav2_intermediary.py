import rclpy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, TransformStamped, Point
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
from time import time, sleep

"""
Because we are using a PixHawk FCU to send signals to the ESC, we must go through initialization steps to arm the motors.
When initialized, this class goes through these steps.

This code is a modified version of the ROS2Control class from the OpenConvoy paper written by me (Owen Burns)"""

class Nav2Intermediary(Node):

	def __init__(self):
		super().__init__('nav2_intermediary')

		# pubsub setup
		self.velo_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.killswitch_subscriber = self.create_subscription(Bool, '/f1tenth_racer/killswitch', self.killswitch_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.vel_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 20)

	def cmd_vel_callback(self, msg: Twist):
		"""Callback for the cmd_vel topic."""
		
		vel_msg = AckermannDriveStamped()
		vel_msg.drive.steering_angle = msg.angular.z
            
		# hack to get around nav2 giving a very low velocity to the car despite configuration settings telling it otherwise. Will fix later
		if msg.linear.x < 0.5:
			vel_msg.drive.speed = 0.5
		else:
			vel_msg.drive.speed = msg.linear.x
            
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
