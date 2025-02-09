import rclpy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, TransformStamped, Point
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

# set up mission states
MISSIONSTART = 0
ARMING = 1
MOVING = 2
DISARMING = 3
MISSIONCOMPLETE = 4

"""
Because we are using a PixHawk FCU to send signals to the ESC, we must go through initialization steps to arm the motors.
When initialized, this class goes through these steps.

This code is a modified version of the ROS2Control class from the OpenConvoy paper written by me (Owen Burns)"""

class FCUIntermediary(Node):
	velo_msg: TwistStamped = None

	def __init__(self):
		super().__init__('fcu_intermediary')

		# declare parameters
		self.declare_parameter('broadcast_interval', 0.1)
		broadcast_interval = self.get_parameter('broadcast_interval').value

		# pubsub setup
		self.velo_subscriber = self.create_subscription(Twist, '/f1tenth_racer/cmd_vel', self.cmd_vel_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.killswitch_subscriber = self.create_subscription(Bool, '/f1tenth_racer/killswitch', self.killswitch_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.vel_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 20)
		self.vehicle_pose_publisher = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 20)

		# transform listener
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
		self.transform_timer = self.create_timer(0.01, self.transform_callback)
		self.position_published_flag = False

        # arming and disarming services
		print('setting up motion clients')
		self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
		while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
			print('set mode service not available, waiting again...')
			
		self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
		while not self.arming_client.wait_for_service(timeout_sec=1.0):
			print('arming service not available, waiting again...')

		self.killswitch_client = self.create_client(CommandLong, '/mavros/cmd/command')
		while not self.killswitch_client.wait_for_service(timeout_sec=1.0):
			print('killswitch service not available, waiting again...')

		# initialize message to 0 so that if MOVING stage is reached before nav2 begins publishing, there is something to publish
		initial_vel_msg = Twist()
		initial_vel_msg.linear.x = 0.0
		initial_vel_msg.linear.y = 0.0
		initial_vel_msg.linear.z = 0.0
		initial_vel_msg.angular.x = 0.0
		initial_vel_msg.angular.y = 0.0
		initial_vel_msg.angular.z = 0.0

		self.vel_msg = initial_vel_msg

		self.mission_status = MISSIONSTART

		# Start main loop
		self.mission_timer = self.create_timer(broadcast_interval, self.mission_timer_callback)

	def transform_callback(self):
		"""Continuously publishes the vehicle's pose to the vehicle_pose topic."""

		try:
			transform_stamped: TransformStamped = self.tf_buffer.lookup_transform(
				'map',    # target frame
				'base_link',  # source frame
				rclpy.time.Time()
			)

			vehicle_pose = PoseStamped()
			vehicle_pose.header = Header()
			vehicle_pose.header.stamp = self.get_clock().now().to_msg()
			vehicle_pose.header.frame_id = 'map'
			vehicle_pose.pose.orientation = transform_stamped.transform.rotation
			vehicle_pose.pose.position = Point()
			vehicle_pose.pose.position.x = transform_stamped.transform.translation.x
			vehicle_pose.pose.position.y = transform_stamped.transform.translation.y
			vehicle_pose.pose.position.z = transform_stamped.transform.translation.z

			self.vehicle_pose_publisher.publish(vehicle_pose)
			self.position_published_flag = True # mark that we have gotten to a point where a position estimate is being published
		except Exception as e:
			self.get_logger().error(f"Error in transform callback: {e}")
			return

	def cmd_vel_callback(self, msg):
		"""Callback for the cmd_vel topic."""
		self.velo_msg = msg

	def killswitch_callback(self, msg):
		"""Callback for the killswitch topic."""
		if msg.data:
			self.disarm()

	def mission_timer_callback(self):
		"""Main loop for vehicle control. Handles the arming, moving, and disarming of the rover."""

		# start the chain of events that arms the rover
		if self.mission_status == MISSIONSTART:
			if not self.position_published_flag:
				self.get_logger().warning("No position estimate published yet. Waiting for position estimate...")
				return

			print("switching to offboard mode")
			set_mode_req = SetMode.Request()
			set_mode_req.base_mode = 0
			set_mode_req.custom_mode = "OFFBOARD"
			set_mode_future = self.set_mode_client.call_async(set_mode_req)
			set_mode_future.add_done_callback(self.init_callback)
			self.mission_status = ARMING
			return
		
		if self.mission_status == ARMING:
			# px4 requires a stream of setpoint messages to be sent to the rover in order to arm
			
			self.vel_publisher.publish(self.vel_msg)
			return
		
		if self.mission_status == MOVING:
			self.get_logger().debug(f"Published velocity message: {self.vel_msg}")
			self.vel_publisher.publish(self.vel_msg)
			return
		
		if self.mission_status == DISARMING:
			self.vel_msg.linear.x = 0.0
			self.vel_msg.linear.y = 0.0
			self.vel_msg.linear.z = 0.0
			self.vel_msg.angular.x = 0.0
			self.vel_msg.angular.y = 0.0
			self.vel_msg.angular.z = 0.0
			self.vel_publisher.publish(self.vel_msg)
			return
	
		if self.mission_status == MISSIONCOMPLETE:
			print("MISSION COMPLETE")
			self.get_logger().info("Car is shut down.")
			self.destroy_node()
			self.shutdown_flag = True
		
	def init_callback(self, future):
		print("...in offboard mode, arming")
		sleep(4) # wait for the stream of messages to be long enough to allow arming
		arm_req = CommandBool.Request()
		arm_req.value = True
		arm_future = self.arming_client.call_async(arm_req)
		arm_future.add_done_callback(self.arm_callback)

	def arm_callback(self, future):
		print("...armed, moving")
		self.start_time = time()
		self.mission_status = MOVING

	def disarm_callback(self, future):
		print("...disarmed")
		self.mission_status = MISSIONCOMPLETE

	def disarm(self):
		"""Disarm the vehicle"""

		disarm_req = CommandBool.Request()
		disarm_req.value = False
		disarm_future = self.arming_client.call_async(disarm_req)
		disarm_future.add_done_callback(self.disarm_callback)
		self.mission_status = DISARMING

def main(args=None):
    rclpy.init(args=args)
    fcu_intermediary = FCUIntermediary()
    
    # Add a flag to check for shutdown
    fcu_intermediary.shutdown_flag = False
    
    # Spin until shutdown is requested
    while rclpy.ok() and not fcu_intermediary.shutdown_flag:
        rclpy.spin_once(fcu_intermediary)
        
    # Continuously attempt to shut down the node until it succeeds
    while True:
        try:
            rclpy.shutdown()
            break
        finally:
            sleep(1)

if __name__ == '__main__':
    main() 