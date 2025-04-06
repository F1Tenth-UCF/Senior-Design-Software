import rclpy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, TransformStamped, Point
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, OccupancyGrid
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
import cv2
from scipy.ndimage import binary_closing, binary_opening
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.spatial import cKDTree
from scipy.interpolate import CubicSpline
from skimage.morphology import medial_axis, skeletonize
from skimage.morphology import flood_fill
import networkx as nx
import sknw
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon
import pandas as pd
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
COMPUTING_RACELINE = 1
RACELINE = 2

OPTIMIZER_PATH = '/home/cavrel/f1tenth_ws/src/global_racetrajectory_optimization'

def preprocess_track(map: OccupancyGrid):
	"""Converts the occupancy grid into a centerline and track width data."""

	map_resolution = map.info.resolution

	data = np.array(map.data)
	data_reshaped = data.reshape(map.info.height, map.info.width)

	masked_data = np.ma.masked_where(data_reshaped == -1, data_reshaped)
	masked_free_space = np.ma.masked_where(masked_data>40, masked_data)
	darkened_free_space = np.where(masked_free_space==0.0, 255, 0).astype(np.uint8)

	if len(darkened_free_space.shape) == 3:  # Check if it's a color image
		gray_space = cv2.cvtColor(darkened_free_space, cv2.COLOR_BGR2GRAY)
	else:
		gray_space = darkened_free_space  # Already grayscale

	# Convert to binary where 255 becomes 1
	binary_space = np.zeros((gray_space.shape[0], gray_space.shape[1]), dtype=np.uint8)
	binary_space[gray_space > 127] = 1

	# eliminate holes and noise
	track = binary_opening(binary_space, iterations=2)
	track = binary_closing(track, iterations=2)

	# get a 1 pixel wide skeleton of the centerline
	# D = distance_transform_edt(track)
	skel = skeletonize(track)

	# convert that to a path graph
	G = sknw.build_sknw(skel)
	path = []
	for (s, e) in G.edges():
		path.extend(G[s][e]['pts'])
	xy = np.array(path)*map_resolution

	# Smooth the path
	seg_len = np.linalg.norm(np.diff(xy, axis=0), axis=1)
	s = np.hstack(([0], np.cumsum(seg_len)))
	csx = CubicSpline(s, xy[:,0])
	csy = CubicSpline(s, xy[:,1])
	s_u = np.linspace(0, s[-1], 1000)
	centerline = np.c_[csx(s_u), csy(s_u)]

	# get the distance on both sides of the centerline
	# use the edt thing to get the distance to each wall by masking the inside and outside of the track
	poly = Polygon(centerline/map_resolution)
	centroid = poly.centroid

	outer = flood_fill(track, (int(centroid.x), int(centroid.y)), 1)
	inner = flood_fill(track, (0,0), 1)

	outer_distance = distance_transform_edt(outer)
	inner_distance = distance_transform_edt(inner)

	right_width = []
	left_width = []

	# moving clockwise around the track right will always be inner and left will always be outer
	for i in range(len(centerline)):
		scaled_point = centerline[i]/map_resolution
		right_width.append(inner_distance[int(scaled_point[0]), int(scaled_point[1])]*map_resolution)
		left_width.append(outer_distance[int(scaled_point[0]), int(scaled_point[1])]*map_resolution)

	return centerline, right_width, left_width

class Nav2Intermediary(Node):

	def __init__(self):
		super().__init__('nav2_intermediary')

		# pubsub setup
		self.nav2_velo_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.gap_follow_subscriber = self.create_subscription(AckermannDriveStamped, '/gap_follow/cmd_vel', self.gap_follow_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		
		self.killswitch_subscriber = self.create_subscription(Bool, '/f1tenth_racer/killswitch', self.killswitch_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.pose_graph_subscriber = self.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization', self.pose_graph_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.vel_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 20)

		self.state = EXPLORATION
		self.map: OccupancyGrid = None

		self.raceline_computation_thread = threading.Thread(target=self.compute_raceline)

	def compute_raceline(self):
		"""Runs in its own thread and computes the raceline, then kicks off the raceline execution stage of the race."""

		# preprocess the occupancy grid
		centerline, right_width, left_width = preprocess_track(self.map)

		self.get_logger().info("Track preprocessed. Writing to csv.")

		# write the track data to a csv file
		with open(f'hec_track.csv', 'w', newline='') as csvfile: #{OPTIMIZER_PATH}/inputs/tracks/
			writer = csv.writer(csvfile, delimiter=',')
			writer.writerow(['# x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m'])
			for i in range(len(centerline)):
				writer.writerow([
					f"{centerline[i][0]:.2f}",  # x_m
					f"{centerline[i][1]:.2f}",  # y_m
					f"{right_width[i]:.2f}",    # w_tr_right_m
					f"{left_width[i]:.2f}"      # w_tr_left_m
				])

		self.get_logger().info("Track data written to csv. Running optimization.")

		# # run the optimization
		# os.system(f'python3 {OPTIMIZER_PATH}/main_globaltraj.py')
		# sleep(5)

		# # read the output file
		# if os.path.exists(f'{OPTIMIZER_PATH}/outputs/traj_race_cl.csv'):
		# 	df = pd.read_csv(f'{OPTIMIZER_PATH}/outputs/traj_race_cl.csv')
		# 	df = df.drop(columns=['Unnamed: 0'])
		# 	df.to_csv(f'{OPTIMIZER_PATH}/outputs/traj_race_cl.csv', index=False)
		# 	self.get_logger().info("Raceline file found. Switching to raceline execution.")
		# else:
			# self.get_logger().error("Raceline file not found.")
			# self.destroy_node()
			# self.shutdown_flag = True

		self.destroy_node()
		self.shutdown_flag = True

	def gap_follow_callback(self, msg: AckermannDriveStamped):
		"""During the exploration and raceline computation phase, passes the gap follow command to the vel_publisher. After the race line is computed, this function does nothing."""

		if self.state < RACELINE:
			self.vel_publisher.publish(msg)

	def cmd_vel_callback(self, msg: Twist):
		"""During the raceline phase, passes the nav2 command to the vel_publisher."""
		
		if self.state == RACELINE:
			vel_msg = AckermannDriveStamped()
			vel_msg.drive.steering_angle = msg.angular.z
			vel_msg.drive.speed = msg.linear.x
			
			# Check if this is the first non-zero velocity
			if not self.first_nonzero_published and vel_msg.drive.speed > 0:
				self.first_nonzero_published = True
				self.get_logger().info("First non-zero velocity published. Starting odometry error recording.")
				
			self.vel_publisher.publish(vel_msg)

	def map_callback(self, msg: OccupancyGrid):
		"""Stores the latest map"""
		self.map = msg

	def pose_graph_callback(self, msg: MarkerArray):
		"""During the exploraiton phase, checks for loop closure. During raceline computation and execution, does nothing."""

		if self.state == EXPLORATION:
			# order the markers by their id, ensuring they're in the order they were added to the graph
			marker_array = np.zeros((len(msg.markers), 2))
			for marker in msg.markers:
				marker_array[marker.id-1, :] = [marker.pose.position.x, marker.pose.position.y]

			# save the marker array as an image using matplotlib
			plt.figure(figsize=(10, 10))
			plt.scatter(marker_array[:, 0], marker_array[:, 1], s=10, c='blue')
			plt.savefig(f'/home/cavrel/f1tenth_ws/src/Senior-Design-Software/pose_graph_images/marker_array_{self.get_clock().now().to_msg().sec}.png')
			plt.close()

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

			# if there are loop closures, we change the mode to raceline computation and start the thread. This thread will run asynchronously to compute the raceline, and switch off exploration mode when complete.
			if len(closures) > 0:
				self.get_logger().info("Loop closure detected. Switching to raceline computation.")
				self.state = COMPUTING_RACELINE
				self.raceline_computation_thread.start()

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