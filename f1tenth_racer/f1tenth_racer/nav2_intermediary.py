import rclpy
from geometry_msgs.msg import Twist, Quaternion, PoseStamped, PoseWithCovarianceStamped, TwistStamped, PoseArray
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy.time
from std_msgs.msg import Bool
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from tf2_ros import Buffer, TransformListener
import threading
import matplotlib.pyplot as plt
import cv2
from scipy.ndimage import binary_closing, binary_opening, distance_transform_edt
from scipy.spatial import cKDTree
from scipy.interpolate import CubicSpline
from skimage.morphology import skeletonize
from skimage.morphology import flood_fill
import sknw
import numpy as np
import networkx as nx
from shapely.geometry import Polygon
import numpy as np
from scipy.spatial import cKDTree
from time import sleep
import csv
import os
import time
import subprocess

"""
Because we are using a PixHawk FCU to send signals to the ESC, we must go through initialization steps to arm the motors.
When initialized, this class goes through these steps.

This code is a modified version of the ROS2Control class from the OpenConvoy paper written by me (Owen Burns)"""

EXPLORATION = 0
COMPUTING_RACELINE = 1
RACELINE = 2

LOG_EXPERIMENTS = True

OPTIMIZER_PATH = 'global_racetrajectory_optimization'
DEBUG_KEY = str(time.time())
os.makedirs(f"src/Senior-Design-Software/debug_data/{DEBUG_KEY}/pose_graph_images", exist_ok=True)

def display_binary_image(image, title):
	plt.figure(figsize=(10, 8))
	plt.imshow(image, cmap='gray')
	plt.savefig(f'src/Senior-Design-Software/debug_data/{DEBUG_KEY}/{title}.png')
	plt.close()

def plot_graph(G, title):
    plt.figure(figsize=(10, 8))

    # Plot the graph nodes
    node_pos = {i: (G.nodes[i]['o'][1], G.nodes[i]['o'][0]) for i in G.nodes()}
    nx.draw_networkx_nodes(G, pos=node_pos, node_size=20, node_color='red')

    # Plot the graph edges
    for (s, e) in G.edges():
        ps = G[s][e]['pts']
        plt.plot(ps[:, 1], ps[:, 0], 'green')
    plt.title('Skeleton Graph Visualization')
    plt.axis('equal')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f'src/Senior-Design-Software/debug_data/{DEBUG_KEY}/{title}.png')
    plt.close()

# TODO: fix issue where graph created by sknw is has no cycles. doesn't happen much anymore since i modified the slam params to filter out far away lidar points.
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

	### DEBUGGING CODE ###
	np.save(f'src/Senior-Design-Software/debug_data/{DEBUG_KEY}/track_raw.npy', binary_space)
	### DEBUGGING CODE ###

	# eliminate holes and noise
	track = binary_opening(binary_space, iterations=2)
	track = binary_closing(track, iterations=2)

	display_binary_image(track, 'track')

	# get a 1 pixel wide skeleton of the centerline
	skel = skeletonize(track)

	display_binary_image(skel, 'skel')

	# convert that to a path graph. We also need to check for spurious edges that are not part of the track, and remove them.
	G = sknw.build_sknw(skel)
	plot_graph(G, 'graph')

	leaf_nodes_exist = True
	while leaf_nodes_exist: # make repeated iterations through the graph until there are either no leaf nodes or only 1 node left
		nodes_to_remove = []

		for node in G.nodes():
			if G.degree[node] == 1 and len(nodes_to_remove) < len(G.nodes()) - 1:
				nodes_to_remove.append(node)

		if len(nodes_to_remove) == 0:
			leaf_nodes_exist = False

		for node in nodes_to_remove:
			G.remove_node(node)

	plot_graph(G, 'graph_no_spurs')

	# extract the ordered points from the graph
	path = []
	for (s, e) in G.edges():
		path.extend(G[s][e]['pts'])
	xy = np.array(path)*map_resolution

	# Smooth the path
	seg_len = np.linalg.norm(np.diff(xy, axis=0), axis=1)
	s = np.hstack(([0], np.cumsum(seg_len)))

	# Identify indices where consecutive distances are zero
	s_diff = np.diff(s)
	nic = np.where(s_diff == 0)[0]

	# Shift overlapping values slightly forward to maintain strict monotonicity
	# Create a copy to avoid modifying the original s array
	s_fixed = s.copy()

	for i in nic:
		# i is the index where s[i+1] == s[i]
		# Find next increasing index j > i+1
		j = i + 2
		while j < len(s_fixed) and s_fixed[j] == s_fixed[i]:
			j += 1
		if j < len(s_fixed):
			midpoint = (s_fixed[j] - s_fixed[i]) / 2
			s_fixed[i+1] = s_fixed[i] + midpoint
		else:
			# If all following values are the same, just bump it slightly
			s_fixed[i+1] = s_fixed[i] + 1e-6

	csx = CubicSpline(s_fixed, xy[:,0])
	csy = CubicSpline(s_fixed, xy[:,1])
	s_u = np.linspace(0, s_fixed[-1], 1000)
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

	centroid_meters = np.array([centroid.y, centroid.x])*map_resolution + np.array([map.info.origin.position.x, map.info.origin.position.y])

	### DEBUGGING CODE ###
	plt.figure(figsize=(10, 8))
	plt.plot(centerline[:, 0], centerline[:, 1], 'b-', linewidth=2, label='Centerline')
	plt.scatter(xy[:, 0], xy[:, 1], c='r', s=10, alpha=0.5, label='Original Points')
	plt.title('Track Centerline')
	plt.xlabel('X (meters)')
	plt.ylabel('Y (meters)')
	plt.legend()
	plt.axis('equal')
	plt.grid(True)

	for i in range(len(centerline[:-1])):
		_range = centerline[i:i+2]

		p1x, p1y = _range[0]
		p2x, p2y = _range[1]

		right_vector = np.array((p2y - p1y, p1x - p2x))
		left_vector = np.array((p1y - p2y, p2x - p1x))

		right_vector = (right_vector / np.linalg.norm(right_vector)) * right_width[i]
		left_vector = (left_vector / np.linalg.norm(left_vector)) * left_width[i]

		plt.arrow(p1x, p1y, right_vector[0], right_vector[1], head_width=0.1, head_length=0.1, color='red')
		plt.arrow(p1x, p1y, left_vector[0], left_vector[1], head_width=0.1, head_length=0.1, color='blue')

	plt.savefig(f'src/Senior-Design-Software/debug_data/{DEBUG_KEY}/centerline_debug.png')
	plt.close()
	### DEBUGGING CODE ###
	
	return centerline, right_width, left_width, centroid_meters

class Nav2Intermediary(Node):

	def __init__(self):
		super().__init__('nav2_intermediary')

		# class variables
		self.state = EXPLORATION
		self.map: OccupancyGrid = None
		self.raceline: Path = None

		# processes used for controlling the car
		self.nav2_velo_subscriber = self.create_subscription(Twist, '/cmd_vel', self.nav2_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.gap_follow_subscriber = self.create_subscription(AckermannDriveStamped, '/wall_follower/cmd_vel', self.gap_follow_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.updated_raceline_publisher = self.create_timer(1.0, self.publish_updated_raceline)
		self.vel_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 20)
		self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 20)
		# self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
		# self.get_logger().info("Waiting for FollowPath action server...")
		# self.follow_path_client.wait_for_server()
		# self.get_logger().info("FollowPath action server is up!")
		
		# topics on which we receive data to store at a class level for use by other processes within this node
		self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

		# processes used for progressing the state of the race
		self.pose_graph_subscriber = self.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization', self.pose_graph_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.raceline_computation_thread = threading.Thread(target=self.compute_raceline)

		# misc processes
		self.killswitch_subscriber = self.create_subscription(Bool, '/f1tenth_racer/killswitch', self.killswitch_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.raceline_publisher = self.create_publisher(Path, '/raceline', 20)
		self.raceline_publisher_thread = threading.Thread(target=self.publish_raceline)
		self.pose_array_publisher = self.create_publisher(PoseArray, '/raceline_poses', 20)

		# set up logging for experiments
		if LOG_EXPERIMENTS:
			self.set_up_csvs()
			self.last_pose_publish_time = None
			self.plan_subscriber = self.create_subscription(Path, '/plan', self.plan_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		
	# VVV Car control functions VVV

	def gap_follow_callback(self, msg: AckermannDriveStamped):
		"""During the exploration and raceline computation phase, passes the gap follow command to the vel_publisher. After the race line is computed, this function does nothing."""

		if self.state < RACELINE:
			self.vel_publisher.publish(msg)

	def nav2_callback(self, msg: Twist):
		"""During the raceline phase, translates the nav2 command to the AckermannDriveStamped message that the VESC expects and publishes it."""
		
		if self.state == RACELINE:

			self.get_logger().info(f"Nav2 command received: {msg.linear.x}, {msg.angular.z}")

			vel_msg = AckermannDriveStamped()
			vel_msg.drive.steering_angle = msg.angular.z
			vel_msg.drive.speed = msg.linear.x
				
			self.vel_publisher.publish(vel_msg)

	def publish_updated_raceline(self):
		"""Publishes the updated raceline to the /raceline topic."""

		if self.state == RACELINE:

			# get the current position of the car in the map frame
			transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
			car_pose = np.array([transform.transform.translation.x, transform.transform.translation.y])

			# find the index of the raceline closest to the car
			poses = np.array([[p.pose.position.x, p.pose.position.y] for p in self.raceline.poses])
			distances = np.linalg.norm(poses - car_pose, axis=1)
			closest_index = np.argmin(distances)

			# get the next goal pose as 25% of the way around the current lap.
			# TODO: Fix this so that it goes 25% forward instead of backwards.
			reindexed_poses = self.raceline.poses[:closest_index] + self.raceline.poses[closest_index:]
			next_goal_pose = reindexed_poses[int(0.1*len(reindexed_poses))]

			# publish the next goal pose
			goal_pose_msg = PoseStamped()
			goal_pose_msg.header.frame_id = 'map'
			goal_pose_msg.pose = next_goal_pose.pose
			self.goal_pose_publisher.publish(goal_pose_msg)

			# # create a new path message which is a full loop starting from the closest point on the raceline to the car
			# sliced_raceline_msg = Path()
			# sliced_raceline_msg.header.frame_id = 'map'
			# sliced_raceline_msg.poses = self.raceline.poses[closest_index:] #+ self.raceline.poses[:closest_index]

			# # create a follow path goal
			# goal = FollowPath.Goal()
			# goal.path = sliced_raceline_msg
			# goal.controller_id = 'FollowPath'

			# # send the goal to the action server, and don't wait for a result
			# self.follow_path_client.send_goal_async(goal)

			if LOG_EXPERIMENTS:
				self.last_pose_publish_time = time.time()

	# VVV Sensor callback functions VVV

	def map_callback(self, msg: OccupancyGrid):
		"""Stores the latest map. After the raceline computation begins, this function does nothing."""

		if self.state == EXPLORATION: # this ensures that the map isn't updates while the raceline is being computed
			self.map = msg

	# VVV State Management Functions VVV

	# TODO: if our loop closure screws up, we need to reset the pose graph.
	def pose_graph_callback(self, msg: MarkerArray):
		"""During the exploraiton phase, checks for loop closure. During raceline computation and execution, does nothing."""

		if self.state == EXPLORATION:
			# order the markers by their id, ensuring they're in the order they were added to the graph
			marker_array = np.zeros((len(msg.markers), 2))
			for marker in msg.markers:
				marker_array[marker.id-1, :] = [marker.pose.position.x, marker.pose.position.y]

			### DEBUGGING CODE ###
			plt.figure(figsize=(10, 10))
			# Create a colormap that shows the order of markers (from blue to red)
			colors = plt.cm.viridis(np.linspace(0, 1, len(marker_array)))
			plt.scatter(marker_array[:, 0], marker_array[:, 1], s=10, c=colors)
			plt.colorbar(label='Marker Order')
			plt.title(f'Pose Graph Markers - {len(marker_array)} points')
			plt.xlabel('X Position (m)')
			plt.ylabel('Y Position (m)')
			plt.savefig(f'src/Senior-Design-Software/debug_data/{DEBUG_KEY}/pose_graph_images/marker_array_{self.get_clock().now().to_msg().sec}.png')
			plt.close()
			### DEBUGGING CODE ###

			tree = cKDTree(marker_array)

			# for each marker, we find the indices of the markers within 0.1m. If any of these markers is more than 10 scans later, we have a loop closure.
			closures = []
			for i in range(len(marker_array)):
				idxs = tree.query_ball_point(marker_array[i, :], 0.1)
				
				for j in idxs:
					if j <= i + 10:
						continue

					closures.append(j)

			self.get_logger().info(f"Loop closure detected at {len(closures)} points.")

			# if there are loop closures, we change the mode to raceline computation and start the thread. This thread will run asynchronously to compute the raceline, and switch off exploration mode when complete.
			if len(closures) > 0:
				self.get_logger().info("Loop closure detected. Switching to raceline computation.")
				self.state = COMPUTING_RACELINE
				self.raceline_computation_thread.start()

	def compute_raceline(self):
		"""Runs in its own thread and computes the raceline, then kicks off the raceline execution stage of the race."""

		# preprocess the occupancy grid
		centerline, right_width, left_width, track_centroid = preprocess_track(self.map)

		self.get_logger().info("Track preprocessed. Writing to csv.")

		# write the track data to a csv file
		with open(f'{OPTIMIZER_PATH}/inputs/tracks/hec_track.csv', 'w', newline='') as csvfile:
			writer = csv.writer(csvfile, delimiter=',')
			writer.writerow(['# x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m'])
			for i in range(len(centerline)):
				writer.writerow([
					f"{centerline[i][0]:.2f}",  # x_m
					f"{centerline[i][1]:.2f}",  # y_m
					f"{right_width[i]:.2f}",    # w_tr_right_m
					f"{left_width[i]:.2f}"      # w_tr_left_m
				])

		### DEBUGGING CODE ###
		os.system(f"cp {OPTIMIZER_PATH}/inputs/tracks/hec_track.csv src/Senior-Design-Software/debug_data/{DEBUG_KEY}/hec_track.csv")
		### DEBUGGING CODE ###

		self.get_logger().info("Track data written to csv. Running optimization.")

		# run the optimization
		cmd = [
			"python3",
			f"{OPTIMIZER_PATH}/main_globaltraj.py",
			"--x",
			str(-1 * self.map.info.origin.position.x),
			"--y",
			str(-1 * self.map.info.origin.position.y)
		]

		result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

		os.system(f'python3 {OPTIMIZER_PATH}/main_globaltraj.py --x {-1*self.map.info.origin.position.x} --y {-1*self.map.info.origin.position.y}')
		sleep(5)

		# Should be `result.stderr == ""` in the humble distro since we can reset slam there. Here, we're just gonna load an old track if this fails
		if result.stderr == "":  #os.path.exists(f'{OPTIMIZER_PATH}/outputs/traj_race_cl.csv'):
			self.get_logger().info("Raceline file computation successful.")

			if LOG_EXPERIMENTS: # log the raceline computation duration
				experiment_data_dir = "src/Senior-Design-Software/experiment_data"
				with open(os.path.join(experiment_data_dir, 'raceline_times.csv'), 'a', newline='') as csvfile:
					writer = csv.writer(csvfile)
					raceline_computation_duration = result.stdout.split("Runtime from import to final trajectory was ")[1].split("\n")[0]
					writer.writerow([raceline_computation_duration])

			raceline = Path()
			raceline.header.frame_id = 'map'

			# begin broadcasting the raceline
			with open(f'{OPTIMIZER_PATH}/outputs/traj_race_cl.csv', 'r') as csvfile:
				reader = csv.reader(csvfile, delimiter=';')
				for i in range(3):# skip the header rows
					next(reader)

				x_orig = []
				y_orig = []
				psi_orig = []

				for row in reader:
					x_orig.append(float(row[1]) + self.map.info.origin.position.x)
					y_orig.append(float(row[2]) + self.map.info.origin.position.y)
					psi_orig.append(float(row[3]))

				x_orig = np.array(x_orig)
				y_orig = np.array(y_orig)
				psi_orig = np.array(psi_orig)

				# rotate the raceline 90 degrees clockwise to align with the track
				x = y_orig
				y = -x_orig
				psi = psi_orig - np.pi / 2
				xy = np.c_[x, y]

				# now calculate the centroid of the raceline
				raceline_polygon = Polygon(xy)
				raceline_centroid = np.array([raceline_polygon.centroid.x, raceline_polygon.centroid.y])

				# now move the raceline to the track centroid
				diff = track_centroid - raceline_centroid
				xy += diff

				# throw in some more flips for good measure
				center_x = track_centroid[0]
				# center_y = track_centroid[1]
				xy[:, 0] = 2.0 * center_x - xy[:, 0] # mirror across x
				# xy[:, 1] = 2.0 * center_y - xy[:, 1] # mirror across y
				psi += np.pi # add 180 degrees to the orientation to keep in line with the track
				psi -= np.pi / 2 #

				### DEBUGGING CODE ### this will allos us to skip straight to raceline for testing if we get a good one
				np.save(f'src/Senior-Design-Software/debug_data/{DEBUG_KEY}/raceline_xy.npy', xy)
				np.save(f'src/Senior-Design-Software/debug_data/{DEBUG_KEY}/raceline_psi.npy', psi)
				### DEBUGGING CODE ###

				for i in reversed(range(len(xy))):
					pose = PoseStamped()
					pose.header.frame_id = 'map'
					pose.pose.position.x = xy[i, 0]
					pose.pose.position.y = xy[i, 1]
					pose.pose.position.z = 0.0
					pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=np.sin(psi[i]/2), w=np.cos(psi[i]/2))

					raceline.poses.append(pose)

			self.raceline = raceline
			self.raceline_publisher_thread.start() # used for visualization
			self.state = RACELINE
		else:
			self.get_logger().error("Raceline file not found.")
			self.get_logger().info("EXITING PROGRAM")
			self.destroy_node()
			self.shutdown_flag = True

	# VVV Experiment logging functions (only ever used if LOG_EXPERIMENTS is True) VVV
	def set_up_csvs(self):
		"""Creates the CSV files for logging experiment data"""

		experiment_data_dir = "src/Senior-Design-Software/experiment_data"
		replanning_times_dir = os.path.join(experiment_data_dir, "replanning_times")

		if not os.path.exists(replanning_times_dir):
			os.makedirs(replanning_times_dir)

		# create the csv for logging replanning times for this trial
		with open(os.path.join(replanning_times_dir, f"{DEBUG_KEY}_replanning_times.csv"), 'w', newline='') as csvfile:
			writer = csv.writer(csvfile)
			writer.writerow(['pose_publish_time', 'plan_receive_time', 'time_difference_seconds'])

		# check if the global raceline time csv exists, and create it if it doesn't
		if not os.path.exists(os.path.join(experiment_data_dir, 'raceline_times.csv')):
			with open(os.path.join(experiment_data_dir, 'raceline_times.csv'), 'w', newline='') as csvfile:
				writer = csv.writer(csvfile)
				writer.writerow(['raceline_computation_duration_seconds'])

	def plan_callback(self, msg: Path):
		"""Logs the time it takes to receive a plan from the /plan topic."""

		experiment_data_dir = "src/Senior-Design-Software/experiment_data"
		replanning_times_dir = os.path.join(experiment_data_dir, "replanning_times")

		if self.last_pose_publish_time is not None:
			current_time = time.time()
			time_difference = current_time - self.last_pose_publish_time
			
			# Write to CSV
			with open(os.path.join(replanning_times_dir, f"{DEBUG_KEY}_replanning_times.csv"), 'a', newline='') as csvfile:
				writer = csv.writer(csvfile)
				writer.writerow([self.last_pose_publish_time, current_time, time_difference])
			
			self.get_logger().info(f"Plan received after {time_difference:.4f} seconds")
			self.last_pose_publish_time = None

	# VVV Misc functions VVV

	def killswitch_callback(self, msg):
		"""Callback for the killswitch topic."""
		if msg.data:
			self.get_logger().info("Car is shut down.")
			self.destroy_node()
			self.shutdown_flag = True

	def publish_raceline(self):
		"""Once this thread is activated, it will continuously publish the raceline for visualization purposes."""
		
		while True:
			self.raceline.header.stamp = self.get_clock().now().to_msg()
			self.raceline_publisher.publish(self.raceline)

			# also publish a pose array
			pose_array_msg = PoseArray()
			pose_array_msg.header.frame_id = 'map'
			pose_array_msg.header.stamp = self.get_clock().now().to_msg()
			pose_array_msg.poses = [pose.pose for pose in self.raceline.poses]
			self.pose_array_publisher.publish(pose_array_msg)

			sleep(0.1)

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