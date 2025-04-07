import rclpy
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy.time
from std_msgs.msg import Bool
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
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
"""
Because we are using a PixHawk FCU to send signals to the ESC, we must go through initialization steps to arm the motors.
When initialized, this class goes through these steps.

This code is a modified version of the ROS2Control class from the OpenConvoy paper written by me (Owen Burns)"""

EXPLORATION = 0
COMPUTING_RACELINE = 1
RACELINE = 2

OPTIMIZER_PATH = '/home/cavrel/f1tenth_ws/global_racetrajectory_optimization'
DEBUG_KEY = str(time.time())
os.makedirs(f"/home/cavrel/f1tenth_ws/src/Senior-Design-Software/debug_data/{DEBUG_KEY}/pose_graph_images", exist_ok=True)

# TODO: fix issue where graph created by sknw is has no cycles. doesn't happen much anymore since i modified the slam params to filter out far away lidar points.
def display_binary_image(image, title):
	plt.figure(figsize=(10, 8))
	plt.imshow(image, cmap='gray')
	plt.savefig(f'/home/cavrel/f1tenth_ws/src/Senior-Design-Software/debug_data/{DEBUG_KEY}/{title}.png')
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
    plt.savefig(f'/home/cavrel/f1tenth_ws/src/Senior-Design-Software/debug_data/{DEBUG_KEY}/{title}.png')
    plt.close()

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
	np.save(f'/home/cavrel/f1tenth_ws/src/Senior-Design-Software/debug_data/{DEBUG_KEY}/track_raw.npy', binary_space)
	### DEBUGGING CODE ###

	# eliminate holes and noise
	track = binary_opening(binary_space, iterations=2)
	track = binary_closing(track, iterations=2)

	display_binary_image(track, 'track')

	# get a 1 pixel wide skeleton of the centerline
	# D = distance_transform_edt(track)
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

	plt.savefig(f'/home/cavrel/f1tenth_ws/src/Senior-Design-Software/debug_data/{DEBUG_KEY}/centerline_debug.png')
	plt.close()
	### DEBUGGING CODE ###

	return centerline, right_width, left_width

def format_pose(row, coordinate_offset, raceline_offset, angle_offset):

    x_orig = float(row[1]) + coordinate_offset[0]
    y_orig = float(row[2]) + coordinate_offset[1]
    psi_orig = float(row[3]) # Heading of raceline in current point from -pi to +pi rad. Zero is north (along y-axis).

    # rotate the raceline 90 degrees clockwise
    x = y_orig
    y = -x_orig
    psi = psi_orig - np.pi / 2

    # reflect aross the x axis
    y = -y
    psi = -psi

    # translate for final fit
    x += raceline_offset[0]
    y += raceline_offset[1]

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=np.sin(psi/2), w=np.cos(psi/2))
    
    return pose

class Nav2Intermediary(Node):

	def __init__(self):
		super().__init__('nav2_intermediary')

		# pubsub setup
		self.nav2_velo_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.gap_follow_subscriber = self.create_subscription(AckermannDriveStamped, '/wall_follower/cmd_vel', self.gap_follow_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		
		self.killswitch_subscriber = self.create_subscription(Bool, '/f1tenth_racer/killswitch', self.killswitch_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.pose_graph_subscriber = self.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization', self.pose_graph_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.vel_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 20)
		self.raceline_publisher = self.create_publisher(Path, '/raceline', 20)

		self.state = EXPLORATION
		self.map: OccupancyGrid = None

		self.raceline_computation_thread = threading.Thread(target=self.compute_raceline)
		self.raceline_publisher_thread = threading.Thread(target=self.publish_raceline)

	def publish_raceline(self):
		"""Publishes the raceline to the raceline topic, with updated timestamps."""
		while True:
			self.raceline.header.stamp = self.get_clock().now().to_msg()
			self.raceline_publisher.publish(self.raceline)
			sleep(0.1)

	def compute_raceline(self):
		"""Runs in its own thread and computes the raceline, then kicks off the raceline execution stage of the race."""

		# preprocess the occupancy grid
		centerline, right_width, left_width = preprocess_track(self.map)

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
		os.system(f"cp {OPTIMIZER_PATH}/inputs/tracks/hec_track.csv /home/cavrel/f1tenth_ws/src/Senior-Design-Software/debug_data/{DEBUG_KEY}/hec_track.csv")
		### DEBUGGING CODE ###

		self.get_logger().info("Track data written to csv. Running optimization.")

		# run the optimization
		os.system(f'python3 {OPTIMIZER_PATH}/main_globaltraj.py --x {-1*self.map.info.origin.position.x} --y {-1*self.map.info.origin.position.y}')
		sleep(5)

		# read the output file
		if os.path.exists(f'{OPTIMIZER_PATH}/outputs/traj_race_cl.csv'):
			self.get_logger().info("Raceline file found.")

			raceline = Path()
			raceline.header.frame_id = 'map'

			# begin broadcasting the raceline
			with open(f'{OPTIMIZER_PATH}/outputs/traj_race_cl.csv', 'r') as csvfile:
				reader = csv.reader(csvfile, delimiter=';')
				for i in range(3):# skip the header rows
					next(reader)

				first_row = next(reader)

				coordinate_offset = np.array([-float(first_row[1]), -float(first_row[2])]) # offset from the optimizer frame to the map frame
				raceline_offset = np.array([float(first_row[1]) + self.map.info.origin.position.x, float(first_row[2]) + self.map.info.origin.position.y]) # offset between the centerline and the raceline

				first_pose = format_pose(first_row, coordinate_offset, raceline_offset)
				raceline.poses.append(first_pose)

				for row in reader:
					pose = format_pose(row, coordinate_offset, raceline_offset)
					raceline.poses.append(pose)

			self.raceline = raceline
			self.raceline_publisher_thread.start()
			self.state = RACELINE
		else:
			self.get_logger().error("Raceline file not found.")
			self.get_logger().info("EXITING PROGRAM")
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

		if self.state == EXPLORATION: # this ensures that the map isn't updates while the raceline is being computed
			self.map = msg

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
			plt.savefig(f'/home/cavrel/f1tenth_ws/src/Senior-Design-Software/debug_data/{DEBUG_KEY}/pose_graph_images/marker_array_{self.get_clock().now().to_msg().sec}.png')
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