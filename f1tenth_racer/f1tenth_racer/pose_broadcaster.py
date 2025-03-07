import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from tf_transformations import quaternion_from_euler
import numpy as np
from cv2 import dilate, erode
import cv2
import logging
from typing import Tuple
import time

"""
This code serves as the mission controller for the race, guiding the behavior tree through the different stages of competition
"""

def build_graph(wall_coords: np.ndarray, cut: np.ndarray) -> Tuple[dict, np.ndarray]:
    """Build a graph of the wall by connecting the points in the wall to their neighbors. Cuts out the 
    
    Args:
        wall_coords: the coordinates of the points in the wall
        cut: the coordinates of the cut
    """

    graph = {}
    # convert to a set of tuples for O(1) neighbor lookups
    S = set(tuple(rc) for rc in wall_coords)
    S = S - set(tuple(rc) for rc in cut)
    for (r, c) in S:
        graph[(r, c)] = set()
    
    # For each point, look at its 8 neighbors to see if they are in the set
    directions = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)]
    for (r, c) in S:
        for dr, dc in directions:
            nbr = (r + dr, c + dc)
            if nbr in S:
                graph[(r, c)].add(nbr)

    # find the start point
    start_directions = [(1,0), (1,1), (1,-1)]
    for point in cut:
        for dr, dc in start_directions:
            start_candidate = (point[0] + dr, point[1] + dc)
            if start_candidate in S:
                return graph, start_candidate

    raise ValueError("No start point found")

def dfs_deepest_point(graph: dict, start: Tuple[int, int]) -> Tuple[int, int]:
    """Find the deepest point in the graph by performing a depth first search
    
    Args:
        graph: the graph of the wall
        start: the starting point of the search
    """

    stack = [(start, 0)]
    visited = set([start])
    deepest_node = start
    max_dist = 0

    while stack:
        node, dist = stack.pop()
        # If this is the furthest we have gotten, update
        if dist > max_dist:
            max_dist = dist
            deepest_node = node

        # Traverse neighbors
        for nbr in graph[node]:
            if nbr not in visited:
                visited.add(nbr)
                stack.append((nbr, dist+1))

    return deepest_node

def find_endpoint(label: int, labels: np.ndarray, x_pos: float) -> np.ndarray:
    """Find the endpoint of the wall by traveling along it in the direction of the track
    
    Args:
        label: the integer label of the connected component corresponding to the wall
        labels: the labels of the connected components in the image
        x_pos: the x position of the starting point

    Returns:
        the endpoint of the wall
    """

    # extract the x,y coordinates of the points in the wall
    wall_coords = np.transpose((labels == label).nonzero())
    wall_coords = wall_coords[:, [1, 0]]

    # get a vertical array of points that line up along the y-axis with the starting point.
    # this allows us to to cut the graph in half, removing the points behind the starting point.
    # it also tells us which side of the graph to start DFS on, since we need to move +1 from the x coordinate of the cut at the y coordinate of any point in the cut.
    wall_cut_candidates = wall_coords[abs(wall_coords[:,0] - x_pos) < 1]
    wall_cut = wall_cut_candidates[wall_cut_candidates[:,0] == np.max(wall_cut_candidates[:,0])]
    
    # build the graph
    wall_graph, start_point = build_graph(wall_coords, wall_cut)

    # find the deepest point in the graph
    deepest_point = dfs_deepest_point(wall_graph, start_point)

    return deepest_point

class Nav2Intermediary(Node):

    def __init__(self):
        super().__init__('nav2_intermediary')

        # pubsub setup for pose broadcasting
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
        self.pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 20)

    def map_callback(self, msg: OccupancyGrid):
        """Callback for the map topic. Determines the next goal pose for exploring the map"""

        resolution = msg.info.resolution
        x_pos = -1*(msg.info.origin.position.x/resolution)
        y_pos = -1*(msg.info.origin.position.y/resolution)
        
        # construct the occupancy grid
        data = np.array(msg.data)
        data_reshaped = data.reshape(msg.info.height, msg.info.width)

        # create the free space array
        masked_data = np.ma.masked_where(data_reshaped == -1, data_reshaped)
        masked_free_space = np.ma.masked_where(masked_data>40, masked_data)
        darkened_free_space = np.where(masked_free_space==0.0, 255, 0).astype(np.uint8)
        cv2.imwrite('/home/cavrel/f1tenth_ws/src/Senior-Design-Software/darkened_free_space.png', darkened_free_space)

        # create the walls array
        walls_array = np.where(data_reshaped<100, 0, 255).astype(np.uint8)
        cv2.imwrite('/home/cavrel/f1tenth_ws/src/Senior-Design-Software/walls_array.png', walls_array)

        # Find the connected components corresponding to the two walls
        walls_binary = (walls_array > 127).astype(np.uint8)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(walls_binary)

        # sort the stats by the number of pixels in the component
        sorted_indices = np.argsort(stats[:, 4], axis=0)[::-1]

        # remove the largest one, which is the background
        background_index = sorted_indices[0]
        sorted_indices = np.delete(sorted_indices, background_index, axis=0)

        # find the two walls on either side of the car
        up_ray = labels[0:int(y_pos), int(x_pos)]
        down_ray = labels[int(y_pos):, int(x_pos)]

        selected_walls = []
        for val in up_ray[::-1]:
            if val in sorted_indices:
                selected_walls.append(val)
                break
        for val in down_ray:
            if val in sorted_indices:
                selected_walls.append(val)
                break

        ##### DEBUG #####
        print("sorted_indices")
        print(sorted_indices)
        print(f"x_pos: {x_pos}, y_pos: {y_pos}")

        # print a file containing a map and visualization of the rays used to find the walls
        with open('/home/cavrel/f1tenth_ws/src/Senior-Design-Software/debug.txt', 'w') as f:
            labels_copy = labels.copy()
            labels_copy[0:int(y_pos), int(x_pos)] = 8
            labels_copy[int(y_pos):, int(x_pos)] = 9

            # Set NumPy print options to prevent truncation
            np.set_printoptions(threshold=np.inf, linewidth=np.inf)
            f.write(f"Labels markup array: \n{labels_copy}\n\nUp ray: \n{up_ray}\n\nDown ray: \n{down_ray}")
            # Reset NumPy print options to default
            np.set_printoptions(threshold=1000, linewidth=75)

        # create an image of the selected walls
        selected_walls_image = np.zeros_like(walls_array)
        for wall in selected_walls:
            selected_walls_image[labels == wall] = 255
        # Draw red pixels around target point
        radius = 5
        
        # Create RGB version of image
        selected_walls_rgb = cv2.cvtColor(selected_walls_image, cv2.COLOR_GRAY2RGB)
        
        # Draw red circle around target point
        cv2.circle(selected_walls_rgb, (int(x_pos), int(y_pos)), radius, (0,0,255), -1)
        
        # Convert back to grayscale for consistency with rest of code
        selected_walls_image = cv2.cvtColor(selected_walls_rgb, cv2.COLOR_RGB2GRAY)
        cv2.imwrite('/home/cavrel/f1tenth_ws/src/Senior-Design-Software/selected_walls.png', selected_walls_image)

        ##### DEBUG #####

        try:
            assert len(selected_walls) == 2
        except AssertionError:
            self.get_logger().error(f"Found {len(selected_walls)} walls")
            return

        self.get_logger().info(f"Got {num_labels} connected components")
        self.get_logger().debug(str(stats))
        self.get_logger().debug(str(sorted_indices))

        self.get_logger().info(f"Selected walls: {selected_walls}")

        # Find the endpoint of both walls
        wall_1_deepest_point = find_endpoint(selected_walls[0], labels, x_pos)
        wall_2_deepest_point = find_endpoint(selected_walls[1], labels, x_pos)

        # Find the target point
        def find_axis_midpoint(wall_1_coord, wall_2_coord):
            larger = np.max([wall_1_coord, wall_2_coord])
            smaller = np.min([wall_1_coord, wall_2_coord])
            return smaller + (larger - smaller) / 2
        
        target_point = np.array([find_axis_midpoint(wall_1_deepest_point[0], wall_2_deepest_point[0]), find_axis_midpoint(wall_1_deepest_point[1], wall_2_deepest_point[1])])

        self.get_logger().debug(f"Wall 1 deepest point: {wall_1_deepest_point}")
        self.get_logger().debug(f"Wall 2 deepest point: {wall_2_deepest_point}")
        self.get_logger().info(f"Target point: {target_point}")

        # find the direction that does not intersect the free space
        rise = np.max([wall_2_deepest_point[1], wall_1_deepest_point[1]]) - np.min([wall_2_deepest_point[1], wall_1_deepest_point[1]])
        run = np.max([wall_2_deepest_point[0], wall_1_deepest_point[0]]) - np.min([wall_2_deepest_point[0], wall_1_deepest_point[0]])

        perpendicular_slope_vector = np.array([-rise, run])
        perpendicular_slope_vector_normalized = perpendicular_slope_vector / np.linalg.norm(perpendicular_slope_vector)

        test_point = target_point + perpendicular_slope_vector_normalized * 3

        if darkened_free_space[int(test_point[1]), int(test_point[0])] > 127:
            direction = -1 * perpendicular_slope_vector_normalized
        else:
            direction = perpendicular_slope_vector_normalized

        base_direction = np.array([1,0])
        angle = np.arccos(np.dot(direction, base_direction)/(np.linalg.norm(direction)*np.linalg.norm(base_direction)))
        quaternion = quaternion_from_euler(0, 0, angle)

        self.get_logger().info(f"Angle: {angle}")

        # construct the pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = target_point[0]*resolution + msg.info.origin.position.x
        pose.pose.position.y = target_point[1]*resolution + msg.info.origin.position.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.pose_publisher.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    nav2_intermediary = Nav2Intermediary()

    rclpy.spin(nav2_intermediary)
    
    nav2_intermediary.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()