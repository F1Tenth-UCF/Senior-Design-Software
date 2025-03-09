import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
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
import csv
import os
from collections import deque
import collections
"""
This code serves as the mission controller for the race, guiding the behavior tree through the different stages of competition
"""

def build_graph(wall_coords: np.ndarray) -> Tuple[dict, np.ndarray]:
    """Build a graph of the wall by connecting the points in the wall to their neighbors. Cuts out the 
    
    Args:
        wall_coords: the coordinates of the points in the wall
        cut: the coordinates of the cut
    """

    graph = {}
    # convert to a set of tuples for O(1) neighbor lookups
    S = set(tuple(rc) for rc in wall_coords)
    for (r, c) in S:
        graph[(r, c)] = set()
    
    # For each point, look at its 8 neighbors to see if they are in the set
    directions = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)]
    for (r, c) in S:
        for dr, dc in directions:
            nbr = (r + dr, c + dc)
            if nbr in S:
                graph[(r, c)].add(nbr)

    return graph

def dfs_deepest_point(graph: dict, start: Tuple[int, int]) -> Tuple[int, int]:
    """Find the deepest point in the graph by performing a depth first search
    
    Args:
        graph: the graph of the wall
        start: the starting point of the search
    """

    # find the point in the graph closest to the start point
    distances = {}
    for node in graph:
        distances[node] = np.linalg.norm(np.array(node) - np.array(start))
    start = min(distances, key=distances.get)

    stack = [(start, 0)]
    visited = set([start])
    parent = {start: None}
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
                parent[nbr] = node
                stack.append((nbr, dist+1))

    deepest_parent = parent[deepest_node]

    # retreive the coordinates of the path from the deepest node to the start node
    path = []
    current = deepest_node
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()

    return deepest_node, deepest_parent, path

def bfs_furthest_point(graph: dict, start: Tuple[int, int]) -> Tuple[int, int, list]:
    """
    Use BFS to find the node in 'graph' whose shortest path from 'start'
    is the longest. Return (furthest_node, parent_of_furthest, path_to_furthest).

    Args:
        graph: A dict of the form { (r, c): set([(r1, c1), (r2, c2), ...]), ... }
               storing adjacency for each node.
        start: A (row, col) tuple for where to begin the BFS.

    Returns:
        furthest_node: The grid node with the greatest BFS distance from 'start'.
        parent_of_furthest: The parent of that furthest node in the BFS tree.
        path: The BFS path (list of (r, c)) from 'start' to furthest_node.
    """
    from collections import deque
    
    # Make sure the 'start' you use is actually in the graph; if not, pick the
    # nearest node in 'graph' to the supplied 'start'.
    if start not in graph:
        # find the point in the graph closest to 'start'
        start = min(
            graph.keys(),
            key=lambda node: np.linalg.norm(np.array(node) - np.array(start))
        )

    # Standard BFS init
    queue = deque([start])
    visited = set([start])
    parent = {start: None}
    distance = {start: 0}

    # Keep track of the node that ends up furthest away from 'start'
    furthest_node = start
    max_dist = 0

    while queue:
        node = queue.popleft()
        for nbr in graph[node]:
            if nbr not in visited:
                visited.add(nbr)
                parent[nbr] = node
                distance[nbr] = distance[node] + 1
                queue.append(nbr)

                # If this is the furthest so far, record it
                if distance[nbr] > max_dist:
                    max_dist = distance[nbr]
                    furthest_node = nbr

    # Now reconstruct path from furthest_node back to start
    path = []
    current = furthest_node
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()

    # The BFS tree's parent of 'furthest_node' is just parent[furthest_node],
    # but we can define "deepest_parent" as the node’s immediate parent to match
    # your usage if desired:
    deepest_parent = parent[furthest_node]
    return furthest_node, deepest_parent, path

class PoseBroadcaster(Node):

    def __init__(self):
        super().__init__('pose_broadcaster')

        # pubsub setup for pose broadcasting
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
        self.pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 20)
        self.plan_subscriber = self.create_subscription(Path, '/plan', self.plan_callback, QoSPresetProfiles.SYSTEM_DEFAULT.value, callback_group=MutuallyExclusiveCallbackGroup())
        
        # Timing variables
        self.last_pose_publish_time = None
        
        # CSV file setup
        self.csv_file_path = os.path.expanduser('~/f1tenth_ws/pose_plan_timing.csv')
        self.create_csv_file()
        
    def create_csv_file(self):
        """Create the CSV file with headers if it doesn't exist"""
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['pose_publish_time', 'plan_receive_time', 'time_difference_seconds'])
            self.get_logger().info(f"Created timing CSV file at {self.csv_file_path}")
    
    def plan_callback(self, msg: Path):
        """Callback for the plan topic. Records timing information between pose publishing and plan receiving."""
        if self.last_pose_publish_time is not None:
            current_time = time.time()
            time_difference = current_time - self.last_pose_publish_time
            
            # Write to CSV
            with open(self.csv_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([self.last_pose_publish_time, current_time, time_difference])
            
            self.get_logger().info(f"Plan received after {time_difference:.4f} seconds")
            self.last_pose_publish_time = None

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

        # Get coordinates of free space points (where value is 255)
        free_space_coords = np.transpose(np.where(darkened_free_space == 255))
        free_space_coords = free_space_coords[:, [1, 0]]

        # build the graph
        graph = build_graph(free_space_coords)

        # find the deepest point in the graph
        target_point, deepest_parent, path = bfs_furthest_point(graph, (int(x_pos), int(y_pos)))

        # plot the path
        free_space_copy = darkened_free_space.copy()
        for coord in path:
            free_space_copy[coord[1], coord[0]] = 120
        cv2.imwrite('/home/cavrel/f1tenth_ws/src/Senior-Design-Software/free_space_path.png', free_space_copy)
        
        self.get_logger().info(f"Target point: {target_point}")

        # find the direction that does not intersect the free space
        dx = target_point[0] - deepest_parent[0]
        dy = target_point[1] - deepest_parent[1]
        direction_vec = np.array([dx, dy], dtype=float)
        self.get_logger().info(f"Direction vector: {direction_vec}")
        direction_norm = direction_vec / np.linalg.norm(direction_vec)

        base_direction = np.array([1,0])
        angle = np.arccos(np.dot(direction_norm, base_direction)/(np.linalg.norm(direction_norm)*np.linalg.norm(base_direction)))
        quaternion = quaternion_from_euler(0, 0, -angle)

        self.get_logger().info(f"Angle: {np.rad2deg(angle)}")

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

        # Record the time before publishing the pose
        self.last_pose_publish_time = time.time()
        self.pose_publisher.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    pose_broadcaster = PoseBroadcaster()

    rclpy.spin(pose_broadcaster)
    
    pose_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()