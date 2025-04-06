#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

##############################################################################
# Helper data structures (Obstacle, Gap, Corner, etc.) mirroring the C++ code
##############################################################################

# inspired by https://github.com/CTU-IIG/f1t-ftg/tree/master/src

class NoGapFoundException(Exception):
    """Thrown when no gap is found."""
    pass

class InvalidAngleException(Exception):
    """Thrown when an invalid angle is encountered."""
    pass

class CenterOutsideGapException(Exception):
    """Thrown if the computed gap center is outside the actual gap."""
    pass

class Obstacle:
    """
    Mirrors the behavior of the C++ 'Obstacle' struct/class.
    For simplicity, we'll store:
      - distance_to_center (float)
      - angle (float)
      - radius (float)  # optional if needed

    We'll also store x, y for convenience, which can be derived
    from (distance, angle).
    """
    def __init__(self, distance_to_center, angle, radius):
        self.distance_to_center = distance_to_center
        self.angle = angle
        self.radius = radius

        # For convenience, define x, y in vehicle coordinates:
        self.x = distance_to_center * math.cos(angle)
        self.y = distance_to_center * math.sin(angle)

        # Sometimes in the C++ code, 'distance' is the same as distance_to_center.
        # If the code uses `obstacle.distance`, you can define a property or just copy:
        self.distance = distance_to_center

    def overlaps(self, other):
        """
        Mirroring the logic that checks if two obstacles overlap horizontally.
        In the original code, `Overlaps()` is not fully shown, but often
        it's a bounding check or an angle check. We keep it simple here or
        adapt to your real logic.
        """
        # If the angles or positions of these obstacles cause them to merge as a "single obstacle".
        # This is a placeholder. You might want more logic:
        angle_diff = abs(self.angle - other.angle)
        return (angle_diff < 0.01)

    def distance_between_centers(self, other):
        """
        Euclidean distance between the centers of self and other.
        """
        dx = self.x - other.x
        dy = self.y - other.y
        return math.hypot(dx, dy)


class Gap:
    """
    Mirrors the C++ `Gap` struct, storing references to left and right obstacles
    and possibly other gap info (like gap_size).
    """
    def __init__(self, obstacle_left, obstacle_right):
        self.obstacle_left = obstacle_left
        self.obstacle_right = obstacle_right

        # In the original code, you may see references to 'angle_left' or 'angle_right'.
        self.angle_left = obstacle_left.angle
        self.angle_right = obstacle_right.angle

        # A "gap_size" can be computed in many ways. The code does a geometry-based approach.
        # For simplicity, let's define it as difference in angles. You can adapt if needed.
        self.gap_size = abs(self.angle_left - self.angle_right)

        # For "vertical gap" logic, you might define gap_distance, etc.


class Corner:
    """
    Mirrors the C++ `Corner` struct. 
    Here we store references to obstacle_left and obstacle_right, plus a corner type, etc.
    """
    class CornerTypes:
        kRight = 1
        kLeft = 2

    def __init__(self, obstacle_left, obstacle_right, corner_type):
        self.obstacle_left = obstacle_left
        self.obstacle_right = obstacle_right
        self.corner_type = corner_type

        # Compute gap_size similarly or however your original code does it.
        # Example:
        self.gap_size = obstacle_left.distance_between_centers(obstacle_right)

    def CornerType(self):
        return self.corner_type


class LidarData:
    """
    Simple container to mimic the usage in the C++ code (range_min, range_max, angle_min, angle_increment, etc.).
    """
    def __init__(self, range_min, range_max, angle_min, angle_max, angle_increment, ranges):
        self.range_min = range_min
        self.range_max = range_max
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.ranges = ranges


##############################################################################
# Translating the FollowTheGap logic from C++ to Python
##############################################################################

# Global-like parameters (mirroring the original static variables)
kMaxRange = 0.9
g_fovAngleMax = math.pi / 2 + math.pi / 16
g_goal_angle = 0.0  # Suppose your high-level goal angle is 0 (straight)
kDistanceToCorner = 0.22
kTurnRadius = 0.3
kCarRadius = 0.4
kGapWeightCoefficient = 100.0
kCornerWeightCoefficient = 100.0
kFovAngle = math.pi / 4
kTrackMinWidth = 0.35

# "Nhol" obstacles used in the original code
obstacle_nhol_left = Obstacle(kTurnRadius, math.pi/2, kTurnRadius)
obstacle_nhol_right = Obstacle(kTurnRadius, -math.pi/2, kTurnRadius)


def filter_obstacles(obstacles_in):
    """
    Applies the same logic as `FilterObstacles(...)` in the C++ code:
     - Filter out data outside fov
     - Filter out data beyond kMaxRange
     - Possibly filter 'lone obstacles'
    For simplicity, we'll do minimal filtering here. You can adapt to
    exactly replicate your C++ approach.
    """
    global g_fovAngleMax, kMaxRange

    # 1) Filter out by angle and distance
    filtered = []
    for obs in obstacles_in:
        if abs(obs.angle) <= g_fovAngleMax and obs.distance_to_center <= kMaxRange:
            filtered.append(obs)

    # 2) Optionally do additional filtering for "lone" obstacles
    # The original code includes "FilterLoneObstacleGroups" to remove small spurious points.
    # You can place that logic here if desired.

    return filtered


def find_gaps_angle(obstacles):
    """
    Mimic `FindGapsAngle()` from the C++ code.
    We look at consecutive obstacles, check if they do *not* overlap, then form a gap.
    """
    gaps = []
    if not obstacles:
        return gaps

    # Sort by angle so we can consider them in ascending order.
    # (C++ code presumably had them sorted already.)
    obstacles.sort(key=lambda o: o.angle)

    for i in range(1, len(obstacles)):
        left_obs = obstacles[i-1]
        right_obs = obstacles[i]

        # If they do not "overlap", form a gap
        # The original code checks angles or uses Overlaps() logic.
        # We'll do a simple angle-based approach:
        if not left_obs.overlaps(right_obs):
            # Make a new Gap
            gap = Gap(left_obs, right_obs)
            # Possibly check if the gap is "valid" (e.g., gap.gap_size > something)
            # For now, just append it.
            gaps.append(gap)

    return gaps


def calculate_gap_center_angle(gap):
    """
    Translate `CalculateGapCenterAngle(...)` from C++.
    This is the more advanced geometry approach. The code in C++ uses
    distances, angles, and arcsine/cosine logic.
    For brevity, we can do the simpler approach or replicate fully.
    """
    # We'll attempt the simpler approach from the fallback function
    # if the complex approach might cause issues. 
    # The fallback in the original code is `CalculateGapCenterAngleBasic`.
    # Let's do the same logic:

    try:
        # distance to each obstacle
        d1 = gap.obstacle_right.distance
        d2 = gap.obstacle_left.distance
        theta1 = abs(gap.obstacle_right.angle)
        theta2 = abs(gap.obstacle_left.angle)

        # The original code has multiple if/else blocks depending on sign.
        # We'll do a direct translation of that approach. 
        # For simplicity, if the sign pattern is complicated, we might revert to the basic average.
        if (gap.angle_left >= 0.0) and (gap.angle_right <= 0.0):
            # mirrored from C++:
            #   theta_gap_c = acos( (d1 + d2*cos(theta1+theta2)) / sqrt(...) ) - theta1
            # big chunk of geometry ...
            # We'll demonstrate a partial approach, but this can get complex quickly:
            numerator = d1 + d2*math.cos(theta1 + theta2)
            denominator = math.sqrt(d1*d1 + d2*d2 + 2.0*d1*d2*math.cos(theta1 + theta2))
            val = numerator / denominator
            # clamp val for domain safety
            val = max(min(val, 1.0), -1.0)
            theta_gap_c = math.acos(val) - theta1
            if math.isnan(theta_gap_c):
                raise InvalidAngleException("gap center angle is NaN.")
        elif gap.angle_right >= 0.0:
            # Another geometry block
            # If you'd rather keep it simple, do the fallback
            raise CenterOutsideGapException("Simplifying for demonstration!")
        else:
            # gap.angle_left <= 0.0
            raise CenterOutsideGapException("Simplifying for demonstration!")

        # Check if the center angle is inside the gap. If not, throw.
        if (theta_gap_c > gap.obstacle_left.angle) or (theta_gap_c < gap.obstacle_right.angle):
            raise CenterOutsideGapException("Center angle outside gap")

        if math.isnan(theta_gap_c):
            raise InvalidAngleException("gap center angle is NaN unknown case")

        return theta_gap_c

    except (InvalidAngleException, CenterOutsideGapException):
        # Fall back to basic approach
        return calculate_gap_center_angle_basic(gap)


def calculate_gap_center_angle_basic(gap):
    """
    Direct average of left and right angles: (theta_1 + theta_2)/2
    from `CalculateGapCenterAngleBasic` in C++.
    """
    return 0.5 * (gap.obstacle_right.angle + gap.obstacle_left.angle)


def calculate_final_heading_angle(theta_goal, theta_c, d_min, alpha):
    """
    Mirror of `CalculateFinalHeadingAngle(...)` in C++:
       theta_final = ((alpha/d_min) * theta_c + theta_goal) / ((alpha/d_min) + 1)
    """
    # Prevent division by zero or extremely small values
    if d_min < 0.001:
        d_min = 0.001
    return ((alpha / d_min) * theta_c + theta_goal) / ((alpha / d_min) + 1.0)


def find_nearest_obstacle(obstacles):
    """
    Return the obstacle with minimal distance.
    """
    return min(obstacles, key=lambda o: o.distance)


def follow_the_gap_method(obstacles, lidar_data):
    """
    This is a direct translation of `FollowTheGap::FollowTheGapMethod`.
    Returns a steering angle (float).
    """
    # 1) Find all gaps
    gaps = find_gaps_angle(obstacles)
    if not gaps:
        raise NoGapFoundException("No gaps found at all")

    # 2) Find the largest gap by gap_size
    largest_gap = max(gaps, key=lambda g: g.gap_size)

    # 3) Calculate center angle of that gap
    gap_center_angle = calculate_gap_center_angle(largest_gap)

    # 4) Find nearest obstacle distance
    nearest = find_nearest_obstacle(obstacles)
    d_min = nearest.distance

    # 5) Calculate final heading angle
    final_angle = calculate_final_heading_angle(
        g_goal_angle, gap_center_angle, d_min, kGapWeightCoefficient
    )

    if math.isnan(final_angle):
        raise NoGapFoundException("Result angle is NaN")

    return final_angle


def run_callback(obstacles_in, lidar_data):
    """
    A Pythonic version of `FollowTheGap::Callback`. 
    We try multiple attempts with different ranges, fallback logic, etc.
    For simplicity, we’ll do just one attempt: call follow_the_gap_method.
    """
    # Filter obstacles first
    obs = filter_obstacles(obstacles_in)

    if not obs:
        raise NoGapFoundException("No valid obstacles after filtering")

    # Just do a single attempt:
    angle = follow_the_gap_method(obs, lidar_data)
    return angle


##############################################################################
# ROS 2 Node
##############################################################################

class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')

        # Create subscriber to /scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Create publisher to /drive
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.get_logger().info("FollowTheGapNode initialized. Subscribed to /scan, publishing to /drive")

    def scan_callback(self, msg: LaserScan):
        """
        Callback whenever a LaserScan is received on /scan.
        We'll convert the LaserScan to a list of 'Obstacle's
        (in a simplistic way), then call run_callback(...).
        Finally, we'll publish an AckermannDrive command.
        """
        # Convert LaserScan into an array of "Obstacle" objects
        # ignoring intensities. For each valid range, we compute angle,
        # create an obstacle, etc.
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment
        range_min = msg.range_min
        range_max = msg.range_max

        # Store it into a LidarData structure (mirroring the original usage)
        lidar_data = LidarData(
            range_min=range_min,
            range_max=range_max,
            angle_min=angle_min,
            angle_max=angle_max,
            angle_increment=angle_inc,
            ranges=msg.ranges
        )

        # Build obstacles
        obstacles = []
        angle = angle_min
        for r in msg.ranges:
            # Filter out invalid ranges
            if r < range_min or r > range_max:
                angle += angle_inc
                continue

            # Create an obstacle with radius=0.0 for simplicity
            obs = Obstacle(r, angle, 0.0)
            obstacles.append(obs)
            angle += angle_inc

        # Attempt to run follow-the-gap
        try:
            final_heading_angle = run_callback(obstacles, lidar_data)
            # Publish result
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.35  # fixed speed
            drive_msg.drive.steering_angle = final_heading_angle

            self.drive_pub.publish(drive_msg)
            self.get_logger().info(f"Publishing steering={final_heading_angle:.3f}, speed=0.35")

        except NoGapFoundException as e:
            self.get_logger().warn(f"No gap found: {e}")
            # Optionally publish something safe, e.g., stop or small angle
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()