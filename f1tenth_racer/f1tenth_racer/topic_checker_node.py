#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSPresetProfiles
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time

class TopicChecker(Node):
    def __init__(self):
        super().__init__('topic_checker')
        print("Node initialized") # Basic print for startup verification
        
        self.declare_parameter('topic_name', '')
        self.declare_parameter('topic_type', '')
        
        topic_name = self.get_parameter('topic_name').value
        topic_type_str = self.get_parameter('topic_type').value

        self.get_logger().info(f"Parameters received - topic: {topic_name}, type: {topic_type_str}")

        if topic_type_str == 'LaserScan':
            msg_type = LaserScan
        elif topic_type_str == 'Odometry':
            msg_type = Odometry
        elif topic_type_str == 'OccupancyGrid':
            msg_type = OccupancyGrid
        else:
            self.get_logger().error(f'Invalid topic type: {topic_type_str}')
            rclpy.shutdown()
            return
        
        self.subscription = self.create_subscription(msg_type, topic_name, self.topic_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f"Subscription created to {self.subscription.topic_name}")
        
        self.received_message = False
        
        self.timer = self.create_timer(1.0, self.log_waiting)
    
    def topic_callback(self, msg):
        if not self.received_message:
            self.received_message = True
            self.get_logger().info(f'Topic {self.subscription.topic_name} is publishing')
            self.destroy_node()
            self._shutdown_flag = True
    
    def log_waiting(self):
        if not self.received_message:
            self.get_logger().info(f'Waiting for topic {self.subscription.topic_name}...')

def main(args=None):
    rclpy.init(args=args)
    checker = TopicChecker()
    
    # Add a flag to check for shutdown
    checker._shutdown_flag = False
    
    # Spin until shutdown is requested
    while rclpy.ok() and not checker._shutdown_flag:
        rclpy.spin_once(checker)
        
    # Continuously attempt to shut down the node until it succeeds
    while True:
        try:
            rclpy.shutdown()
            break
        finally:
            time.sleep(1)

if __name__ == '__main__':
    main() 
