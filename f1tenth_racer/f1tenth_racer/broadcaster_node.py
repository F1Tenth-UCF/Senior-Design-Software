import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSPresetProfiles
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time
from flask import Flask, jsonify
import random
import psutil

app = Flask(__name__)

@app.route("/data")
def get_data():
    try:
        data = {
            "speed": round(random.uniform(0, 200), 1),
            "rpm": random.randint(1000, 8000),
            "battery": round(random.uniform(10.5, 14.8), 2),
            "throttle": random.randint(0, 100),
            "brake": random.randint(0, 100),
            "gear": random.choice(["N", "1", "2", "3", "4", "5"]),
            "cpu_temp": round(random.uniform(45.0, 75.0), 1),  # Replaced with simulated value
            "gpu_usage": psutil.cpu_percent(interval=0.1),
            "lap_time": round(random.uniform(60.0, 90.0), 2),
            "wifi_signal": random.randint(-70, -30)
        }
        return jsonify(data)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

class Broadcaster(Node):
    def __init__(self):
        super().__init__('broadcaster')
        
        self.subscription = self.create_subscription(msg_type, topic_name, self.topic_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())        
    
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
    checker = Broadcaster()
    
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
    app.run(host="0.0.0.0", port=5000)
    main() 
