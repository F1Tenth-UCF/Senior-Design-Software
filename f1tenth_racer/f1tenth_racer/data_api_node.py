import rclpy
from rclpy.qos import QoSPresetProfiles
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped # motor temp, speed, distance traveled (odometry)
from flask import Flask, jsonify
import os
import threading

app = Flask(__name__)

global_state = {
    "speed": 0,
    "esc_temp": 0,
    "motor_temp": 0,
    "tachometer": 0,
    "error": 0
}

ERRORS = [
    "No error",
    "Over voltage",
    "Under voltage",
    "DRV8302 chip failure",
    "Over current",
    "ESC over temp",
    "Motor over temp",
]
SPEED_TO_ERPM_GAIN = 4614.0
SPEED_TO_ERPM_OFFSET = 0.0

def get_cpu_temp():
    # run the following command cat /sys/class/thermal/thermal_zone1/temp
    # to get the temperature of the CPU
    cpu_temp = os.popen("cat /sys/class/thermal/thermal_zone1/temp").read()
    cpu_temp = int(cpu_temp) / 1000
    return cpu_temp

def get_wifi_signal(interface: str = "wlan0"):
    # run the following command iwconfig wlan0
    # to get the signal strength of the WiFi
    signal_info = os.popen(f"iwconfig {interface}").read()
    try:
        signal_part = signal_info.split("Link Quality=")[1].split(" ")[0]
        n, d = signal_part.split("/")
        return int(n) / int(d)
    except Exception:
        # If parsing fails, return something safe
        return -1.0

@app.route("/data")
def get_data():
    print("get_data")
    try:

        data = {
            "speed": global_state["speed"],
            "cpu_temp": get_cpu_temp(),
            "vesc_temp": global_state["esc_temp"],
            "motor_temp": global_state["motor_temp"],
            "tachometer": global_state["tachometer"],
            "error": global_state["error"],
            "wifi_signal": get_wifi_signal()
        }
        return jsonify(data)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

class DataApiNode(Node):
    def __init__(self):
        super().__init__('data_api_node')
        
        self.subscription = self.create_subscription(VescStateStamped, "/sensors/core", self.vesc_state_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())        
    
    def vesc_state_callback(self, msg: VescStateStamped):
        global global_state

        global_state["speed"] = -(-msg.state.speed - SPEED_TO_ERPM_OFFSET) / SPEED_TO_ERPM_GAIN
        global_state["esc_temp"] = msg.state.temp_fet
        global_state["motor_temp"] = msg.state.temp_motor
        global_state["tachometer"] = msg.state.distance_traveled
        global_state["error"] = ERRORS[msg.state.fault_code]

def run_flask():
    app.run(host="0.0.0.0", port=5000)

def main(args=None):
    rclpy.init(args=args)
    api_node = DataApiNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(api_node)

    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000),
        daemon=True
    )
    flask_thread.start()

    try:
        # Spin the executor (this will block) until process is killed or interrupted
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown things gracefully
        executor.shutdown()
        api_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
