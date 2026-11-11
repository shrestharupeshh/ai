"""Publishes greenhouse climate readings from the mast sensor pod (SCD41, SHT45, AS7341) at 0.5 Hz.

Topics: /env/temperature (sensor_msgs/Temperature), /env/relative_humidity (sensor_msgs/RelativeHumidity),
        /env/co2_ppm (std_msgs/Float32), /env/light_raw (std_msgs/Float32), /env/json (std_msgs/String, full sample)
STATUS: not yet executed against ROS 2 or hardware.
"""
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RelativeHumidity, Temperature
from std_msgs.msg import Float32, String

from agrirover.hw.env_sensors import EnvSensors


class EnvNode(Node):
    def __init__(self):
        super().__init__("agrirover_env")
        self.sensors = None
        self.pt = self.create_publisher(Temperature, "env/temperature", 5)
        self.ph = self.create_publisher(RelativeHumidity, "env/relative_humidity", 5)
        self.pc = self.create_publisher(Float32, "env/co2_ppm", 5)
        self.pl = self.create_publisher(Float32, "env/light_raw", 5)
        self.pj = self.create_publisher(String, "env/json", 5)
        self.create_timer(2.0, self.tick)

    def tick(self):
        try:
            if self.sensors is None:
                self.sensors = EnvSensors()
            s = self.sensors.read()
        except Exception as exc:                     # keep retrying: a sensor fault must never take the robot down
            self.sensors = None
            self.get_logger().warn(f"environment sensors unavailable: {exc}", throttle_duration_sec=30.0)
            return
        stamp = self.get_clock().now().to_msg()
        t = Temperature(temperature=s["temperature_c"])
        t.header.stamp, t.header.frame_id = stamp, "env_link"
        h = RelativeHumidity(relative_humidity=s["humidity_pct"] / 100.0)
        h.header.stamp, h.header.frame_id = stamp, "env_link"
        self.pt.publish(t)
        self.ph.publish(h)
        if s["co2_ppm"] is not None:
            self.pc.publish(Float32(data=float(s["co2_ppm"])))
        self.pl.publish(Float32(data=float(s["light"])))
        self.pj.publish(String(data=json.dumps(s)))


def main():
    rclpy.init()
    node = EnvNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
