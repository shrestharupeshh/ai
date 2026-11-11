"""ROS 2 node bridging the STM32 base controller (serial, firmware/core/protocol.h) to standard topics.

Subscribes  /cmd_vel (geometry_msgs/Twist)        -> wheel speed set-points at 20 Hz (firmware stops after 300 ms silence)
Publishes   /wheel/odometry (nav_msgs/Odometry)    wheel-encoder odometry (twist is what the EKF fuses)
            /imu/data_raw (sensor_msgs/Imu)        gyro z only
            /battery_state (sensor_msgs/BatteryState)
            /base/state (diagnostic_msgs/DiagnosticStatus)  safety state + latched faults
Services    ~/enable, ~/disable, ~/clear_fault (std_srvs/Trigger)

Requires the `agrirover` Python package (software/) on PYTHONPATH: `pip install -e software`.
STATUS: not yet executed against ROS 2 or hardware.
"""
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_srvs.srv import Trigger

from agrirover import config
from agrirover.hw.base_driver import BaseDriver
from agrirover.kinematics import Odometry as WheelOdometry
from agrirover.kinematics import limit_twist, twist_to_wheels, wheels_to_twist


class BaseNode(Node):
    def __init__(self):
        super().__init__("agrirover_base")
        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("auto_enable", True)
        self.r, self.track = config.wheel_radius(), config.track_width()
        self.drv = BaseDriver(self.get_parameter("port").value, self.get_parameter("baud").value)
        cpr = config.rover()["motor"]["encoder_cpr_motor"] * config.rover()["motor"]["gear_ratio"]
        self.odo = WheelOdometry(self.r, self.track, cpr)
        self.cmd = (0.0, 0.0)
        self.t_cmd = self.get_clock().now()
        self.capacity_ah = config.rover()["battery"]["capacity_ah"]
        self.used_ah = 0.0
        self.t_prev_ms = None
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.pub_odom = self.create_publisher(Odometry, "wheel/odometry", 20)
        self.pub_imu = self.create_publisher(Imu, "imu/data_raw", 20)
        self.pub_bat = self.create_publisher(BatteryState, "battery_state", 5)
        self.pub_state = self.create_publisher(DiagnosticStatus, "base/state", 5)
        self.create_service(Trigger, "~/enable", lambda q, r: self._srv(r, lambda: self.drv.enable(True)))
        self.create_service(Trigger, "~/disable", lambda q, r: self._srv(r, lambda: self.drv.enable(False)))
        self.create_service(Trigger, "~/clear_fault", lambda q, r: self._srv(r, self.drv.clear_fault))
        self.create_timer(0.02, self.spin_serial)
        self.create_timer(0.05, self.send_cmd)
        self.create_timer(1.0, self.slow)
        self.drv.request_info()

    def _srv(self, res, fn):
        fn()
        res.success, res.message = True, "sent"
        return res

    def on_cmd(self, msg: Twist):
        v, w = limit_twist(msg.linear.x, msg.angular.z, 0.5, 1.2, 12.0, self.r, self.track)
        self.cmd, self.t_cmd = (v, w), self.get_clock().now()

    def send_cmd(self):
        age = (self.get_clock().now() - self.t_cmd).nanoseconds * 1e-9
        v, w = self.cmd if age < self.get_parameter("cmd_timeout").value else (0.0, 0.0)
        self.drv.set_wheels(*twist_to_wheels(v, w, self.r, self.track))

    def spin_serial(self):
        st = self.drv.poll()
        if st is None:
            return
        now = self.get_clock().now().to_msg()
        dt = None if self.t_prev_ms is None else ((st.t_ms - self.t_prev_ms) & 0xFFFFFFFF) / 1000.0
        self.t_prev_ms = st.t_ms
        gyro = st.gyro_z if math.isfinite(st.gyro_z) else None
        pose = self.odo.update(st.enc_left, st.enc_right, gyro, dt)
        v, w = wheels_to_twist(st.w_left, st.w_right, self.r, self.track)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = now, "odom", "base_footprint"
        od.pose.pose.position.x, od.pose.pose.position.y = pose.x, pose.y
        od.pose.pose.orientation = Quaternion(z=math.sin(pose.yaw / 2), w=math.cos(pose.yaw / 2))
        od.twist.twist.linear.x, od.twist.twist.angular.z = v, w
        od.twist.covariance[0], od.twist.covariance[35] = 1e-3, 5e-2      # wheel yaw rate is trusted less than the gyro
        od.pose.covariance[0] = od.pose.covariance[7] = od.pose.covariance[35] = 1e-2
        self.pub_odom.publish(od)
        if gyro is not None:
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = now, "imu_link"
            imu.angular_velocity.z = gyro
            imu.angular_velocity_covariance[8] = 4e-4
            imu.orientation_covariance[0] = -1.0                          # orientation not provided
            self.pub_imu.publish(imu)
        if dt:
            self.used_ah += st.ibat_ma / 1000.0 * dt / 3600.0
        self._last = st

    def slow(self):
        st = getattr(self, "_last", None)
        ds = DiagnosticStatus(name="agrirover_base", hardware_id="ARC-1")
        if st is None or not self.drv.link_ok(1.0):
            ds.level, ds.message = DiagnosticStatus.STALE, "no telemetry from the base controller"
            self.pub_state.publish(ds)
            return
        if st.state_name == "IDLE" and self.get_parameter("auto_enable").value and not st.faults:
            self.drv.enable(True)
        bat = BatteryState()
        bat.header.stamp = self.get_clock().now().to_msg()
        bat.voltage, bat.current = st.vbat_mv / 1000.0, -st.ibat_ma / 1000.0
        bat.design_capacity = bat.capacity = float(self.capacity_ah)
        bat.charge = max(0.0, self.capacity_ah - self.used_ah)
        bat.percentage = bat.charge / self.capacity_ah          # coulomb counting from a full charge; LiFePO4 voltage is too flat for SoC
        bat.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        bat.present = True
        self.pub_bat.publish(bat)
        ds.level = DiagnosticStatus.ERROR if st.faults else DiagnosticStatus.OK
        ds.message = st.state_name + (": " + ",".join(st.fault_names) if st.faults else "")
        ds.values = [KeyValue(key="state", value=st.state_name), KeyValue(key="faults", value=",".join(st.fault_names)),
                     KeyValue(key="vmot_mv", value=str(st.vmot_mv)), KeyValue(key="i_left_ma", value=str(st.i_left_ma)),
                     KeyValue(key="i_right_ma", value=str(st.i_right_ma)), KeyValue(key="rx_errors", value=str(self.drv.parser.rx_err))]
        self.pub_state.publish(ds)


def main():
    rclpy.init()
    node = BaseNode()
    try:
        rclpy.spin(node)
    finally:
        node.drv.set_wheels(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
