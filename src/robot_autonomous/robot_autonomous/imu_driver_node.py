#!/usr/bin/env python3
"""
IMU ROS Yahboom Driver Node untuk ROS Humble
=========================================================================
Hardware : Yahboom 6-DOF IMU (JY901 / WT901 protocol via UART)
Protocol : Binary packet 0x55 header – Acc, Gyro, Angle frames
Default  : /dev/ttyUSB1 @ 115200 baud
Datarate : 100 Hz (default Yahboom firmware)

Published Topics:
  /imu/data         [sensor_msgs/Imu]          – Filtered IMU (quat + gyro + accel)
  /imu/data_raw     [sensor_msgs/Imu]          – Raw IMU (tanpa quaternion)
  /imu/mag          [sensor_msgs/MagneticField] – Magnetometer (bila ada)
  /imu/temperature  [sensor_msgs/Temperature]   – Suhu sensor

Frame IDs : imu_link

Usage:
  ros2 run robot_autonomous imu_driver_node.py
  ros2 run robot_autonomous imu_driver_node.py --ros-args -p port:=/dev/ttyUSB1
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Quaternion

import serial
import struct
import math
import time


# ── Yahboom/JY901 packet constants ───────────────────────────────────────────
HEADER       = 0x55
CMD_ACC      = 0x51   # acceleration
CMD_GYRO     = 0x52   # gyroscope
CMD_ANGLE    = 0x53   # euler angles
CMD_MAG      = 0x54   # magnetometer
CMD_TEMP     = 0x56   # temperature
PACKET_LEN   = 11     # header(1) + cmd(1) + data(8) + checksum(1)

G = 9.80665           # gravity m/s²
DEG2RAD = math.pi / 180.0


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (rad) → quaternion (x,y,z,w)"""
    cr = math.cos(roll  * 0.5)
    sr = math.sin(roll  * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw   * 0.5)
    sy = math.sin(yaw   * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class IMUDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('port',        '/dev/ttyUSB1')
        self.declare_parameter('baudrate',    115200)
        self.declare_parameter('frame_id',    'imu_link')
        self.declare_parameter('gyro_range',  250.0)    # ±250 or ±500 °/s
        self.declare_parameter('accel_range', 16.0)     # ±2, ±4, ±8, ±16 g
        self.declare_parameter('publish_rate', 100.0)   # Hz

        self.port        = self.get_parameter('port').value
        self.baudrate    = self.get_parameter('baudrate').value
        self.frame_id    = self.get_parameter('frame_id').value
        self.gyro_range  = self.get_parameter('gyro_range').value
        self.accel_range = self.get_parameter('accel_range').value

        # ── QoS ──────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers ────────────────────────────────────────────────
        self.pub_imu     = self.create_publisher(Imu,          '/imu/data',        qos)
        self.pub_raw     = self.create_publisher(Imu,          '/imu/data_raw',    qos)
        self.pub_mag     = self.create_publisher(MagneticField,'/imu/mag',         qos)
        self.pub_temp    = self.create_publisher(Temperature,  '/imu/temperature', qos)

        # ── Internal state ────────────────────────────────────────────
        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.mx = self.my = self.mz = 0.0
        self.temperature = 25.0
        self.buf = bytearray()

        # ── Serial ───────────────────────────────────────────────────
        self.ser = None
        self.connect_serial()

        # ── Timer ─────────────────────────────────────────────────────
        self.create_timer(0.005, self.timer_callback)  # 200 Hz poll
        self.get_logger().info(f'IMU Yahboom driver started on {self.port}@{self.baudrate}')

    # ─────────────────────────────────────────────────────────────────────────
    def connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate, timeout=0
            )
            self.get_logger().info(f'Serial IMU terbuka: {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Gagal buka serial IMU: {e}')
            self.ser = None

    # ─────────────────────────────────────────────────────────────────────────
    def timer_callback(self):
        if self.ser is None:
            self.connect_serial()
            return

        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                self.buf.extend(self.ser.read(waiting))
                self.parse_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f'IMU serial error: {e}')
            self.ser = None

    # ─────────────────────────────────────────────────────────────────────────
    def parse_buffer(self):
        """Parse Yahboom JY901 binary packet stream"""
        while len(self.buf) >= PACKET_LEN:
            # Find header
            idx = self.buf.find(bytes([HEADER]))
            if idx < 0:
                self.buf.clear()
                return
            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < PACKET_LEN:
                return

            packet = self.buf[:PACKET_LEN]

            # Checksum: sum of bytes[0..9] & 0xFF == bytes[10]
            checksum = sum(packet[:10]) & 0xFF
            if checksum != packet[10]:
                del self.buf[0]   # discard header byte, re-sync
                continue

            cmd = packet[1]
            data = packet[2:10]

            if cmd == CMD_ACC:
                self._parse_acc(data)
            elif cmd == CMD_GYRO:
                self._parse_gyro(data)
                self.publish_imu()   # publish after getting gyro (full set)
            elif cmd == CMD_ANGLE:
                self._parse_angle(data)
            elif cmd == CMD_MAG:
                self._parse_mag(data)
                self.publish_mag()
            elif cmd == CMD_TEMP:
                self._parse_temp(data)
                self.publish_temp()

            del self.buf[:PACKET_LEN]

    def _parse_acc(self, data):
        raw_x, raw_y, raw_z = struct.unpack('<hhh', data[:6])
        scale = self.accel_range / 32768.0 * G
        self.ax = raw_x * scale
        self.ay = raw_y * scale
        self.az = raw_z * scale

    def _parse_gyro(self, data):
        raw_x, raw_y, raw_z = struct.unpack('<hhh', data[:6])
        scale = self.gyro_range / 32768.0 * DEG2RAD
        self.gx = raw_x * scale
        self.gy = raw_y * scale
        self.gz = raw_z * scale

    def _parse_angle(self, data):
        raw_r, raw_p, raw_y = struct.unpack('<hhh', data[:6])
        self.roll  = raw_r / 32768.0 * 180.0 * DEG2RAD
        self.pitch = raw_p / 32768.0 * 180.0 * DEG2RAD
        self.yaw   = raw_y / 32768.0 * 180.0 * DEG2RAD

    def _parse_mag(self, data):
        raw_x, raw_y, raw_z = struct.unpack('<hhh', data[:6])
        # Convert raw to Tesla (approximate: 1 raw ≈ 0.16 µT)
        self.mx = raw_x * 0.16e-6
        self.my = raw_y * 0.16e-6
        self.mz = raw_z * 0.16e-6

    def _parse_temp(self, data):
        raw_t = struct.unpack('<h', data[:2])[0]
        self.temperature = raw_t / 100.0

    # ─────────────────────────────────────────────────────────────────────────
    def publish_imu(self):
        now = self.get_clock().now().to_msg()

        # Filtered IMU (with quaternion from Euler)
        imu = Imu()
        imu.header.stamp    = now
        imu.header.frame_id = self.frame_id
        imu.orientation     = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        imu.orientation_covariance     = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu.angular_velocity.x         = self.gx
        imu.angular_velocity.y         = self.gy
        imu.angular_velocity.z         = self.gz
        imu.angular_velocity_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
        imu.linear_acceleration.x      = self.ax
        imu.linear_acceleration.y      = self.ay
        imu.linear_acceleration.z      = self.az
        imu.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        self.pub_imu.publish(imu)

        # Raw IMU (orientation unknown)
        raw = Imu()
        raw.header = imu.header
        raw.orientation_covariance[0]  = -1.0  # unknown
        raw.angular_velocity           = imu.angular_velocity
        raw.angular_velocity_covariance = imu.angular_velocity_covariance
        raw.linear_acceleration         = imu.linear_acceleration
        raw.linear_acceleration_covariance = imu.linear_acceleration_covariance
        self.pub_raw.publish(raw)

    def publish_mag(self):
        msg = MagneticField()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.magnetic_field.x = self.mx
        msg.magnetic_field.y = self.my
        msg.magnetic_field.z = self.mz
        self.pub_mag.publish(msg)

    def publish_temp(self):
        msg = Temperature()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.temperature     = self.temperature
        self.pub_temp.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = IMUDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
