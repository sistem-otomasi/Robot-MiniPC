#!/usr/bin/env python3
"""
Robot Controller Node – Autonomous Mobile Robot
=========================================================================
Menerima perintah velocity dari Nav2 (/cmd_vel) dan mengirimkan ke
motor driver melalui serial UART.

Subscribed Topics :
  /cmd_vel          [geometry_msgs/Twist]   – Perintah kecepatan dari Nav2

Published Topics  :
  /odom             [nav_msgs/Odometry]     – Odometri dari encoder
  /robot/status     [std_msgs/String]       – Status robot

Serial protocol ke embedded (mikrokontroler):
  Kirim: "V:{linear_x:.3f},{angular_z:.3f}\n"
  Terima: "E:{left_ticks},{right_ticks}\n"

Usage:
  ros2 run robot_autonomous robot_controller_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import serial
import math
import time

from tf2_ros import TransformBroadcaster


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('serial_port',     '/dev/ttyACM0')
        self.declare_parameter('serial_baud',     115200)
        self.declare_parameter('wheel_base',      0.30)   # jarak antar roda (m)
        self.declare_parameter('wheel_radius',    0.065)  # radius roda (m)
        self.declare_parameter('encoder_ticks',   1440)   # ticks per revolusi
        self.declare_parameter('max_linear_vel',  1.0)    # m/s
        self.declare_parameter('max_angular_vel', 2.0)    # rad/s
        self.declare_parameter('odom_frame',      'odom')
        self.declare_parameter('base_frame',      'base_link')
        self.declare_parameter('publish_tf',      True)

        port     = self.get_parameter('serial_port').value
        baud     = self.get_parameter('serial_baud').value
        self.wheel_base    = self.get_parameter('wheel_base').value
        self.wheel_radius  = self.get_parameter('wheel_radius').value
        self.enc_ticks     = self.get_parameter('encoder_ticks').value
        self.max_lin       = self.get_parameter('max_linear_vel').value
        self.max_ang       = self.get_parameter('max_angular_vel').value
        self.odom_frame    = self.get_parameter('odom_frame').value
        self.base_frame    = self.get_parameter('base_frame').value
        self.publish_tf    = self.get_parameter('publish_tf').value

        # ── QoS ──────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers & Subscriptions ────────────────────────────────
        self.pub_odom   = self.create_publisher(Odometry, '/odom',          qos)
        self.pub_status = self.create_publisher(String,   '/robot/status',  10)
        self.sub_cmd    = self.create_subscription(Twist, '/cmd_vel',
                                                   self.cmd_vel_callback, 10)

        # ── TF broadcaster ────────────────────────────────────────────
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # ── Serial ───────────────────────────────────────────────────
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Controller serial terbuka: {port}@{baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Gagal buka serial controller: {e}')

        # ── Odometry state ────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.th  = 0.0    # heading (rad)
        self.prev_left  = 0
        self.prev_right = 0
        self.last_time  = self.get_clock().now()

        # ── Timers ────────────────────────────────────────────────────
        self.create_timer(0.02,  self.read_encoder_callback)  # 50 Hz read
        self.create_timer(0.1,   self.status_callback)        # 10 Hz status

        self.get_logger().info('Robot Controller Node started')

    # ─────────────────────────────────────────────────────────────────────────
    def cmd_vel_callback(self, msg: Twist):
        """Kirim perintah kecepatan ke embedded"""
        if self.ser is None:
            return

        lin = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        ang = max(-self.max_ang, min(self.max_ang, msg.angular.z))

        cmd = f'V:{lin:.4f},{ang:.4f}\n'
        try:
            self.ser.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Write serial error: {e}')

    # ─────────────────────────────────────────────────────────────────────────
    def read_encoder_callback(self):
        """Baca encoder dari embedded dan hitung odometri"""
        if self.ser is None:
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('E:'):
                    parts = line[2:].split(',')
                    if len(parts) == 2:
                        left  = int(parts[0])
                        right = int(parts[1])
                        self.compute_odometry(left, right)
        except (serial.SerialException, ValueError) as e:
            self.get_logger().debug(f'Encoder read error: {e}')

    def compute_odometry(self, left_ticks: int, right_ticks: int):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Ticks delta
        d_left  = (left_ticks  - self.prev_left)  / self.enc_ticks * 2 * math.pi * self.wheel_radius
        d_right = (right_ticks - self.prev_right) / self.enc_ticks * 2 * math.pi * self.wheel_radius
        self.prev_left  = left_ticks
        self.prev_right = right_ticks

        # Center distance & heading change
        d_center = (d_left + d_right) / 2.0
        d_th     = (d_right - d_left) / self.wheel_base

        self.x  += d_center * math.cos(self.th + d_th / 2.0)
        self.y  += d_center * math.sin(self.th + d_th / 2.0)
        self.th += d_th

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp         = now.to_msg()
        odom.header.frame_id      = self.odom_frame
        odom.child_frame_id       = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Quaternion from yaw
        q_z = math.sin(self.th / 2.0)
        q_w = math.cos(self.th / 2.0)
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w

        if dt > 0:
            odom.twist.twist.linear.x  = d_center / dt
            odom.twist.twist.angular.z = d_th / dt

        self.pub_odom.publish(odom)

        # TF odom → base_link
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp            = now.to_msg()
            t.header.frame_id         = self.odom_frame
            t.child_frame_id          = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z    = q_z
            t.transform.rotation.w    = q_w
            self.tf_broadcaster.sendTransform(t)

    # ─────────────────────────────────────────────────────────────────────────
    def status_callback(self):
        msg = String()
        msg.data = (
            f'x={self.x:.3f} y={self.y:.3f} '
            f'th={math.degrees(self.th):.1f}°'
        )
        self.pub_status.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
