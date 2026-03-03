#!/usr/bin/env python3
"""
GPS CUAV M3 Driver Node untuk ROS Humble
=========================================================================
Hardware : CUAV M3 GPS Module (NMEA over UART)
Protocol : NMEA 0183 – sentences: GGA, RMC
Default  : /dev/ttyUSB0 @ 9600 baud (bisa diganti via parameter)

Published Topics:
  /gps/fix          [sensor_msgs/NavSatFix]   – Posisi GPS utama
  /gps/velocity     [geometry_msgs/TwistStamped] – Kecepatan dari GPS
  /gps/status       [std_msgs/String]          – Status lock GPS

Usage:
  ros2 run robot_autonomous gps_driver_node.py
  ros2 run robot_autonomous gps_driver_node.py --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=9600
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

import serial
import time
import math
import re


class GPSDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('publish_rate', 10.0)    # Hz
        self.declare_parameter('min_satellites', 4)

        self.port         = self.get_parameter('port').value
        self.baudrate     = self.get_parameter('baudrate').value
        self.frame_id     = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.min_sat      = self.get_parameter('min_satellites').value

        # ── QoS ──────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers ────────────────────────────────────────────────
        self.pub_fix  = self.create_publisher(NavSatFix, '/gps/fix', qos)
        self.pub_vel  = self.create_publisher(TwistStamped, '/gps/velocity', qos)
        self.pub_stat = self.create_publisher(String, '/gps/status', 10)

        # ── Serial connection ─────────────────────────────────────────
        self.ser = None
        self.connect_serial()

        # ── Internal state ────────────────────────────────────────────
        self.lat      = 0.0
        self.lon      = 0.0
        self.alt      = 0.0
        self.speed    = 0.0   # m/s
        self.heading  = 0.0   # degrees
        self.fix_type = NavSatStatus.STATUS_NO_FIX
        self.num_sat  = 0
        self.hdop     = 99.9

        # ── Timer ─────────────────────────────────────────────────────
        period = 1.0 / self.publish_rate
        self.create_timer(period, self.timer_callback)
        self.get_logger().info(f'GPS CUAV M3 driver started on {self.port}@{self.baudrate}')

    # ─────────────────────────────────────────────────────────────────────────
    def connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.5
            )
            self.get_logger().info(f'Serial GPS terbuka: {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Gagal buka serial GPS: {e}')
            self.ser = None

    # ─────────────────────────────────────────────────────────────────────────
    def timer_callback(self):
        if self.ser is None:
            self.get_logger().warn('Serial GPS tidak tersedia, coba reconnect...')
            self.connect_serial()
            return

        try:
            # Baca semua line yang tersedia
            while self.ser.in_waiting > 0:
                raw = self.ser.readline()
                try:
                    line = raw.decode('ascii', errors='ignore').strip()
                except Exception:
                    continue
                self.parse_nmea(line)

            self.publish_data()

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            self.ser = None

    # ─────────────────────────────────────────────────────────────────────────
    def parse_nmea(self, sentence: str):
        """Parse NMEA 0183 sentences: GGA dan RMC"""
        if not sentence.startswith('$'):
            return

        # Strip checksum for parsing
        if '*' in sentence:
            sentence = sentence[:sentence.index('*')]

        parts = sentence.split(',')

        try:
            if parts[0] in ('$GPGGA', '$GNGGA'):
                self._parse_gga(parts)
            elif parts[0] in ('$GPRMC', '$GNRMC'):
                self._parse_rmc(parts)
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Parse error: {e} | sentence: {sentence}')

    def _parse_gga(self, p):
        """$GPGGA – fix data"""
        if len(p) < 10:
            return
        if not p[2] or not p[4]:
            return

        # Latitude: ddmm.mmmmm → decimal degrees
        raw_lat = float(p[2])
        deg_lat = int(raw_lat / 100)
        min_lat = raw_lat - deg_lat * 100
        self.lat = deg_lat + min_lat / 60.0
        if p[3] == 'S':
            self.lat = -self.lat

        # Longitude: dddmm.mmmmm → decimal degrees
        raw_lon = float(p[4])
        deg_lon = int(raw_lon / 100)
        min_lon = raw_lon - deg_lon * 100
        self.lon = deg_lon + min_lon / 60.0
        if p[5] == 'W':
            self.lon = -self.lon

        # Fix quality: 0=no fix, 1=GPS fix, 2=DGPS, 4=RTK fixed, 5=RTK float
        fix_q = int(p[6]) if p[6] else 0
        if fix_q == 0:
            self.fix_type = NavSatStatus.STATUS_NO_FIX
        elif fix_q in (1, 2):
            self.fix_type = NavSatStatus.STATUS_FIX
        elif fix_q in (3, 4, 5):
            self.fix_type = NavSatStatus.STATUS_GBAS_FIX

        self.num_sat = int(p[7]) if p[7] else 0
        self.hdop    = float(p[8]) if p[8] else 99.9
        self.alt     = float(p[9]) if p[9] else 0.0

    def _parse_rmc(self, p):
        """$GPRMC – speed & heading"""
        if len(p) < 9:
            return
        if p[2] != 'A':       # A=active, V=void
            return
        # Speed in knots → m/s
        if p[7]:
            self.speed = float(p[7]) * 0.514444
        # Course over ground (degrees)
        if p[8]:
            self.heading = float(p[8])

    # ─────────────────────────────────────────────────────────────────────────
    def publish_data(self):
        now = self.get_clock().now().to_msg()

        # NavSatFix ──────────────────────────────────────────────────────────
        fix_msg                        = NavSatFix()
        fix_msg.header.stamp           = now
        fix_msg.header.frame_id        = self.frame_id
        fix_msg.status.status          = self.fix_type
        fix_msg.status.service         = NavSatStatus.SERVICE_GPS
        fix_msg.latitude               = self.lat
        fix_msg.longitude              = self.lon
        fix_msg.altitude               = self.alt

        # Covariance diagonal (m²) berdasarkan HDOP
        cov = (self.hdop * 3.0) ** 2
        fix_msg.position_covariance    = [cov, 0, 0,
                                          0, cov, 0,
                                          0, 0, cov * 4.0]
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_fix.publish(fix_msg)

        # TwistStamped (velocity) ──────────────────────────────────────────
        vel_msg                  = TwistStamped()
        vel_msg.header.stamp     = now
        vel_msg.header.frame_id  = self.frame_id
        heading_rad              = math.radians(self.heading)
        vel_msg.twist.linear.x   = self.speed * math.cos(heading_rad)
        vel_msg.twist.linear.y   = self.speed * math.sin(heading_rad)
        vel_msg.twist.linear.z   = 0.0
        self.pub_vel.publish(vel_msg)

        # Status string ────────────────────────────────────────────────────
        status_str  = String()
        status_str.data = (
            f'fix={self.fix_type} sat={self.num_sat} '
            f'hdop={self.hdop:.1f} lat={self.lat:.6f} lon={self.lon:.6f} '
            f'alt={self.alt:.1f}m speed={self.speed:.2f}m/s'
        )
        self.pub_stat.publish(status_str)

        if self.num_sat < self.min_sat:
            self.get_logger().warn(
                f'Satelit hanya {self.num_sat} (min={self.min_sat}), akurasi rendah')


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = GPSDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
