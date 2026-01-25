"""
ROS 2 Node - Kinematics Processor
Menghitung dan mempublikasi kinematics dari robot
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, TwistStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import math
import time

from robot_example.kinematics.differential import DifferentialDriveKinematics
from robot_example.utils.logger import get_logger

logger = get_logger(__name__)


class KinematicsNode(Node):
    """ROS 2 Node untuk perhitungan kinematics"""

    def __init__(self):
        super().__init__('kinematics_processor')
        
        # Parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.3)
        
        wheel_radius = self.get_parameter('wheel_radius').value
        wheel_base = self.get_parameter('wheel_base').value
        
        # Initialize kinematics
        self.kinematics = DifferentialDriveKinematics(wheel_radius, wheel_base)
        
        # Publishers
        self.odometry_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_motors_pub = self.create_publisher(Float32MultiArray, 'cmd_motors', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.motor_state_sub = self.create_subscription(
            Float32MultiArray, 'hardware/motor_speeds', self.motor_state_callback, 10
        )
        
        # State
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0
        self.last_time = time.time()
        
        # Timer untuk update odometry
        self.timer = self.create_timer(0.05, self.update_odometry)
        
        self.get_logger().info('Kinematics Node initialized')
        
    def cmd_vel_callback(self, msg: Twist):
        """Convert velocity command to motor commands"""
        vx = msg.linear.x
        wz = msg.angular.z
        
        # Forward kinematics untuk mendapatkan motor speeds
        motor_speeds = self.kinematics.inverse(vx, wz)
        
        # Publish motor commands
        cmd = Float32MultiArray()
        cmd.data = motor_speeds
        self.cmd_motors_pub.publish(cmd)
        
        self.get_logger().debug(f'Cmd vel: vx={vx:.3f}, wz={wz:.3f} -> motors={motor_speeds}')
        
    def motor_state_callback(self, msg: Float32MultiArray):
        """Process motor state for odometry"""
        # Motor speeds dari hardware
        motor_speeds = msg.data
        self.motor_states = motor_speeds
        
    def update_odometry(self):
        """Update odometry dari motor states"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Buat odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        
        # Orientation (simplified)
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        # Velocity (placeholder)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0
        
        self.odometry_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
