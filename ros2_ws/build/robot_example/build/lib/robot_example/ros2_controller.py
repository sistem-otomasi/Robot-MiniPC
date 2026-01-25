"""
ROS 2 Node - Robot Controller Main
Mengintegrasikan robotwin dengan ROS 2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import Twist, Pose
import asyncio
import json
from pathlib import Path

from robot_example.core.robot import Robot
from robot_example.utils.logger import get_logger

logger = get_logger(__name__)


class RobotControllerNode(Node):
    """ROS 2 Node untuk mengontrol robot menggunakan robotwin"""

    def __init__(self):
        super().__init__('robot_controller')
        
        self.declare_parameter('config_path', 'config/robot_config.yaml')
        config_path = self.get_parameter('config_path').value
        
        # Initialize robotwin Robot
        self.robot = Robot(config_path)
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        # Publishers
        self.status_publisher = self.create_publisher(String, 'robot/status', 10)
        self.odometry_publisher = self.create_publisher(Pose, 'robot/odometry', 10)
        self.sensor_publisher = self.create_publisher(Float32MultiArray, 'robot/sensors', 10)
        
        # Subscribers
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        # Timer untuk update status
        self.timer = self.create_timer(0.1, self.update_callback)
        
        self.get_logger().info('Robot Controller Node initialized')
        
    async def async_initialize(self):
        """Initialize robot asynchronously"""
        try:
            await self.robot.initialize()
            self.get_logger().info('Robot initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize robot: {e}')
            
    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands from ROS 2"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        self.get_logger().info(f'Received cmd_vel: linear={linear_x}, angular={angular_z}')
        
        # Update robot target velocity
        if self.robot.ctx:
            self.robot.ctx.state.target_linear_velocity = linear_x
            self.robot.ctx.state.target_angular_velocity = angular_z
    
    def update_callback(self):
        """Periodic update callback"""
        if self.robot.ctx:
            # Publish status
            status_msg = String()
            status_msg.data = json.dumps({
                'robot_id': self.robot.ctx.state.robot_id,
                'status': self.robot.ctx.state.status.value,
                'battery': getattr(self.robot.ctx.state, 'battery_level', 0),
            })
            self.status_publisher.publish(status_msg)
            
            # Publish odometry
            odom_msg = Pose()
            odom_msg.position.x = self.robot.ctx.state.position_x
            odom_msg.position.y = self.robot.ctx.state.position_y
            self.odometry_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = RobotControllerNode()
    
    # Initialize robot
    try:
        node.loop.run_until_complete(node.async_initialize())
    except Exception as e:
        logger.error(f'Initialization failed: {e}')
        rclpy.shutdown()
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
