"""
ROS 2 Node - Configuration Manager
Mengelola konfigurasi robot melalui ROS 2 services
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
import json
from pathlib import Path

from robot_example.utils.config import load_config, RobotConfig
from robot_example.utils.logger import get_logger

logger = get_logger(__name__)


class ConfigManagerNode(Node):
    """Manage robot configuration via ROS 2"""

    def __init__(self):
        super().__init__('config_manager')
        
        self.declare_parameter('config_path', 'config/robot_config.yaml')
        self.config_path = self.get_parameter('config_path').value
        
        # Load configuration
        try:
            self.config = load_config(self.config_path)
            self.get_logger().info(f'Configuration loaded from {self.config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            self.config = None
        
        # Publishers
        self.config_pub = self.create_publisher(String, 'robot/config', 10)
        
        # Services
        self.create_service(Trigger, 'robot/reload_config', self.reload_config_callback)
        self.create_service(Trigger, 'robot/get_config', self.get_config_callback)
        
        # Timer untuk publish config
        self.timer = self.create_timer(5.0, self.publish_config)
        
        self.get_logger().info('Config Manager Node initialized')
        
    def reload_config_callback(self, request, response):
        """Reload configuration from file"""
        try:
            self.config = load_config(self.config_path)
            self.get_logger().info('Configuration reloaded successfully')
            response.success = True
            response.message = 'Configuration reloaded'
        except Exception as e:
            self.get_logger().error(f'Failed to reload config: {e}')
            response.success = False
            response.message = f'Error: {str(e)}'
        return response
        
    def get_config_callback(self, request, response):
        """Get current configuration"""
        if self.config:
            response.success = True
            response.message = 'Configuration retrieved'
        else:
            response.success = False
            response.message = 'Configuration not loaded'
        return response
        
    def publish_config(self):
        """Publish current configuration"""
        if self.config:
            msg = String()
            msg.data = json.dumps({
                'robot_id': self.config.robot.id,
                'robot_name': self.config.robot.name,
                'robot_type': self.config.robot.type,
                'version': '1.0.0'
            })
            self.config_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConfigManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
