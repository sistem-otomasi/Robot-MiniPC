"""
ROS 2 Node - Hardware Status Monitor
Monitoring sensor dan hardware status dari robotwin
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image
from std_msgs.msg import Float32, Float32MultiArray
import json

from robot_example.hardware.sensors import SensorManager
from robot_example.utils.logger import get_logger

logger = get_logger(__name__)


class HardwareMonitorNode(Node):
    """Monitor status hardware dan sensors"""

    def __init__(self):
        super().__init__('hardware_monitor')
        
        # Publishers untuk berbagai sensor
        self.motor_state_pub = self.create_publisher(Float32MultiArray, 'hardware/motor_speeds', 10)
        self.battery_pub = self.create_publisher(Float32, 'hardware/battery', 10)
        self.temperature_pub = self.create_publisher(Float32, 'hardware/temperature', 10)
        self.status_pub = self.create_publisher(Float32MultiArray, 'hardware/status', 10)
        
        # Timer untuk update
        self.timer = self.create_timer(0.5, self.update_callback)
        
        self.counter = 0
        self.get_logger().info('Hardware Monitor Node initialized')
        
    def update_callback(self):
        """Update hardware status"""
        self.counter += 1
        
        # Mock data untuk testing
        motor_speeds = Float32MultiArray()
        motor_speeds.data = [float(self.counter % 100), float((self.counter + 25) % 100),
                            float((self.counter + 50) % 100), float((self.counter + 75) % 100)]
        self.motor_state_pub.publish(motor_speeds)
        
        # Battery status
        battery = Float32()
        battery.data = 75.0 + (self.counter % 25) * 0.1
        self.battery_pub.publish(battery)
        
        # Temperature
        temp = Float32()
        temp.data = 35.0 + (self.counter % 10) * 0.1
        self.temperature_pub.publish(temp)
        
        # General status
        status = Float32MultiArray()
        status.data = [
            float(self.counter % 100),  # Uptime
            75.0,  # Battery
            35.0,  # Temperature
            1.0 if self.counter % 2 == 0 else 0.0  # Status flag
        ]
        self.status_pub.publish(status)
        
        if self.counter % 10 == 0:
            self.get_logger().info(f'Hardware status published (cycle {self.counter})')


def main(args=None):
    rclpy.init(args=args)
    node = HardwareMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
