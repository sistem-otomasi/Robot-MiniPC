from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mqtt_broker',
            default_value='localhost',
            description='MQTT Broker address'
        ),
        DeclareLaunchArgument(
            'mqtt_port',
            default_value='1883',
            description='MQTT Broker port'
        ),
        Node(
            package='embedded_bridge',
            executable='embedded_bridge_node',
            name='embedded_bridge',
            parameters=[
                {'mqtt_broker': LaunchConfiguration('mqtt_broker')},
                {'mqtt_port': LaunchConfiguration('mqtt_port')},
                {'mqtt_qos': 1},
            ],
            output='screen'
        ),
    ])
