"""
RoboTwin Hardware Module
Motor controllers, sensors, and GPIO handling
"""

from robot_example.hardware.motors import (
    MotorController,
    SerialMotorController,
    I2CMotorController,
    CANMotorController,
    create_motor_controller
)

from robot_example.hardware.sensors import (
    SensorManager,
    LidarSensor,
    CameraSensor,
    IMUSensor,
    EncoderSensor,
    BatterySensor
)

from robot_example.hardware.gpio import GPIOManager

__all__ = [
    "MotorController",
    "SerialMotorController", 
    "I2CMotorController",
    "CANMotorController",
    "create_motor_controller",
    "SensorManager",
    "LidarSensor",
    "CameraSensor",
    "IMUSensor",
    "EncoderSensor",
    "BatterySensor",
    "GPIOManager"
]
