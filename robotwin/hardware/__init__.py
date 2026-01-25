"""
RoboTwin Hardware Module
Motor controllers, sensors, and GPIO handling
"""

from robotwin.hardware.motors import (
    MotorController,
    SerialMotorController,
    I2CMotorController,
    CANMotorController,
    create_motor_controller
)

from robotwin.hardware.sensors import (
    SensorManager,
    LidarSensor,
    CameraSensor,
    IMUSensor,
    EncoderSensor,
    BatterySensor
)

from robotwin.hardware.gpio import GPIOManager

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
