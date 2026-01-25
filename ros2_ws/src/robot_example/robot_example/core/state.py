"""
RoboTwin Robot Core - State Management
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List
from datetime import datetime
import math


class RobotStatus(Enum):
    """Robot status enumeration"""
    INITIALIZING = "initializing"
    READY = "ready"
    RUNNING = "running"
    PAUSED = "paused"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"
    CHARGING = "charging"
    SHUTDOWN = "shutdown"


@dataclass
class Pose2D:
    """2D Pose representation"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    
    @property
    def theta_sin(self) -> float:
        return math.sin(self.theta / 2)
    
    @property
    def theta_cos(self) -> float:
        return math.cos(self.theta / 2)
    
    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "theta": self.theta}


@dataclass
class Velocity2D:
    """2D Velocity representation"""
    linear: float = 0.0
    angular: float = 0.0
    
    def to_dict(self) -> Dict[str, float]:
        return {"linear": self.linear, "angular": self.angular}


@dataclass 
class Odometry:
    """Odometry state"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    timestamp: float = 0.0
    
    @property
    def theta_sin(self) -> float:
        return math.sin(self.theta / 2)
    
    @property
    def theta_cos(self) -> float:
        return math.cos(self.theta / 2)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "timestamp": self.timestamp
        }


@dataclass
class BatteryState:
    """Battery state"""
    voltage: float = 0.0
    current: float = 0.0
    percentage: float = 0.0
    charging: bool = False
    temperature: Optional[float] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "voltage": self.voltage,
            "current": self.current,
            "percentage": self.percentage,
            "charging": self.charging,
            "temperature": self.temperature
        }


@dataclass
class IMUState:
    """IMU sensor state"""
    # Orientation (quaternion)
    orientation_x: float = 0.0
    orientation_y: float = 0.0
    orientation_z: float = 0.0
    orientation_w: float = 1.0
    
    # Angular velocity (rad/s)
    angular_velocity_x: float = 0.0
    angular_velocity_y: float = 0.0
    angular_velocity_z: float = 0.0
    
    # Linear acceleration (m/s²)
    linear_acceleration_x: float = 0.0
    linear_acceleration_y: float = 0.0
    linear_acceleration_z: float = 0.0
    
    timestamp: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "orientation": {
                "x": self.orientation_x,
                "y": self.orientation_y,
                "z": self.orientation_z,
                "w": self.orientation_w
            },
            "angular_velocity": {
                "x": self.angular_velocity_x,
                "y": self.angular_velocity_y,
                "z": self.angular_velocity_z
            },
            "linear_acceleration": {
                "x": self.linear_acceleration_x,
                "y": self.linear_acceleration_y,
                "z": self.linear_acceleration_z
            },
            "timestamp": self.timestamp
        }


@dataclass
class LaserScan:
    """LIDAR scan data"""
    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    time_increment: float = 0.0
    scan_time: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    ranges: List[float] = field(default_factory=list)
    intensities: List[float] = field(default_factory=list)
    timestamp: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "angle_min": self.angle_min,
            "angle_max": self.angle_max,
            "angle_increment": self.angle_increment,
            "time_increment": self.time_increment,
            "scan_time": self.scan_time,
            "range_min": self.range_min,
            "range_max": self.range_max,
            "ranges": self.ranges,
            "intensities": self.intensities,
            "timestamp": self.timestamp
        }


@dataclass
class RobotState:
    """Complete robot state"""
    robot_id: str
    name: str
    robot_type: str
    status: RobotStatus = RobotStatus.INITIALIZING
    
    # Pose and velocity
    pose: Pose2D = field(default_factory=Pose2D)
    velocity: Velocity2D = field(default_factory=Velocity2D)
    odometry: Odometry = field(default_factory=Odometry)
    
    # Sensors
    battery: Optional[BatteryState] = None
    imu: Optional[IMUState] = None
    last_scan: Optional[LaserScan] = None
    
    # Navigation
    goal: Optional[Pose2D] = None
    path: List[Pose2D] = field(default_factory=list)
    
    # Errors
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    
    # Timestamps
    last_update: datetime = field(default_factory=datetime.now)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "robotId": self.robot_id,
            "name": self.name,
            "type": self.robot_type,
            "status": self.status.value,
            "pose": self.pose.to_dict() if self.pose else None,
            "velocity": self.velocity.to_dict() if self.velocity else None,
            "odometry": self.odometry.to_dict() if self.odometry else None,
            "battery": self.battery.to_dict() if self.battery else None,
            "imu": self.imu.to_dict() if self.imu else None,
            "goal": self.goal.to_dict() if self.goal else None,
            "errors": self.errors,
            "warnings": self.warnings,
            "lastUpdate": self.last_update.isoformat()
        }
