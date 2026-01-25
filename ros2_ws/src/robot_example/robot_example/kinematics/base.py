"""
RoboTwin Kinematics - Base Classes
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Any, List, Tuple
import math


@dataclass
class RobotVelocity:
    """Robot velocity in body frame (Twist)"""
    linear_x: float = 0.0   # m/s forward
    linear_y: float = 0.0   # m/s left (for omni/mecanum)
    angular_z: float = 0.0  # rad/s counter-clockwise
    
    def to_dict(self) -> Dict[str, float]:
        return {
            "linear_x": self.linear_x,
            "linear_y": self.linear_y,
            "angular_z": self.angular_z
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "RobotVelocity":
        return cls(
            linear_x=data.get("linear_x", data.get("linear", 0.0)),
            linear_y=data.get("linear_y", 0.0),
            angular_z=data.get("angular_z", data.get("angular", 0.0))
        )


@dataclass
class WheelVelocities:
    """Wheel angular velocities (rad/s)"""
    velocities: List[float]
    
    def __getitem__(self, index: int) -> float:
        return self.velocities[index]
    
    def __len__(self) -> int:
        return len(self.velocities)
    
    def to_list(self) -> List[float]:
        return self.velocities.copy()
    
    def to_dict(self) -> Dict[str, float]:
        return {f"wheel_{i}": v for i, v in enumerate(self.velocities)}


@dataclass
class RobotPhysicalParams:
    """Physical parameters of the robot"""
    wheel_radius: float = 0.05          # meters
    wheel_base: float = 0.3             # meters (front to rear)
    wheel_separation: float = 0.3       # meters (left to right)
    max_linear_velocity: float = 1.0    # m/s
    max_angular_velocity: float = 2.0   # rad/s
    
    def to_dict(self) -> Dict[str, float]:
        return {
            "wheel_radius": self.wheel_radius,
            "wheel_base": self.wheel_base,
            "wheel_separation": self.wheel_separation,
            "max_linear_velocity": self.max_linear_velocity,
            "max_angular_velocity": self.max_angular_velocity
        }


class KinematicsBase(ABC):
    """
    Base class for robot kinematics
    Handles forward and inverse kinematics for different drive configurations
    """
    
    def __init__(self, params: RobotPhysicalParams):
        self.params = params
        
    @abstractmethod
    def inverse_kinematics(self, velocity: RobotVelocity) -> WheelVelocities:
        """
        Convert robot velocity (Twist) to wheel velocities
        
        Args:
            velocity: Robot velocity in body frame
            
        Returns:
            Wheel angular velocities in rad/s
        """
        pass
    
    @abstractmethod
    def forward_kinematics(self, wheel_velocities: WheelVelocities) -> RobotVelocity:
        """
        Convert wheel velocities to robot velocity (Twist)
        
        Args:
            wheel_velocities: Wheel angular velocities in rad/s
            
        Returns:
            Robot velocity in body frame
        """
        pass
    
    def clamp_velocity(self, velocity: RobotVelocity) -> RobotVelocity:
        """Clamp velocity to physical limits"""
        return RobotVelocity(
            linear_x=max(-self.params.max_linear_velocity, 
                        min(self.params.max_linear_velocity, velocity.linear_x)),
            linear_y=max(-self.params.max_linear_velocity,
                        min(self.params.max_linear_velocity, velocity.linear_y)),
            angular_z=max(-self.params.max_angular_velocity,
                         min(self.params.max_angular_velocity, velocity.angular_z))
        )
    
    def wheel_velocity_to_linear(self, angular_velocity: float) -> float:
        """Convert wheel angular velocity to linear velocity"""
        return angular_velocity * self.params.wheel_radius
    
    def linear_to_wheel_velocity(self, linear_velocity: float) -> float:
        """Convert linear velocity to wheel angular velocity"""
        return linear_velocity / self.params.wheel_radius
    
    @property
    def num_wheels(self) -> int:
        """Number of wheels"""
        return 2  # Default for differential drive
