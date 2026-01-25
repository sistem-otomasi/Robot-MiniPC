"""
RoboTwin Kinematics - Odometry Calculator
Computes robot odometry from wheel encoders with filtering
"""

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List, Tuple
from enum import Enum

from robot_example.kinematics.base import (
    KinematicsBase,
    RobotPhysicalParams,
    RobotVelocity,
    WheelVelocities
)
from robot_example.kinematics.differential import DifferentialDriveKinematics
from robot_example.kinematics.mecanum import MecanumDriveKinematics
from robot_example.kinematics.omni import OmniDriveKinematics
from robot_example.control.filters import LowPassFilter, KalmanFilter1D


class DriveType(Enum):
    DIFFERENTIAL = "differential"
    MECANUM = "mecanum"
    OMNI = "omni"
    ACKERMANN = "ackermann"


@dataclass
class OdometryState:
    """Complete odometry state"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    timestamp: float = field(default_factory=time.time)
    
    # Covariance (for EKF integration)
    covariance: List[float] = field(default_factory=lambda: [0.0] * 36)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "timestamp": self.timestamp
        }
    
    def reset(self) -> None:
        """Reset odometry to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0


class OdometryCalculator:
    """
    Odometry calculator with filtering
    
    Supports:
    - Multiple drive types (differential, mecanum, omni)
    - Encoder fusion
    - Velocity filtering
    - IMU integration (optional)
    """
    
    def __init__(
        self,
        drive_type: DriveType,
        params: RobotPhysicalParams,
        use_velocity_filter: bool = True,
        use_kalman: bool = False
    ):
        self.drive_type = drive_type
        self.params = params
        self.state = OdometryState()
        
        # Create kinematics model
        self.kinematics: KinematicsBase = self._create_kinematics()
        
        # Previous encoder values
        self._prev_encoders: Optional[List[float]] = None
        self._prev_time: Optional[float] = None
        
        # Velocity filters
        self.use_velocity_filter = use_velocity_filter
        self._linear_filter = LowPassFilter(cutoff_freq=5.0, sample_rate=50.0)
        self._angular_filter = LowPassFilter(cutoff_freq=5.0, sample_rate=50.0)
        
        # Kalman filters for position (optional)
        self.use_kalman = use_kalman
        if use_kalman:
            self._x_kalman = KalmanFilter1D(
                process_noise=0.01,
                measurement_noise=0.1
            )
            self._y_kalman = KalmanFilter1D(
                process_noise=0.01,
                measurement_noise=0.1
            )
            self._theta_kalman = KalmanFilter1D(
                process_noise=0.01,
                measurement_noise=0.05
            )
            
    def _create_kinematics(self) -> KinematicsBase:
        """Create appropriate kinematics model"""
        if self.drive_type == DriveType.DIFFERENTIAL:
            return DifferentialDriveKinematics(self.params)
        elif self.drive_type == DriveType.MECANUM:
            return MecanumDriveKinematics(self.params)
        elif self.drive_type == DriveType.OMNI:
            return OmniDriveKinematics(self.params)
        else:
            # Default to differential
            return DifferentialDriveKinematics(self.params)
    
    def update(
        self,
        encoder_values: List[float],
        timestamp: Optional[float] = None,
        imu_angular_velocity: Optional[float] = None
    ) -> OdometryState:
        """
        Update odometry from encoder readings
        
        Args:
            encoder_values: Current encoder positions in radians
            timestamp: Current timestamp (uses time.time() if not provided)
            imu_angular_velocity: Optional IMU angular velocity for fusion
            
        Returns:
            Updated OdometryState
        """
        current_time = timestamp or time.time()
        
        if self._prev_encoders is None or self._prev_time is None:
            # First update - just store values
            self._prev_encoders = encoder_values.copy()
            self._prev_time = current_time
            self.state.timestamp = current_time
            return self.state
        
        # Calculate time delta
        dt = current_time - self._prev_time
        if dt <= 0:
            return self.state
        
        # Calculate encoder deltas
        encoder_deltas = [
            curr - prev
            for curr, prev in zip(encoder_values, self._prev_encoders)
        ]
        
        # Compute odometry delta based on drive type
        if self.drive_type == DriveType.DIFFERENTIAL:
            dx, dy, dtheta = self.kinematics.compute_odometry_delta(
                encoder_deltas[0],  # Left
                encoder_deltas[1],  # Right
                self.state.theta
            )
        elif self.drive_type == DriveType.MECANUM:
            dx, dy, dtheta = self.kinematics.compute_odometry_delta(
                tuple(encoder_deltas[:4]),
                self.state.theta
            )
        elif self.drive_type == DriveType.OMNI:
            dx, dy, dtheta = self.kinematics.compute_odometry_delta(
                tuple(encoder_deltas[:3]),
                self.state.theta
            )
        else:
            dx, dy, dtheta = 0.0, 0.0, 0.0
        
        # Fuse with IMU if available
        if imu_angular_velocity is not None:
            # Complementary filter for angular velocity
            alpha = 0.98
            dtheta = alpha * dtheta + (1 - alpha) * (imu_angular_velocity * dt)
        
        # Update position
        new_x = self.state.x + dx
        new_y = self.state.y + dy
        new_theta = self._normalize_angle(self.state.theta + dtheta)
        
        # Apply Kalman filter if enabled
        if self.use_kalman:
            new_x = self._x_kalman.update(new_x)
            new_y = self._y_kalman.update(new_y)
            new_theta = self._theta_kalman.update(new_theta)
        
        self.state.x = new_x
        self.state.y = new_y
        self.state.theta = new_theta
        
        # Calculate velocities
        linear_vel = math.sqrt(dx**2 + dy**2) / dt
        angular_vel = dtheta / dt
        
        # Apply velocity filtering
        if self.use_velocity_filter:
            linear_vel = self._linear_filter.update(linear_vel)
            angular_vel = self._angular_filter.update(angular_vel)
        
        self.state.linear_velocity = linear_vel
        self.state.angular_velocity = angular_vel
        self.state.timestamp = current_time
        
        # Store current values for next iteration
        self._prev_encoders = encoder_values.copy()
        self._prev_time = current_time
        
        return self.state
    
    def update_from_wheel_velocities(
        self,
        wheel_velocities: WheelVelocities,
        dt: float
    ) -> OdometryState:
        """
        Update odometry from wheel velocities (for simulation)
        
        Args:
            wheel_velocities: Current wheel angular velocities (rad/s)
            dt: Time step
            
        Returns:
            Updated OdometryState
        """
        # Convert to robot velocity
        robot_vel = self.kinematics.forward_kinematics(wheel_velocities)
        
        # Integrate
        theta_mid = self.state.theta + robot_vel.angular_z * dt / 2
        
        dx = robot_vel.linear_x * math.cos(theta_mid) * dt
        dy = robot_vel.linear_x * math.sin(theta_mid) * dt
        
        if self.drive_type in [DriveType.MECANUM, DriveType.OMNI]:
            # Account for lateral motion
            dx -= robot_vel.linear_y * math.sin(theta_mid) * dt
            dy += robot_vel.linear_y * math.cos(theta_mid) * dt
        
        self.state.x += dx
        self.state.y += dy
        self.state.theta = self._normalize_angle(
            self.state.theta + robot_vel.angular_z * dt
        )
        
        # Update velocities
        linear_vel = math.sqrt(robot_vel.linear_x**2 + robot_vel.linear_y**2)
        
        if self.use_velocity_filter:
            linear_vel = self._linear_filter.update(linear_vel)
            angular_vel = self._angular_filter.update(robot_vel.angular_z)
        else:
            angular_vel = robot_vel.angular_z
        
        self.state.linear_velocity = linear_vel
        self.state.angular_velocity = angular_vel
        self.state.timestamp = time.time()
        
        return self.state
    
    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """Reset odometry to specified pose"""
        self.state.x = x
        self.state.y = y
        self.state.theta = theta
        self.state.linear_velocity = 0.0
        self.state.angular_velocity = 0.0
        self._prev_encoders = None
        self._prev_time = None
        
        # Reset filters
        if self.use_velocity_filter:
            self._linear_filter.reset()
            self._angular_filter.reset()
        
        if self.use_kalman:
            self._x_kalman.reset(x)
            self._y_kalman.reset(y)
            self._theta_kalman.reset(theta)
    
    def get_pose(self) -> Tuple[float, float, float]:
        """Get current pose (x, y, theta)"""
        return self.state.x, self.state.y, self.state.theta
    
    def get_velocity(self) -> Tuple[float, float]:
        """Get current velocity (linear, angular)"""
        return self.state.linear_velocity, self.state.angular_velocity
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def create_odometry_calculator(
    drive_type: str,
    params: RobotPhysicalParams,
    **kwargs
) -> OdometryCalculator:
    """Factory function to create odometry calculator"""
    drive_type_enum = DriveType(drive_type.lower())
    return OdometryCalculator(drive_type_enum, params, **kwargs)
