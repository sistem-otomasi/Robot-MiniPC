"""
RoboTwin Kinematics - Differential Drive
Two-wheel differential drive robot kinematics
"""

import math
from typing import Tuple

from robot_example.kinematics.base import (
    KinematicsBase,
    RobotPhysicalParams,
    RobotVelocity,
    WheelVelocities
)


class DifferentialDriveKinematics(KinematicsBase):
    """
    Differential drive kinematics (2-wheel robot)
    
    Left wheel: wheel_velocities[0]
    Right wheel: wheel_velocities[1]
    
    Coordinate frame:
    - X: forward
    - Y: left
    - Z: up
    - Angular: counter-clockwise positive
    """
    
    def __init__(self, params: RobotPhysicalParams):
        super().__init__(params)
        self.wheel_separation = params.wheel_separation
        self.wheel_radius = params.wheel_radius
        
    def inverse_kinematics(self, velocity: RobotVelocity) -> WheelVelocities:
        """
        Convert robot velocity to wheel velocities
        
        For differential drive:
        v_left = (v - ω * L/2) / r
        v_right = (v + ω * L/2) / r
        
        Where:
        - v: linear velocity (m/s)
        - ω: angular velocity (rad/s)
        - L: wheel separation (m)
        - r: wheel radius (m)
        """
        v = velocity.linear_x
        omega = velocity.angular_z
        
        # Calculate linear velocities at each wheel
        v_left = v - (omega * self.wheel_separation / 2)
        v_right = v + (omega * self.wheel_separation / 2)
        
        # Convert to angular velocities
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius
        
        return WheelVelocities([omega_left, omega_right])
    
    def forward_kinematics(self, wheel_velocities: WheelVelocities) -> RobotVelocity:
        """
        Convert wheel velocities to robot velocity
        
        For differential drive:
        v = (v_left + v_right) * r / 2
        ω = (v_right - v_left) * r / L
        """
        omega_left = wheel_velocities[0]
        omega_right = wheel_velocities[1]
        
        # Convert to linear velocities
        v_left = omega_left * self.wheel_radius
        v_right = omega_right * self.wheel_radius
        
        # Calculate robot velocity
        linear_x = (v_left + v_right) / 2
        angular_z = (v_right - v_left) / self.wheel_separation
        
        return RobotVelocity(linear_x=linear_x, angular_z=angular_z)
    
    def compute_odometry_delta(
        self,
        left_delta: float,
        right_delta: float,
        current_theta: float
    ) -> Tuple[float, float, float]:
        """
        Compute odometry change from wheel encoder deltas
        
        Args:
            left_delta: Left wheel position change (radians)
            right_delta: Right wheel position change (radians)
            current_theta: Current heading (radians)
            
        Returns:
            Tuple of (dx, dy, dtheta) in robot frame
        """
        # Convert to distance traveled by each wheel
        left_dist = left_delta * self.wheel_radius
        right_dist = right_delta * self.wheel_radius
        
        # Calculate distance and angle change
        distance = (left_dist + right_dist) / 2
        dtheta = (right_dist - left_dist) / self.wheel_separation
        
        # Calculate position change using midpoint integration
        # This is more accurate than simple Euler integration
        theta_mid = current_theta + dtheta / 2
        dx = distance * math.cos(theta_mid)
        dy = distance * math.sin(theta_mid)
        
        return dx, dy, dtheta
    
    def turning_radius(self, velocity: RobotVelocity) -> float:
        """
        Calculate turning radius for given velocity
        
        Returns:
            Turning radius in meters (inf for straight motion)
        """
        if abs(velocity.angular_z) < 1e-6:
            return float('inf')
        return velocity.linear_x / velocity.angular_z
    
    def instant_center_of_rotation(
        self,
        velocity: RobotVelocity
    ) -> Tuple[float, float]:
        """
        Calculate instant center of rotation (ICR)
        
        Returns:
            (x, y) position of ICR relative to robot center
        """
        if abs(velocity.angular_z) < 1e-6:
            # Moving straight, ICR at infinity
            return float('inf'), 0.0
        
        r = velocity.linear_x / velocity.angular_z
        return 0.0, r  # ICR is on the y-axis (perpendicular to motion)
    
    @property
    def num_wheels(self) -> int:
        return 2
