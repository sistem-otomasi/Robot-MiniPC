"""
RoboTwin Kinematics - Mecanum Drive
Four-wheel mecanum drive robot kinematics
"""

import math
from typing import Tuple

from robotwin.kinematics.base import (
    KinematicsBase,
    RobotPhysicalParams,
    RobotVelocity,
    WheelVelocities
)


class MecanumDriveKinematics(KinematicsBase):
    """
    Mecanum drive kinematics (4-wheel holonomic robot)
    
    Wheel arrangement (top view):
       Front
    [0]     [1]
       \\   /
        \\ /
        / \\
       /   \\
    [2]     [3]
       Rear
    
    Wheel indices:
    - 0: Front Left
    - 1: Front Right
    - 2: Rear Left
    - 3: Rear Right
    
    Roller angle: 45 degrees from wheel axis
    """
    
    def __init__(self, params: RobotPhysicalParams):
        super().__init__(params)
        self.wheel_radius = params.wheel_radius
        self.wheel_separation = params.wheel_separation  # Left-right distance
        self.wheel_base = params.wheel_base  # Front-rear distance
        
        # Half distances for calculations
        self.lx = self.wheel_base / 2
        self.ly = self.wheel_separation / 2
        
    def inverse_kinematics(self, velocity: RobotVelocity) -> WheelVelocities:
        """
        Convert robot velocity to wheel velocities
        
        For mecanum drive:
        ω_fl = (vx - vy - (lx + ly) * ωz) / r
        ω_fr = (vx + vy + (lx + ly) * ωz) / r
        ω_rl = (vx + vy - (lx + ly) * ωz) / r
        ω_rr = (vx - vy + (lx + ly) * ωz) / r
        """
        vx = velocity.linear_x
        vy = velocity.linear_y
        omega = velocity.angular_z
        
        k = self.lx + self.ly
        r = self.wheel_radius
        
        omega_fl = (vx - vy - k * omega) / r
        omega_fr = (vx + vy + k * omega) / r
        omega_rl = (vx + vy - k * omega) / r
        omega_rr = (vx - vy + k * omega) / r
        
        return WheelVelocities([omega_fl, omega_fr, omega_rl, omega_rr])
    
    def forward_kinematics(self, wheel_velocities: WheelVelocities) -> RobotVelocity:
        """
        Convert wheel velocities to robot velocity
        
        For mecanum drive:
        vx = (ω_fl + ω_fr + ω_rl + ω_rr) * r / 4
        vy = (-ω_fl + ω_fr + ω_rl - ω_rr) * r / 4
        ωz = (-ω_fl + ω_fr - ω_rl + ω_rr) * r / (4 * (lx + ly))
        """
        omega_fl = wheel_velocities[0]
        omega_fr = wheel_velocities[1]
        omega_rl = wheel_velocities[2]
        omega_rr = wheel_velocities[3]
        
        r = self.wheel_radius
        k = self.lx + self.ly
        
        vx = (omega_fl + omega_fr + omega_rl + omega_rr) * r / 4
        vy = (-omega_fl + omega_fr + omega_rl - omega_rr) * r / 4
        omega_z = (-omega_fl + omega_fr - omega_rl + omega_rr) * r / (4 * k)
        
        return RobotVelocity(linear_x=vx, linear_y=vy, angular_z=omega_z)
    
    def compute_odometry_delta(
        self,
        wheel_deltas: Tuple[float, float, float, float],
        current_theta: float
    ) -> Tuple[float, float, float]:
        """
        Compute odometry change from wheel encoder deltas
        
        Args:
            wheel_deltas: Wheel position changes (fl, fr, rl, rr) in radians
            current_theta: Current heading (radians)
            
        Returns:
            Tuple of (dx, dy, dtheta) in world frame
        """
        d_fl, d_fr, d_rl, d_rr = wheel_deltas
        r = self.wheel_radius
        k = self.lx + self.ly
        
        # Calculate robot frame velocities
        dx_robot = (d_fl + d_fr + d_rl + d_rr) * r / 4
        dy_robot = (-d_fl + d_fr + d_rl - d_rr) * r / 4
        dtheta = (-d_fl + d_fr - d_rl + d_rr) * r / (4 * k)
        
        # Transform to world frame
        theta_mid = current_theta + dtheta / 2
        cos_theta = math.cos(theta_mid)
        sin_theta = math.sin(theta_mid)
        
        dx_world = dx_robot * cos_theta - dy_robot * sin_theta
        dy_world = dx_robot * sin_theta + dy_robot * cos_theta
        
        return dx_world, dy_world, dtheta
    
    def clamp_velocity(self, velocity: RobotVelocity) -> RobotVelocity:
        """Clamp velocity considering holonomic motion"""
        # Calculate combined linear velocity magnitude
        linear_mag = math.sqrt(velocity.linear_x**2 + velocity.linear_y**2)
        
        if linear_mag > self.params.max_linear_velocity:
            scale = self.params.max_linear_velocity / linear_mag
            return RobotVelocity(
                linear_x=velocity.linear_x * scale,
                linear_y=velocity.linear_y * scale,
                angular_z=max(-self.params.max_angular_velocity,
                             min(self.params.max_angular_velocity, velocity.angular_z))
            )
        
        return RobotVelocity(
            linear_x=velocity.linear_x,
            linear_y=velocity.linear_y,
            angular_z=max(-self.params.max_angular_velocity,
                         min(self.params.max_angular_velocity, velocity.angular_z))
        )
    
    @property
    def num_wheels(self) -> int:
        return 4
