"""
RoboTwin Kinematics - Omni Drive
Three-wheel omni drive robot kinematics
"""

import math
from typing import Tuple

from robotwin.kinematics.base import (
    KinematicsBase,
    RobotPhysicalParams,
    RobotVelocity,
    WheelVelocities
)


class OmniDriveKinematics(KinematicsBase):
    """
    Three-wheel omni drive kinematics (holonomic robot)
    
    Wheel arrangement (top view, 120 degrees apart):
    
           [0] Front
            |
           / \\
          /   \\
        [1]   [2]
        Left  Right
    
    Wheel angles from front:
    - Wheel 0: 0° (front)
    - Wheel 1: 120° (back left)
    - Wheel 2: 240° (back right)
    
    Each wheel has rollers perpendicular to wheel rotation
    """
    
    def __init__(self, params: RobotPhysicalParams, wheel_radius_from_center: float = None):
        super().__init__(params)
        self.wheel_radius = params.wheel_radius
        # Distance from robot center to wheel center
        self.robot_radius = wheel_radius_from_center or params.wheel_separation / 2
        
        # Wheel angles (radians)
        self.wheel_angles = [0.0, 2 * math.pi / 3, 4 * math.pi / 3]
        
        # Precompute sin/cos for efficiency
        self._sin_angles = [math.sin(a) for a in self.wheel_angles]
        self._cos_angles = [math.cos(a) for a in self.wheel_angles]
        
    def inverse_kinematics(self, velocity: RobotVelocity) -> WheelVelocities:
        """
        Convert robot velocity to wheel velocities
        
        For 3-wheel omni drive:
        ω_i = (1/r) * [-sin(θ_i) * vx + cos(θ_i) * vy + R * ωz]
        
        Where:
        - r: wheel radius
        - R: robot radius (distance to wheel)
        - θ_i: wheel angle from forward direction
        """
        vx = velocity.linear_x
        vy = velocity.linear_y
        omega_z = velocity.angular_z
        
        wheel_velocities = []
        for i in range(3):
            omega = (
                -self._sin_angles[i] * vx +
                self._cos_angles[i] * vy +
                self.robot_radius * omega_z
            ) / self.wheel_radius
            wheel_velocities.append(omega)
            
        return WheelVelocities(wheel_velocities)
    
    def forward_kinematics(self, wheel_velocities: WheelVelocities) -> RobotVelocity:
        """
        Convert wheel velocities to robot velocity
        
        Using pseudo-inverse of the Jacobian matrix
        """
        # Build the Jacobian matrix
        # J = [[-sin(θ0), cos(θ0), R],
        #      [-sin(θ1), cos(θ1), R],
        #      [-sin(θ2), cos(θ2), R]]
        
        # For 120-degree symmetric arrangement, the inverse is well-defined
        # Using the analytical solution
        
        r = self.wheel_radius
        R = self.robot_radius
        
        omega_0 = wheel_velocities[0]
        omega_1 = wheel_velocities[1]
        omega_2 = wheel_velocities[2]
        
        # Analytical solution for 3-wheel omni at 120 degrees
        # vx = (2/3) * r * (-omega_0 * sin(0) - omega_1 * sin(120) - omega_2 * sin(240))
        # vy = (2/3) * r * (omega_0 * cos(0) + omega_1 * cos(120) + omega_2 * cos(240))
        # ωz = (1/3) * r / R * (omega_0 + omega_1 + omega_2)
        
        vx = (2.0 / 3.0) * r * (
            -omega_0 * self._sin_angles[0] -
            omega_1 * self._sin_angles[1] -
            omega_2 * self._sin_angles[2]
        )
        
        vy = (2.0 / 3.0) * r * (
            omega_0 * self._cos_angles[0] +
            omega_1 * self._cos_angles[1] +
            omega_2 * self._cos_angles[2]
        )
        
        omega_z = (1.0 / 3.0) * r / R * (omega_0 + omega_1 + omega_2)
        
        return RobotVelocity(linear_x=vx, linear_y=vy, angular_z=omega_z)
    
    def compute_odometry_delta(
        self,
        wheel_deltas: Tuple[float, float, float],
        current_theta: float
    ) -> Tuple[float, float, float]:
        """
        Compute odometry change from wheel encoder deltas
        
        Args:
            wheel_deltas: Wheel position changes (0, 1, 2) in radians
            current_theta: Current heading (radians)
            
        Returns:
            Tuple of (dx, dy, dtheta) in world frame
        """
        # Convert deltas to velocity-like terms
        wheel_vels = WheelVelocities(list(wheel_deltas))
        robot_delta = self.forward_kinematics(wheel_vels)
        
        # Transform to world frame
        dtheta = robot_delta.angular_z
        theta_mid = current_theta + dtheta / 2
        
        cos_theta = math.cos(theta_mid)
        sin_theta = math.sin(theta_mid)
        
        dx_world = robot_delta.linear_x * cos_theta - robot_delta.linear_y * sin_theta
        dy_world = robot_delta.linear_x * sin_theta + robot_delta.linear_y * cos_theta
        
        return dx_world, dy_world, dtheta
    
    def clamp_velocity(self, velocity: RobotVelocity) -> RobotVelocity:
        """Clamp velocity considering holonomic motion"""
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
        return 3


class FourWheelOmniKinematics(KinematicsBase):
    """
    Four-wheel omni drive kinematics
    
    Wheel arrangement (90 degrees apart):
         [0]
          |
    [3]--+--[1]
          |
         [2]
    """
    
    def __init__(self, params: RobotPhysicalParams, wheel_radius_from_center: float = None):
        super().__init__(params)
        self.wheel_radius = params.wheel_radius
        self.robot_radius = wheel_radius_from_center or params.wheel_separation / 2
        
        # Wheel angles (radians) - 0, 90, 180, 270 degrees
        self.wheel_angles = [0.0, math.pi/2, math.pi, 3*math.pi/2]
        
    def inverse_kinematics(self, velocity: RobotVelocity) -> WheelVelocities:
        """Convert robot velocity to wheel velocities"""
        vx = velocity.linear_x
        vy = velocity.linear_y
        omega_z = velocity.angular_z
        
        r = self.wheel_radius
        R = self.robot_radius
        
        # For 4-wheel omni at 90 degrees:
        # Front: moves with vy, rotation adds R*omega_z
        omega_0 = (vy + R * omega_z) / r
        # Right: moves with -vx, rotation adds R*omega_z
        omega_1 = (-vx + R * omega_z) / r
        # Rear: moves with -vy, rotation adds R*omega_z
        omega_2 = (-vy + R * omega_z) / r
        # Left: moves with vx, rotation adds R*omega_z
        omega_3 = (vx + R * omega_z) / r
        
        return WheelVelocities([omega_0, omega_1, omega_2, omega_3])
    
    def forward_kinematics(self, wheel_velocities: WheelVelocities) -> RobotVelocity:
        """Convert wheel velocities to robot velocity"""
        r = self.wheel_radius
        R = self.robot_radius
        
        omega_0 = wheel_velocities[0]  # Front
        omega_1 = wheel_velocities[1]  # Right
        omega_2 = wheel_velocities[2]  # Rear
        omega_3 = wheel_velocities[3]  # Left
        
        vx = (omega_3 - omega_1) * r / 2
        vy = (omega_0 - omega_2) * r / 2
        omega_z = (omega_0 + omega_1 + omega_2 + omega_3) * r / (4 * R)
        
        return RobotVelocity(linear_x=vx, linear_y=vy, angular_z=omega_z)
    
    @property
    def num_wheels(self) -> int:
        return 4
