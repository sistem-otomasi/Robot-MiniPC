"""
RoboTwin Kinematics - Ackermann Steering
Car-like robot kinematics with Ackermann steering geometry
"""

import math
from typing import Tuple, Optional
from dataclasses import dataclass

from robotwin.kinematics.base import (
    KinematicsBase,
    RobotPhysicalParams,
    RobotVelocity,
    WheelVelocities
)


@dataclass
class AckermannParams:
    """Ackermann-specific parameters"""
    wheelbase: float = 0.3          # Distance between front and rear axles
    track_width: float = 0.25       # Distance between left and right wheels
    max_steering_angle: float = 0.5 # Maximum steering angle in radians (~30 degrees)
    min_turning_radius: float = 0.5 # Minimum turning radius in meters


@dataclass
class AckermannCommand:
    """Ackermann steering command"""
    speed: float = 0.0              # Linear speed (m/s)
    steering_angle: float = 0.0     # Front wheel steering angle (radians)
    
    def to_dict(self):
        return {"speed": self.speed, "steering_angle": self.steering_angle}


class AckermannKinematics(KinematicsBase):
    """
    Ackermann steering kinematics (car-like robot)
    
    Layout:
        [FL]----[FR]    Front wheels (steered)
          |      |
          |  L   |
          |      |
        [RL]----[RR]    Rear wheels (driven)
    
    Uses Ackermann geometry for proper turning without wheel slip
    """
    
    def __init__(
        self,
        params: RobotPhysicalParams,
        ackermann_params: Optional[AckermannParams] = None
    ):
        super().__init__(params)
        self.ackermann = ackermann_params or AckermannParams(
            wheelbase=params.wheel_base,
            track_width=params.wheel_separation
        )
        self.wheel_radius = params.wheel_radius
        
    def inverse_kinematics(self, velocity: RobotVelocity) -> WheelVelocities:
        """
        Convert robot velocity to wheel velocities
        
        Returns wheel velocities for [FL, FR, RL, RR]
        Note: This doesn't account for steering angles directly,
        use velocity_to_ackermann() for proper Ackermann commands
        """
        v = velocity.linear_x
        omega = velocity.angular_z
        
        # Approximate: for small steering angles
        # Rear wheels driven, front wheels free
        if abs(omega) < 1e-6:
            # Going straight
            omega_rear = v / self.wheel_radius
            return WheelVelocities([omega_rear, omega_rear, omega_rear, omega_rear])
        
        # Calculate turning radius
        R = v / omega if abs(omega) > 1e-6 else float('inf')
        L = self.ackermann.wheelbase
        W = self.ackermann.track_width / 2
        
        # Wheel speeds based on distance from ICR
        if abs(R) < float('inf'):
            # Inner and outer wheel speeds
            v_inner = omega * (R - W)
            v_outer = omega * (R + W)
            
            omega_left = v_inner / self.wheel_radius
            omega_right = v_outer / self.wheel_radius
            
            if R < 0:  # Turning right
                omega_left, omega_right = omega_right, omega_left
        else:
            omega_left = omega_right = v / self.wheel_radius
            
        return WheelVelocities([omega_left, omega_right, omega_left, omega_right])
    
    def forward_kinematics(self, wheel_velocities: WheelVelocities) -> RobotVelocity:
        """
        Convert wheel velocities to robot velocity
        Uses rear wheel velocities for speed calculation
        """
        # Use rear wheels (indices 2, 3)
        omega_rl = wheel_velocities[2]
        omega_rr = wheel_velocities[3]
        
        v_left = omega_rl * self.wheel_radius
        v_right = omega_rr * self.wheel_radius
        
        # Linear velocity is average of rear wheels
        v = (v_left + v_right) / 2
        
        # Angular velocity from differential
        W = self.ackermann.track_width
        omega = (v_right - v_left) / W
        
        return RobotVelocity(linear_x=v, angular_z=omega)
    
    def velocity_to_ackermann(self, velocity: RobotVelocity) -> AckermannCommand:
        """
        Convert robot velocity (twist) to Ackermann steering command
        
        Args:
            velocity: Robot velocity with linear_x and angular_z
            
        Returns:
            AckermannCommand with speed and steering_angle
        """
        v = velocity.linear_x
        omega = velocity.angular_z
        
        if abs(v) < 1e-6:
            # Stationary
            return AckermannCommand(speed=0.0, steering_angle=0.0)
        
        if abs(omega) < 1e-6:
            # Going straight
            return AckermannCommand(speed=v, steering_angle=0.0)
        
        # Calculate turning radius
        R = v / omega
        L = self.ackermann.wheelbase
        
        # Steering angle from turning radius
        # tan(delta) = L / R
        steering_angle = math.atan2(L, abs(R))
        
        # Sign based on turn direction
        if R < 0:
            steering_angle = -steering_angle
            
        # Clamp to max steering angle
        steering_angle = max(-self.ackermann.max_steering_angle,
                           min(self.ackermann.max_steering_angle, steering_angle))
        
        return AckermannCommand(speed=v, steering_angle=steering_angle)
    
    def ackermann_to_velocity(self, command: AckermannCommand) -> RobotVelocity:
        """
        Convert Ackermann command to robot velocity (twist)
        
        Args:
            command: AckermannCommand with speed and steering_angle
            
        Returns:
            RobotVelocity
        """
        v = command.speed
        delta = command.steering_angle
        L = self.ackermann.wheelbase
        
        if abs(delta) < 1e-6:
            # Going straight
            return RobotVelocity(linear_x=v, angular_z=0.0)
        
        # Angular velocity from steering angle
        # omega = v * tan(delta) / L
        omega = v * math.tan(delta) / L
        
        return RobotVelocity(linear_x=v, angular_z=omega)
    
    def get_wheel_steering_angles(
        self,
        steering_angle: float
    ) -> Tuple[float, float]:
        """
        Calculate individual front wheel steering angles
        using Ackermann geometry
        
        Args:
            steering_angle: Commanded steering angle (center/virtual wheel)
            
        Returns:
            (left_angle, right_angle) tuple
        """
        if abs(steering_angle) < 1e-6:
            return 0.0, 0.0
        
        L = self.ackermann.wheelbase
        W = self.ackermann.track_width
        
        # Turning radius of the virtual center wheel
        R = L / math.tan(abs(steering_angle))
        
        # Inner and outer wheel angles
        inner_angle = math.atan(L / (R - W/2))
        outer_angle = math.atan(L / (R + W/2))
        
        if steering_angle > 0:  # Turning left
            return inner_angle, outer_angle
        else:  # Turning right
            return -outer_angle, -inner_angle
    
    def compute_odometry_delta(
        self,
        rear_left_delta: float,
        rear_right_delta: float,
        steering_angle: float,
        current_theta: float
    ) -> Tuple[float, float, float]:
        """
        Compute odometry change from rear wheel encoders and steering angle
        
        Args:
            rear_left_delta: Rear left wheel position change (radians)
            rear_right_delta: Rear right wheel position change (radians)
            steering_angle: Current steering angle (radians)
            current_theta: Current heading (radians)
            
        Returns:
            Tuple of (dx, dy, dtheta) in world frame
        """
        r = self.wheel_radius
        L = self.ackermann.wheelbase
        W = self.ackermann.track_width
        
        # Distance traveled by rear wheels
        d_left = rear_left_delta * r
        d_right = rear_right_delta * r
        
        # Average distance
        d = (d_left + d_right) / 2
        
        if abs(steering_angle) < 1e-6:
            # Going straight
            dx = d * math.cos(current_theta)
            dy = d * math.sin(current_theta)
            dtheta = 0.0
        else:
            # Turning radius
            R = L / math.tan(steering_angle)
            
            # Change in heading
            dtheta = d / R
            
            # Position change using bicycle model
            theta_mid = current_theta + dtheta / 2
            dx = d * math.cos(theta_mid)
            dy = d * math.sin(theta_mid)
        
        return dx, dy, dtheta
    
    def turning_radius(self, steering_angle: float) -> float:
        """Calculate turning radius for given steering angle"""
        if abs(steering_angle) < 1e-6:
            return float('inf')
        return self.ackermann.wheelbase / math.tan(steering_angle)
    
    @property
    def num_wheels(self) -> int:
        return 4
