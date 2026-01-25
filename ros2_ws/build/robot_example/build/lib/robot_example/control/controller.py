"""
RoboTwin Control - Main Controller
High-level robot control coordination
"""

import asyncio
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Callable, List
from enum import Enum

from robot_example.utils.logger import get_logger
from robot_example.control.pid import PIDController, AnglePIDController
from robot_example.control.filters import LowPassFilter
from robot_example.kinematics.base import RobotVelocity, WheelVelocities, RobotPhysicalParams
from robot_example.kinematics.differential import DifferentialDriveKinematics
from robot_example.kinematics.mecanum import MecanumDriveKinematics
from robot_example.kinematics.omni import OmniDriveKinematics
from robot_example.kinematics.odometry import OdometryCalculator, DriveType

logger = get_logger(__name__)


class ControlMode(Enum):
    IDLE = "idle"
    TELEOP = "teleop"           # Manual control
    VELOCITY = "velocity"        # Velocity control
    POSITION = "position"        # Go to position
    HEADING = "heading"          # Maintain heading
    FOLLOW_PATH = "follow_path"  # Path following
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class ControlState:
    """Controller state"""
    mode: ControlMode = ControlMode.IDLE
    target_velocity: RobotVelocity = field(default_factory=RobotVelocity)
    target_position: Optional[tuple] = None  # (x, y, theta)
    target_heading: float = 0.0
    current_velocity: RobotVelocity = field(default_factory=RobotVelocity)
    wheel_velocities: WheelVelocities = field(default_factory=lambda: WheelVelocities([0.0, 0.0]))
    timestamp: float = field(default_factory=time.time)


class MainController:
    """
    Main robot controller
    
    Coordinates:
    - Velocity control (teleop)
    - Position control (navigation)
    - Heading control
    - Emergency stop
    """
    
    def __init__(
        self,
        config: Dict[str, Any],
        physical_params: RobotPhysicalParams
    ):
        self.config = config
        self.params = physical_params
        self.state = ControlState()
        
        # Determine drive type
        robot_type = config.get("robot", {}).get("type", "differential")
        self.drive_type = DriveType(robot_type)
        
        # Create kinematics
        self.kinematics = self._create_kinematics()
        
        # Create velocity controller
        self.velocity_controller = VelocityController(config, physical_params)
        
        # Create position controller
        self.position_controller = PositionController(config, physical_params)
        
        # Create heading controller
        self.heading_controller = HeadingController(config)
        
        # Control loop settings
        self.control_frequency = config.get("robot", {}).get("control_frequency", 50)
        self._control_dt = 1.0 / self.control_frequency
        
        # Velocity smoothing
        self._linear_filter = LowPassFilter(cutoff_freq=5.0, sample_rate=self.control_frequency)
        self._angular_filter = LowPassFilter(cutoff_freq=5.0, sample_rate=self.control_frequency)
        
        # Motor output callback
        self._motor_callback: Optional[Callable] = None
        
        # Control loop task
        self._running = False
        self._control_task: Optional[asyncio.Task] = None
        
    def _create_kinematics(self):
        """Create kinematics model based on drive type"""
        if self.drive_type == DriveType.DIFFERENTIAL:
            return DifferentialDriveKinematics(self.params)
        elif self.drive_type == DriveType.MECANUM:
            return MecanumDriveKinematics(self.params)
        elif self.drive_type == DriveType.OMNI:
            return OmniDriveKinematics(self.params)
        else:
            return DifferentialDriveKinematics(self.params)
    
    async def start(self) -> None:
        """Start control loop"""
        if self._running:
            return
            
        self._running = True
        self._control_task = asyncio.create_task(self._control_loop())
        logger.info(f"Controller started at {self.control_frequency}Hz")
        
    async def stop(self) -> None:
        """Stop control loop"""
        self._running = False
        
        if self._control_task:
            self._control_task.cancel()
            try:
                await self._control_task
            except asyncio.CancelledError:
                pass
                
        # Stop motors
        await self.emergency_stop()
        logger.info("Controller stopped")
        
    async def _control_loop(self) -> None:
        """Main control loop"""
        last_time = time.time()
        
        while self._running:
            try:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                
                # Execute control based on mode
                wheel_vels = await self._execute_control(dt)
                
                # Update state
                self.state.wheel_velocities = wheel_vels
                self.state.timestamp = current_time
                
                # Send to motors
                if self._motor_callback:
                    await self._motor_callback(wheel_vels)
                    
                # Sleep to maintain control frequency
                elapsed = time.time() - current_time
                sleep_time = max(0, self._control_dt - elapsed)
                await asyncio.sleep(sleep_time)
                
            except Exception as e:
                logger.error(f"Control loop error: {e}")
                await asyncio.sleep(self._control_dt)
                
    async def _execute_control(self, dt: float) -> WheelVelocities:
        """Execute control based on current mode"""
        if self.state.mode == ControlMode.EMERGENCY_STOP:
            return WheelVelocities([0.0] * self.kinematics.num_wheels)
            
        elif self.state.mode == ControlMode.IDLE:
            return WheelVelocities([0.0] * self.kinematics.num_wheels)
            
        elif self.state.mode == ControlMode.TELEOP or self.state.mode == ControlMode.VELOCITY:
            # Direct velocity control
            return await self._velocity_control(self.state.target_velocity)
            
        elif self.state.mode == ControlMode.HEADING:
            # Heading control (maintain orientation)
            return await self._heading_control(self.state.target_heading, dt)
            
        elif self.state.mode == ControlMode.POSITION:
            # Position control (go to point)
            if self.state.target_position:
                return await self._position_control(
                    self.state.target_position,
                    dt
                )
            return WheelVelocities([0.0] * self.kinematics.num_wheels)
            
        return WheelVelocities([0.0] * self.kinematics.num_wheels)
    
    async def _velocity_control(self, velocity: RobotVelocity) -> WheelVelocities:
        """Direct velocity control with smoothing"""
        # Clamp velocity
        clamped = self.kinematics.clamp_velocity(velocity)
        
        # Apply smoothing
        smoothed_linear = self._linear_filter.update(clamped.linear_x)
        smoothed_angular = self._angular_filter.update(clamped.angular_z)
        
        smoothed_velocity = RobotVelocity(
            linear_x=smoothed_linear,
            linear_y=clamped.linear_y,
            angular_z=smoothed_angular
        )
        
        self.state.current_velocity = smoothed_velocity
        
        # Convert to wheel velocities
        return self.kinematics.inverse_kinematics(smoothed_velocity)
    
    async def _heading_control(self, target_heading: float, dt: float) -> WheelVelocities:
        """Control to maintain heading"""
        angular_velocity = self.heading_controller.update(
            target_heading,
            self.state.current_velocity.angular_z,  # Current heading from odometry
            dt
        )
        
        velocity = RobotVelocity(
            linear_x=self.state.target_velocity.linear_x,
            angular_z=angular_velocity
        )
        
        return await self._velocity_control(velocity)
    
    async def _position_control(
        self,
        target: tuple,
        dt: float
    ) -> WheelVelocities:
        """Control to reach target position"""
        # This would integrate with odometry
        # For now, use velocity commands from position controller
        velocity = self.position_controller.update(
            target,
            (0.0, 0.0, 0.0),  # Current pose from odometry
            dt
        )
        
        return await self._velocity_control(velocity)
    
    def set_motor_callback(self, callback: Callable) -> None:
        """Set callback for motor output"""
        self._motor_callback = callback
        
    def set_mode(self, mode: ControlMode) -> None:
        """Set control mode"""
        if mode != self.state.mode:
            logger.info(f"Control mode: {self.state.mode.value} -> {mode.value}")
            
            # Reset controllers when changing mode
            self.velocity_controller.reset()
            self.position_controller.reset()
            self.heading_controller.reset()
            
        self.state.mode = mode
        
    def set_velocity(self, linear: float, angular: float, linear_y: float = 0.0) -> None:
        """Set target velocity (for teleop/velocity mode)"""
        self.state.target_velocity = RobotVelocity(
            linear_x=linear,
            linear_y=linear_y,
            angular_z=angular
        )
        
        if self.state.mode == ControlMode.IDLE:
            self.set_mode(ControlMode.TELEOP)
            
    def set_position_target(self, x: float, y: float, theta: float) -> None:
        """Set position target"""
        self.state.target_position = (x, y, theta)
        self.set_mode(ControlMode.POSITION)
        
    def set_heading_target(self, heading: float) -> None:
        """Set heading target"""
        self.state.target_heading = heading
        self.set_mode(ControlMode.HEADING)
        
    async def emergency_stop(self) -> None:
        """Emergency stop - immediately halt all motion"""
        self.state.mode = ControlMode.EMERGENCY_STOP
        self.state.target_velocity = RobotVelocity()
        
        # Reset filters for immediate response
        self._linear_filter.reset()
        self._angular_filter.reset()
        
        # Send stop command
        if self._motor_callback:
            await self._motor_callback(
                WheelVelocities([0.0] * self.kinematics.num_wheels)
            )
            
        logger.warning("Emergency stop activated")
        
    def resume(self) -> None:
        """Resume from emergency stop"""
        if self.state.mode == ControlMode.EMERGENCY_STOP:
            self.set_mode(ControlMode.IDLE)
            logger.info("Resumed from emergency stop")


class VelocityController:
    """Velocity controller with PID for each wheel"""
    
    def __init__(self, config: Dict[str, Any], params: RobotPhysicalParams):
        self.config = config
        self.params = params
        
        pid_config = config.get("motors", {}).get("velocity_pid", {})
        
        # PID for linear velocity
        self.linear_pid = PIDController(
            kp=pid_config.get("kp", 1.0),
            ki=pid_config.get("ki", 0.1),
            kd=pid_config.get("kd", 0.01),
            output_min=pid_config.get("output_min", -255),
            output_max=pid_config.get("output_max", 255)
        )
        
        # PID for angular velocity
        self.angular_pid = PIDController(
            kp=pid_config.get("kp", 1.0),
            ki=pid_config.get("ki", 0.1),
            kd=pid_config.get("kd", 0.01),
            output_min=pid_config.get("output_min", -255),
            output_max=pid_config.get("output_max", 255)
        )
        
    def reset(self) -> None:
        """Reset controllers"""
        self.linear_pid.reset()
        self.angular_pid.reset()


class PositionController:
    """Position controller for navigation"""
    
    def __init__(self, config: Dict[str, Any], params: RobotPhysicalParams):
        self.config = config
        self.params = params
        
        nav_config = config.get("control", {}).get("navigation_pid", {})
        
        # PID for linear motion
        self.linear_pid = PIDController(
            kp=nav_config.get("linear_kp", 0.5),
            ki=nav_config.get("linear_ki", 0.0),
            kd=nav_config.get("linear_kd", 0.1),
            output_min=-params.max_linear_velocity,
            output_max=params.max_linear_velocity
        )
        
        # PID for angular motion
        self.angular_pid = AnglePIDController(
            kp=nav_config.get("angular_kp", 1.0),
            ki=nav_config.get("angular_ki", 0.0),
            kd=nav_config.get("angular_kd", 0.1),
            output_min=-params.max_angular_velocity,
            output_max=params.max_angular_velocity
        )
        
        # Distance threshold for goal reached
        self.goal_tolerance = 0.1  # meters
        self.angle_tolerance = 0.1  # radians
        
    def update(
        self,
        target: tuple,
        current: tuple,
        dt: float
    ) -> RobotVelocity:
        """
        Update position controller
        
        Args:
            target: Target pose (x, y, theta)
            current: Current pose (x, y, theta)
            dt: Time step
            
        Returns:
            Velocity command
        """
        import math
        
        target_x, target_y, target_theta = target
        current_x, current_y, current_theta = current
        
        # Calculate distance and angle to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Angle to target
        angle_to_target = math.atan2(dy, dx)
        
        # Check if goal reached
        if distance < self.goal_tolerance:
            # At position, just rotate to target heading
            angular = self.angular_pid.update(target_theta, current_theta)
            return RobotVelocity(linear_x=0.0, angular_z=angular)
            
        # Calculate heading error
        heading_error = angle_to_target - current_theta
        
        # Normalize heading error
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
            
        # If facing wrong direction, prioritize turning
        if abs(heading_error) > 0.5:  # ~30 degrees
            linear = 0.0
        else:
            # Scale linear velocity by cos of heading error
            linear = self.linear_pid.update(distance) * math.cos(heading_error)
            
        angular = self.angular_pid.update(angle_to_target, current_theta)
        
        return RobotVelocity(linear_x=linear, angular_z=angular)
    
    def reset(self) -> None:
        """Reset controllers"""
        self.linear_pid.reset()
        self.angular_pid.reset()


class HeadingController:
    """Heading controller for maintaining orientation"""
    
    def __init__(self, config: Dict[str, Any]):
        heading_config = config.get("control", {}).get("heading_pid", {})
        
        self.pid = AnglePIDController(
            kp=heading_config.get("kp", 1.5),
            ki=heading_config.get("ki", 0.0),
            kd=heading_config.get("kd", 0.2),
            output_min=-2.0,  # Max angular velocity
            output_max=2.0
        )
        
    def update(
        self,
        target_heading: float,
        current_heading: float,
        dt: float
    ) -> float:
        """
        Update heading controller
        
        Args:
            target_heading: Target heading (radians)
            current_heading: Current heading (radians)
            dt: Time step
            
        Returns:
            Angular velocity command
        """
        return self.pid.update(target_heading, current_heading)
    
    def reset(self) -> None:
        """Reset controller"""
        self.pid.reset()
