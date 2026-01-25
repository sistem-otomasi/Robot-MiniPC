"""
RoboTwin Control - PID Controllers
Various PID controller implementations
"""

import time
from dataclasses import dataclass, field
from typing import Optional, Tuple
import math

from robotwin.utils.logger import get_logger

logger = get_logger(__name__)


@dataclass
class PIDConfig:
    """PID controller configuration"""
    kp: float = 1.0           # Proportional gain
    ki: float = 0.0           # Integral gain
    kd: float = 0.0           # Derivative gain
    output_min: float = -1.0  # Minimum output
    output_max: float = 1.0   # Maximum output
    integral_min: float = -10.0  # Anti-windup minimum
    integral_max: float = 10.0   # Anti-windup maximum
    deadband: float = 0.0     # Error deadband
    setpoint_ramp: float = 0.0  # Setpoint ramping rate (0 = disabled)
    derivative_filter: float = 0.0  # Derivative low-pass filter coefficient
    

class PIDController:
    """
    Standard PID Controller with anti-windup and derivative filtering
    
    Features:
    - Anti-windup clamping
    - Derivative on measurement (not error)
    - Derivative low-pass filtering
    - Output clamping
    - Deadband
    - Setpoint ramping
    """
    
    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -1.0,
        output_max: float = 1.0,
        integral_min: float = -10.0,
        integral_max: float = 10.0,
        deadband: float = 0.0
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.deadband = deadband
        
        # Internal state
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_measurement = None
        self._prev_time = None
        self._derivative_filtered = 0.0
        self._derivative_filter_coeff = 0.1
        
        # Setpoint ramping
        self._setpoint = 0.0
        self._target_setpoint = 0.0
        self._setpoint_ramp_rate = 0.0
        
    def update(
        self,
        error: float,
        timestamp: Optional[float] = None,
        measurement: Optional[float] = None
    ) -> float:
        """
        Update PID controller and return control output
        
        Args:
            error: Current error (setpoint - measurement)
            timestamp: Current timestamp (uses time.time() if not provided)
            measurement: Current measurement (for derivative on measurement)
            
        Returns:
            Control output
        """
        current_time = timestamp or time.time()
        
        # Calculate dt
        if self._prev_time is None:
            dt = 0.02  # Default 50Hz
        else:
            dt = current_time - self._prev_time
            
        if dt <= 0:
            dt = 0.02
            
        self._prev_time = current_time
        
        # Apply deadband
        if abs(error) < self.deadband:
            error = 0.0
            
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self._integral += error * dt
        self._integral = max(self.integral_min, 
                           min(self.integral_max, self._integral))
        i_term = self.ki * self._integral
        
        # Derivative term
        if measurement is not None and self._prev_measurement is not None:
            # Derivative on measurement (avoids derivative kick)
            raw_derivative = -(measurement - self._prev_measurement) / dt
        else:
            # Derivative on error
            raw_derivative = (error - self._prev_error) / dt
            
        # Low-pass filter on derivative
        self._derivative_filtered = (
            self._derivative_filter_coeff * raw_derivative +
            (1 - self._derivative_filter_coeff) * self._derivative_filtered
        )
        d_term = self.kd * self._derivative_filtered
        
        # Store previous values
        self._prev_error = error
        if measurement is not None:
            self._prev_measurement = measurement
            
        # Calculate output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(self.output_min, min(self.output_max, output))
        
        return output
    
    def reset(self) -> None:
        """Reset controller state"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_measurement = None
        self._prev_time = None
        self._derivative_filtered = 0.0
        
    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """Update PID gains"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
    def set_output_limits(self, min_val: float, max_val: float) -> None:
        """Set output limits"""
        self.output_min = min_val
        self.output_max = max_val
        
    def set_integral_limits(self, min_val: float, max_val: float) -> None:
        """Set integral limits (anti-windup)"""
        self.integral_min = min_val
        self.integral_max = max_val
        
    def get_terms(self) -> Tuple[float, float, float]:
        """Get individual PID terms (for debugging)"""
        p = self.kp * self._prev_error
        i = self.ki * self._integral
        d = self.kd * self._derivative_filtered
        return p, i, d
    
    @classmethod
    def from_config(cls, config: PIDConfig) -> "PIDController":
        """Create PID controller from config"""
        return cls(
            kp=config.kp,
            ki=config.ki,
            kd=config.kd,
            output_min=config.output_min,
            output_max=config.output_max,
            integral_min=config.integral_min,
            integral_max=config.integral_max,
            deadband=config.deadband
        )


class AdaptivePIDController(PIDController):
    """
    Adaptive PID Controller with gain scheduling
    
    Automatically adjusts gains based on error magnitude
    """
    
    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -1.0,
        output_max: float = 1.0,
        # Gain schedule parameters
        error_threshold_low: float = 0.1,
        error_threshold_high: float = 1.0,
        gain_scale_low: float = 0.5,
        gain_scale_high: float = 1.5
    ):
        super().__init__(kp, ki, kd, output_min, output_max)
        
        self.error_threshold_low = error_threshold_low
        self.error_threshold_high = error_threshold_high
        self.gain_scale_low = gain_scale_low
        self.gain_scale_high = gain_scale_high
        
        # Base gains (original values)
        self._base_kp = kp
        self._base_ki = ki
        self._base_kd = kd
        
    def update(
        self,
        error: float,
        timestamp: Optional[float] = None,
        measurement: Optional[float] = None
    ) -> float:
        """Update with adaptive gains"""
        # Compute gain scale based on error magnitude
        abs_error = abs(error)
        
        if abs_error < self.error_threshold_low:
            scale = self.gain_scale_low
        elif abs_error > self.error_threshold_high:
            scale = self.gain_scale_high
        else:
            # Linear interpolation
            t = (abs_error - self.error_threshold_low) / (
                self.error_threshold_high - self.error_threshold_low
            )
            scale = self.gain_scale_low + t * (self.gain_scale_high - self.gain_scale_low)
            
        # Apply scaled gains
        self.kp = self._base_kp * scale
        self.ki = self._base_ki * scale
        self.kd = self._base_kd * scale
        
        # Call parent update
        return super().update(error, timestamp, measurement)


class PIDCascade:
    """
    Cascaded PID controller (outer loop feeds inner loop)
    
    Common usage: Position PID -> Velocity PID -> Motor PWM
    """
    
    def __init__(
        self,
        outer_pid: PIDController,
        inner_pid: PIDController,
        inner_setpoint_limit: float = float('inf')
    ):
        self.outer_pid = outer_pid
        self.inner_pid = inner_pid
        self.inner_setpoint_limit = inner_setpoint_limit
        
    def update(
        self,
        outer_error: float,
        inner_measurement: float,
        timestamp: Optional[float] = None
    ) -> float:
        """
        Update cascade controller
        
        Args:
            outer_error: Error for outer loop (e.g., position error)
            inner_measurement: Measurement for inner loop (e.g., current velocity)
            timestamp: Current timestamp
            
        Returns:
            Final control output
        """
        # Outer loop generates setpoint for inner loop
        inner_setpoint = self.outer_pid.update(outer_error, timestamp)
        
        # Limit inner setpoint
        inner_setpoint = max(-self.inner_setpoint_limit,
                           min(self.inner_setpoint_limit, inner_setpoint))
        
        # Inner loop error
        inner_error = inner_setpoint - inner_measurement
        
        # Inner loop control
        return self.inner_pid.update(inner_error, timestamp, inner_measurement)
    
    def reset(self) -> None:
        """Reset both controllers"""
        self.outer_pid.reset()
        self.inner_pid.reset()


class AnglePIDController(PIDController):
    """
    PID controller for angular values
    
    Handles angle wrapping (e.g., 359° to 1° should be 2°, not 358°)
    """
    
    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -1.0,
        output_max: float = 1.0,
        wrap_angle: float = math.pi  # Use pi for radians, 180 for degrees
    ):
        super().__init__(kp, ki, kd, output_min, output_max)
        self.wrap_angle = wrap_angle
        
    def update(
        self,
        setpoint: float,
        measurement: float,
        timestamp: Optional[float] = None
    ) -> float:
        """
        Update with angle wrapping
        
        Args:
            setpoint: Target angle
            measurement: Current angle
            timestamp: Current timestamp
            
        Returns:
            Control output
        """
        # Calculate wrapped error
        error = setpoint - measurement
        
        # Wrap to [-wrap_angle, wrap_angle]
        while error > self.wrap_angle:
            error -= 2 * self.wrap_angle
        while error < -self.wrap_angle:
            error += 2 * self.wrap_angle
            
        return super().update(error, timestamp, measurement)
