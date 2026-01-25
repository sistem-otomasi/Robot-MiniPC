"""
RoboTwin Control - Digital Filters
Various filter implementations for signal processing
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import time

import numpy as np

from robotwin.utils.logger import get_logger

logger = get_logger(__name__)


# ============================================================================
# Basic Filters
# ============================================================================

class LowPassFilter:
    """
    First-order IIR Low-Pass Filter
    
    y[n] = α * x[n] + (1-α) * y[n-1]
    where α = dt / (RC + dt)
    """
    
    def __init__(
        self,
        cutoff_freq: float = 5.0,
        sample_rate: float = 50.0,
        initial_value: float = 0.0
    ):
        self.cutoff_freq = cutoff_freq
        self.sample_rate = sample_rate
        
        # Calculate alpha from cutoff frequency
        dt = 1.0 / sample_rate
        rc = 1.0 / (2 * math.pi * cutoff_freq)
        self.alpha = dt / (rc + dt)
        
        self._value = initial_value
        self._initialized = initial_value != 0.0
        
    def update(self, measurement: float) -> float:
        """Update filter with new measurement"""
        if not self._initialized:
            self._value = measurement
            self._initialized = True
        else:
            self._value = self.alpha * measurement + (1 - self.alpha) * self._value
        return self._value
    
    def get_value(self) -> float:
        """Get current filtered value"""
        return self._value
    
    def reset(self, initial_value: float = 0.0) -> None:
        """Reset filter"""
        self._value = initial_value
        self._initialized = initial_value != 0.0
        
    def set_cutoff(self, cutoff_freq: float) -> None:
        """Update cutoff frequency"""
        self.cutoff_freq = cutoff_freq
        dt = 1.0 / self.sample_rate
        rc = 1.0 / (2 * math.pi * cutoff_freq)
        self.alpha = dt / (rc + dt)


class HighPassFilter:
    """
    First-order IIR High-Pass Filter
    
    y[n] = α * (y[n-1] + x[n] - x[n-1])
    """
    
    def __init__(
        self,
        cutoff_freq: float = 1.0,
        sample_rate: float = 50.0
    ):
        self.cutoff_freq = cutoff_freq
        self.sample_rate = sample_rate
        
        dt = 1.0 / sample_rate
        rc = 1.0 / (2 * math.pi * cutoff_freq)
        self.alpha = rc / (rc + dt)
        
        self._prev_input = 0.0
        self._prev_output = 0.0
        self._initialized = False
        
    def update(self, measurement: float) -> float:
        """Update filter with new measurement"""
        if not self._initialized:
            self._prev_input = measurement
            self._prev_output = 0.0
            self._initialized = True
            return 0.0
            
        output = self.alpha * (self._prev_output + measurement - self._prev_input)
        
        self._prev_input = measurement
        self._prev_output = output
        
        return output
    
    def reset(self) -> None:
        """Reset filter"""
        self._prev_input = 0.0
        self._prev_output = 0.0
        self._initialized = False


class BandPassFilter:
    """
    Band-Pass Filter (combination of low-pass and high-pass)
    """
    
    def __init__(
        self,
        low_cutoff: float = 0.5,
        high_cutoff: float = 10.0,
        sample_rate: float = 50.0
    ):
        self.low_pass = LowPassFilter(high_cutoff, sample_rate)
        self.high_pass = HighPassFilter(low_cutoff, sample_rate)
        
    def update(self, measurement: float) -> float:
        """Update filter"""
        return self.high_pass.update(self.low_pass.update(measurement))
    
    def reset(self) -> None:
        """Reset filter"""
        self.low_pass.reset()
        self.high_pass.reset()


# ============================================================================
# Statistical Filters
# ============================================================================

class MovingAverageFilter:
    """
    Simple Moving Average Filter
    """
    
    def __init__(self, window_size: int = 10):
        self.window_size = window_size
        self._buffer: deque = deque(maxlen=window_size)
        
    def update(self, measurement: float) -> float:
        """Update filter with new measurement"""
        self._buffer.append(measurement)
        return sum(self._buffer) / len(self._buffer)
    
    def get_value(self) -> float:
        """Get current average"""
        if len(self._buffer) == 0:
            return 0.0
        return sum(self._buffer) / len(self._buffer)
    
    def reset(self) -> None:
        """Reset filter"""
        self._buffer.clear()


class ExponentialMovingAverage:
    """
    Exponential Moving Average (EMA)
    
    More weight to recent values
    """
    
    def __init__(self, alpha: float = 0.2):
        """
        Args:
            alpha: Smoothing factor (0-1). Higher = more responsive
        """
        self.alpha = alpha
        self._value = 0.0
        self._initialized = False
        
    def update(self, measurement: float) -> float:
        """Update filter"""
        if not self._initialized:
            self._value = measurement
            self._initialized = True
        else:
            self._value = self.alpha * measurement + (1 - self.alpha) * self._value
        return self._value
    
    def reset(self) -> None:
        """Reset filter"""
        self._value = 0.0
        self._initialized = False


class MedianFilter:
    """
    Median Filter
    
    Good for removing outliers/spikes
    """
    
    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self._buffer: deque = deque(maxlen=window_size)
        
    def update(self, measurement: float) -> float:
        """Update filter with new measurement"""
        self._buffer.append(measurement)
        sorted_buffer = sorted(self._buffer)
        n = len(sorted_buffer)
        
        if n % 2 == 0:
            return (sorted_buffer[n//2 - 1] + sorted_buffer[n//2]) / 2
        else:
            return sorted_buffer[n//2]
    
    def reset(self) -> None:
        """Reset filter"""
        self._buffer.clear()


# ============================================================================
# Kalman Filters
# ============================================================================

class KalmanFilter1D:
    """
    Simple 1D Kalman Filter
    
    Good for smoothing noisy measurements with known process/measurement noise
    """
    
    def __init__(
        self,
        process_noise: float = 0.01,
        measurement_noise: float = 0.1,
        initial_estimate: float = 0.0,
        initial_error: float = 1.0
    ):
        self.process_noise = process_noise      # Q
        self.measurement_noise = measurement_noise  # R
        
        self._estimate = initial_estimate
        self._error_estimate = initial_error
        
    def update(self, measurement: float) -> float:
        """
        Update filter with new measurement
        
        Returns:
            Filtered estimate
        """
        # Prediction
        predicted_estimate = self._estimate
        predicted_error = self._error_estimate + self.process_noise
        
        # Update
        kalman_gain = predicted_error / (predicted_error + self.measurement_noise)
        self._estimate = predicted_estimate + kalman_gain * (measurement - predicted_estimate)
        self._error_estimate = (1 - kalman_gain) * predicted_error
        
        return self._estimate
    
    def get_estimate(self) -> Tuple[float, float]:
        """Get current estimate and uncertainty"""
        return self._estimate, self._error_estimate
    
    def reset(self, initial_estimate: float = 0.0) -> None:
        """Reset filter"""
        self._estimate = initial_estimate
        self._error_estimate = 1.0


class KalmanFilter2D:
    """
    2D Kalman Filter for position/velocity estimation
    
    State: [position, velocity]
    """
    
    def __init__(
        self,
        process_noise_pos: float = 0.01,
        process_noise_vel: float = 0.1,
        measurement_noise: float = 0.5,
        dt: float = 0.02
    ):
        self.dt = dt
        
        # State vector [position, velocity]
        self._x = np.array([[0.0], [0.0]])
        
        # State transition matrix
        self._F = np.array([
            [1, dt],
            [0, 1]
        ])
        
        # Measurement matrix (we only measure position)
        self._H = np.array([[1, 0]])
        
        # Process noise covariance
        self._Q = np.array([
            [process_noise_pos, 0],
            [0, process_noise_vel]
        ])
        
        # Measurement noise covariance
        self._R = np.array([[measurement_noise]])
        
        # Estimate covariance
        self._P = np.eye(2)
        
    def predict(self, dt: Optional[float] = None) -> Tuple[float, float]:
        """
        Predict step
        
        Args:
            dt: Time step (uses default if not provided)
            
        Returns:
            (predicted_position, predicted_velocity)
        """
        if dt is not None and dt != self.dt:
            self._F[0, 1] = dt
            
        # Predict state
        self._x = self._F @ self._x
        
        # Predict covariance
        self._P = self._F @ self._P @ self._F.T + self._Q
        
        return float(self._x[0, 0]), float(self._x[1, 0])
    
    def update(self, measurement: float) -> Tuple[float, float]:
        """
        Update step with position measurement
        
        Args:
            measurement: Position measurement
            
        Returns:
            (estimated_position, estimated_velocity)
        """
        # Measurement residual
        z = np.array([[measurement]])
        y = z - self._H @ self._x
        
        # Kalman gain
        S = self._H @ self._P @ self._H.T + self._R
        K = self._P @ self._H.T @ np.linalg.inv(S)
        
        # Update state
        self._x = self._x + K @ y
        
        # Update covariance
        I = np.eye(2)
        self._P = (I - K @ self._H) @ self._P
        
        return float(self._x[0, 0]), float(self._x[1, 0])
    
    def get_state(self) -> Tuple[float, float]:
        """Get current state estimate"""
        return float(self._x[0, 0]), float(self._x[1, 0])
    
    def reset(self, position: float = 0.0, velocity: float = 0.0) -> None:
        """Reset filter"""
        self._x = np.array([[position], [velocity]])
        self._P = np.eye(2)


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for robot pose estimation
    
    State: [x, y, theta]
    """
    
    def __init__(
        self,
        process_noise: np.ndarray = None,
        measurement_noise: np.ndarray = None
    ):
        # State vector [x, y, theta]
        self._x = np.zeros((3, 1))
        
        # State covariance
        self._P = np.eye(3) * 0.1
        
        # Process noise
        self._Q = process_noise if process_noise is not None else np.diag([0.01, 0.01, 0.01])
        
        # Measurement noise
        self._R = measurement_noise if measurement_noise is not None else np.diag([0.1, 0.1, 0.05])
        
    def predict(self, v: float, omega: float, dt: float) -> np.ndarray:
        """
        Predict step using motion model
        
        Args:
            v: Linear velocity
            omega: Angular velocity
            dt: Time step
            
        Returns:
            Predicted state
        """
        theta = self._x[2, 0]
        
        # Motion model (differential drive)
        if abs(omega) < 1e-6:
            # Straight motion
            dx = v * math.cos(theta) * dt
            dy = v * math.sin(theta) * dt
            dtheta = 0
        else:
            # Curved motion
            dx = (v / omega) * (math.sin(theta + omega * dt) - math.sin(theta))
            dy = (v / omega) * (math.cos(theta) - math.cos(theta + omega * dt))
            dtheta = omega * dt
            
        # Predict state
        self._x[0, 0] += dx
        self._x[1, 0] += dy
        self._x[2, 0] += dtheta
        
        # Normalize theta
        self._x[2, 0] = self._normalize_angle(self._x[2, 0])
        
        # Jacobian of motion model
        if abs(omega) < 1e-6:
            G = np.array([
                [1, 0, -v * math.sin(theta) * dt],
                [0, 1, v * math.cos(theta) * dt],
                [0, 0, 1]
            ])
        else:
            G = np.array([
                [1, 0, (v / omega) * (math.cos(theta + omega * dt) - math.cos(theta))],
                [0, 1, (v / omega) * (math.sin(theta + omega * dt) - math.sin(theta))],
                [0, 0, 1]
            ])
            
        # Predict covariance
        self._P = G @ self._P @ G.T + self._Q
        
        return self._x.copy()
    
    def update(self, z: np.ndarray) -> np.ndarray:
        """
        Update step with pose measurement
        
        Args:
            z: Measurement [x, y, theta]
            
        Returns:
            Updated state
        """
        # Measurement model is linear: z = H * x
        H = np.eye(3)
        
        # Innovation
        y = z.reshape(3, 1) - self._x
        y[2, 0] = self._normalize_angle(y[2, 0])
        
        # Kalman gain
        S = H @ self._P @ H.T + self._R
        K = self._P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self._x = self._x + K @ y
        self._x[2, 0] = self._normalize_angle(self._x[2, 0])
        
        # Update covariance
        I = np.eye(3)
        self._P = (I - K @ H) @ self._P
        
        return self._x.copy()
    
    def get_state(self) -> Tuple[float, float, float]:
        """Get current state"""
        return float(self._x[0, 0]), float(self._x[1, 0]), float(self._x[2, 0])
    
    def get_covariance(self) -> np.ndarray:
        """Get state covariance"""
        return self._P.copy()
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


# ============================================================================
# Sensor Fusion Filters
# ============================================================================

class ComplementaryFilter:
    """
    Complementary Filter for sensor fusion
    
    Commonly used to fuse gyroscope and accelerometer for orientation
    """
    
    def __init__(self, alpha: float = 0.98):
        """
        Args:
            alpha: Weight for high-frequency source (gyroscope)
                   1-alpha is weight for low-frequency source (accelerometer)
        """
        self.alpha = alpha
        self._angle = 0.0
        self._initialized = False
        
    def update(
        self,
        gyro_rate: float,
        accel_angle: float,
        dt: float
    ) -> float:
        """
        Update filter
        
        Args:
            gyro_rate: Angular rate from gyroscope (rad/s)
            accel_angle: Angle calculated from accelerometer (rad)
            dt: Time step
            
        Returns:
            Fused angle estimate
        """
        if not self._initialized:
            self._angle = accel_angle
            self._initialized = True
        else:
            # Integrate gyroscope
            gyro_angle = self._angle + gyro_rate * dt
            
            # Complementary fusion
            self._angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
            
        return self._angle
    
    def get_angle(self) -> float:
        """Get current angle estimate"""
        return self._angle
    
    def reset(self, initial_angle: float = 0.0) -> None:
        """Reset filter"""
        self._angle = initial_angle
        self._initialized = initial_angle != 0.0


class MadgwickFilter:
    """
    Madgwick AHRS (Attitude and Heading Reference System) Filter
    
    Fuses accelerometer, gyroscope, and magnetometer for 3D orientation
    """
    
    def __init__(self, beta: float = 0.1, sample_rate: float = 100.0):
        """
        Args:
            beta: Filter gain (higher = more responsive, noisier)
            sample_rate: Sensor sample rate in Hz
        """
        self.beta = beta
        self.sample_rate = sample_rate
        
        # Quaternion [w, x, y, z]
        self._q = np.array([1.0, 0.0, 0.0, 0.0])
        
    def update(
        self,
        gyro: Tuple[float, float, float],
        accel: Tuple[float, float, float],
        mag: Optional[Tuple[float, float, float]] = None
    ) -> np.ndarray:
        """
        Update filter with IMU data
        
        Args:
            gyro: Gyroscope [gx, gy, gz] in rad/s
            accel: Accelerometer [ax, ay, az] in m/s^2
            mag: Magnetometer [mx, my, mz] (optional)
            
        Returns:
            Quaternion [w, x, y, z]
        """
        q = self._q
        gx, gy, gz = gyro
        ax, ay, az = accel
        
        dt = 1.0 / self.sample_rate
        
        # Normalize accelerometer
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-10:
            return q
        ax, ay, az = ax/norm, ay/norm, az/norm
        
        # Gradient descent step
        q0, q1, q2, q3 = q
        
        # Auxiliary variables
        _2q0, _2q1, _2q2, _2q3 = 2*q0, 2*q1, 2*q2, 2*q3
        _4q0, _4q1, _4q2 = 4*q0, 4*q1, 4*q2
        _8q1, _8q2 = 8*q1, 8*q2
        q0q0, q1q1, q2q2, q3q3 = q0*q0, q1*q1, q2*q2, q3*q3
        
        # Gradient
        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
        s1 = _4q1*q3q3 - _2q3*ax + 4*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az
        s2 = 4*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az
        s3 = 4*q1q1*q3 - _2q1*ax + 4*q2q2*q3 - _2q2*ay
        
        # Normalize gradient
        norm = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if norm > 1e-10:
            s0, s1, s2, s3 = s0/norm, s1/norm, s2/norm, s3/norm
            
        # Compute rate of change
        qDot0 = 0.5 * (-q1*gx - q2*gy - q3*gz) - self.beta * s0
        qDot1 = 0.5 * (q0*gx + q2*gz - q3*gy) - self.beta * s1
        qDot2 = 0.5 * (q0*gy - q1*gz + q3*gx) - self.beta * s2
        qDot3 = 0.5 * (q0*gz + q1*gy - q2*gx) - self.beta * s3
        
        # Integrate
        q0 += qDot0 * dt
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        
        # Normalize quaternion
        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self._q = np.array([q0/norm, q1/norm, q2/norm, q3/norm])
        
        return self._q
    
    def get_quaternion(self) -> np.ndarray:
        """Get quaternion [w, x, y, z]"""
        return self._q.copy()
    
    def get_euler(self) -> Tuple[float, float, float]:
        """Get Euler angles (roll, pitch, yaw) in radians"""
        q = self._q
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2 * (x*x + y*y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w*y - z*x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def reset(self) -> None:
        """Reset to initial orientation"""
        self._q = np.array([1.0, 0.0, 0.0, 0.0])
