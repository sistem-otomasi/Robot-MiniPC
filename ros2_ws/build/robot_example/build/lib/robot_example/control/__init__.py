"""
RoboTwin Control Module
PID controllers, filters, and motion control
"""

from robot_example.control.pid import (
    PIDController,
    PIDConfig,
    AdaptivePIDController
)

from robot_example.control.filters import (
    LowPassFilter,
    HighPassFilter,
    KalmanFilter1D,
    KalmanFilter2D,
    ComplementaryFilter,
    MovingAverageFilter,
    MedianFilter
)

from robot_example.control.controller import (
    MainController,
    VelocityController,
    PositionController,
    HeadingController
)

from robot_example.control.motion import (
    MotionPlanner,
    TrajectoryGenerator,
    VelocityProfile
)

__all__ = [
    # PID
    "PIDController",
    "PIDConfig",
    "AdaptivePIDController",
    # Filters
    "LowPassFilter",
    "HighPassFilter",
    "KalmanFilter1D",
    "KalmanFilter2D",
    "ComplementaryFilter",
    "MovingAverageFilter",
    "MedianFilter",
    # Controllers
    "MainController",
    "VelocityController",
    "PositionController",
    "HeadingController",
    # Motion
    "MotionPlanner",
    "TrajectoryGenerator",
    "VelocityProfile"
]
