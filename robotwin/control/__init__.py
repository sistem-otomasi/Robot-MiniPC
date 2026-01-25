"""
RoboTwin Control Module
PID controllers, filters, and motion control
"""

from robotwin.control.pid import (
    PIDController,
    PIDConfig,
    AdaptivePIDController
)

from robotwin.control.filters import (
    LowPassFilter,
    HighPassFilter,
    KalmanFilter1D,
    KalmanFilter2D,
    ComplementaryFilter,
    MovingAverageFilter,
    MedianFilter
)

from robotwin.control.controller import (
    MainController,
    VelocityController,
    PositionController,
    HeadingController
)

from robotwin.control.motion import (
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
