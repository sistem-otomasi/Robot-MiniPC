"""
RoboTwin Control - Motion Planning
Trajectory generation and motion profiles
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Generator
from enum import Enum

import numpy as np

from robot_example.utils.logger import get_logger
from robot_example.kinematics.base import RobotVelocity, RobotPhysicalParams

logger = get_logger(__name__)


@dataclass
class Waypoint:
    """Navigation waypoint"""
    x: float
    y: float
    theta: Optional[float] = None
    velocity: Optional[float] = None
    
    def distance_to(self, other: "Waypoint") -> float:
        """Calculate distance to another waypoint"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta or 0.0)


@dataclass
class TrajectoryPoint:
    """Point on a trajectory"""
    x: float
    y: float
    theta: float
    velocity: float
    angular_velocity: float
    time: float
    
    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "velocity": self.velocity,
            "angular_velocity": self.angular_velocity,
            "time": self.time
        }


class VelocityProfile(Enum):
    """Velocity profile types"""
    CONSTANT = "constant"
    TRAPEZOIDAL = "trapezoidal"
    S_CURVE = "s_curve"


# ============================================================================
# Velocity Profilers
# ============================================================================

class TrapezoidalProfile:
    """
    Trapezoidal velocity profile generator
    
    Generates smooth acceleration/deceleration with constant velocity cruise
    """
    
    def __init__(
        self,
        max_velocity: float = 1.0,
        max_acceleration: float = 0.5,
        max_deceleration: float = 0.5
    ):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration
        
    def generate(
        self,
        distance: float,
        initial_velocity: float = 0.0,
        final_velocity: float = 0.0,
        dt: float = 0.02
    ) -> Generator[Tuple[float, float, float], None, None]:
        """
        Generate trapezoidal velocity profile
        
        Args:
            distance: Total distance to travel
            initial_velocity: Starting velocity
            final_velocity: Ending velocity
            dt: Time step
            
        Yields:
            (time, position, velocity) tuples
        """
        # Calculate profile parameters
        v_max = self.max_velocity
        a = self.max_acceleration
        d = self.max_deceleration
        
        # Time and distance to accelerate to max velocity
        t_accel = (v_max - initial_velocity) / a
        d_accel = initial_velocity * t_accel + 0.5 * a * t_accel**2
        
        # Time and distance to decelerate to final velocity
        t_decel = (v_max - final_velocity) / d
        d_decel = v_max * t_decel - 0.5 * d * t_decel**2
        
        # Check if we can reach max velocity
        if d_accel + d_decel > distance:
            # Triangle profile - can't reach max velocity
            # Solve for peak velocity
            v_peak = math.sqrt(
                (2 * a * d * distance + d * initial_velocity**2 + a * final_velocity**2) /
                (a + d)
            )
            
            t_accel = (v_peak - initial_velocity) / a
            d_accel = initial_velocity * t_accel + 0.5 * a * t_accel**2
            
            t_decel = (v_peak - final_velocity) / d
            d_decel = v_peak * t_decel - 0.5 * d * t_decel**2
            
            t_cruise = 0.0
            d_cruise = 0.0
            v_cruise = v_peak
        else:
            # Trapezoidal profile
            d_cruise = distance - d_accel - d_decel
            t_cruise = d_cruise / v_max
            v_cruise = v_max
            
        total_time = t_accel + t_cruise + t_decel
        
        # Generate profile points
        t = 0.0
        pos = 0.0
        vel = initial_velocity
        
        while t <= total_time:
            if t < t_accel:
                # Acceleration phase
                vel = initial_velocity + a * t
                pos = initial_velocity * t + 0.5 * a * t**2
            elif t < t_accel + t_cruise:
                # Cruise phase
                tc = t - t_accel
                vel = v_cruise
                pos = d_accel + v_cruise * tc
            else:
                # Deceleration phase
                td = t - t_accel - t_cruise
                vel = v_cruise - d * td
                pos = d_accel + d_cruise + v_cruise * td - 0.5 * d * td**2
                
            yield (t, pos, max(0, vel))
            t += dt
            
        # Final point
        yield (total_time, distance, final_velocity)


class SCurveProfile:
    """
    S-Curve (7-segment) velocity profile generator
    
    Smoother than trapezoidal with limited jerk
    """
    
    def __init__(
        self,
        max_velocity: float = 1.0,
        max_acceleration: float = 0.5,
        max_jerk: float = 1.0
    ):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_jerk = max_jerk
        
    def generate(
        self,
        distance: float,
        initial_velocity: float = 0.0,
        final_velocity: float = 0.0,
        dt: float = 0.02
    ) -> Generator[Tuple[float, float, float], None, None]:
        """
        Generate S-curve velocity profile
        
        This is a simplified implementation using smoothstep blending
        """
        v_max = self.max_velocity
        a_max = self.max_acceleration
        j_max = self.max_jerk
        
        # Time to reach max acceleration
        t_jerk = a_max / j_max
        
        # Calculate total time using simplified model
        # (This is an approximation; full S-curve is more complex)
        t_total = distance / (v_max * 0.7)  # Account for accel/decel
        
        t = 0.0
        pos = 0.0
        vel = initial_velocity
        
        while t <= t_total:
            # Use smoothstep for velocity profile
            progress = t / t_total
            
            # Double smoothstep for S-curve like behavior
            if progress < 0.5:
                # Acceleration phase
                p = progress * 2
                smoothed = self._smoothstep(p)
                vel = initial_velocity + (v_max - initial_velocity) * smoothed
            else:
                # Deceleration phase
                p = (progress - 0.5) * 2
                smoothed = self._smoothstep(p)
                vel = v_max - (v_max - final_velocity) * smoothed
                
            pos = progress * distance
            
            yield (t, pos, vel)
            t += dt
            
        yield (t_total, distance, final_velocity)
        
    @staticmethod
    def _smoothstep(x: float) -> float:
        """Smoothstep interpolation"""
        x = max(0, min(1, x))
        return x * x * (3 - 2 * x)


# ============================================================================
# Trajectory Generator
# ============================================================================

class TrajectoryGenerator:
    """
    Generates trajectories from waypoints
    """
    
    def __init__(
        self,
        params: RobotPhysicalParams,
        profile_type: VelocityProfile = VelocityProfile.TRAPEZOIDAL
    ):
        self.params = params
        self.profile_type = profile_type
        
        # Create velocity profiler
        if profile_type == VelocityProfile.TRAPEZOIDAL:
            self.profiler = TrapezoidalProfile(
                max_velocity=params.max_linear_velocity,
                max_acceleration=0.5,
                max_deceleration=0.5
            )
        elif profile_type == VelocityProfile.S_CURVE:
            self.profiler = SCurveProfile(
                max_velocity=params.max_linear_velocity,
                max_acceleration=0.5,
                max_jerk=1.0
            )
        else:
            self.profiler = TrapezoidalProfile(params.max_linear_velocity)
            
    def generate_line(
        self,
        start: Waypoint,
        end: Waypoint,
        dt: float = 0.02
    ) -> List[TrajectoryPoint]:
        """
        Generate straight line trajectory between two waypoints
        """
        distance = start.distance_to(end)
        
        if distance < 0.001:
            return [TrajectoryPoint(
                x=start.x, y=start.y, theta=start.theta or 0.0,
                velocity=0.0, angular_velocity=0.0, time=0.0
            )]
            
        # Direction
        angle = math.atan2(end.y - start.y, end.x - start.x)
        
        trajectory = []
        
        for t, pos, vel in self.profiler.generate(distance, dt=dt):
            # Position along the line
            x = start.x + pos * math.cos(angle)
            y = start.y + pos * math.sin(angle)
            
            trajectory.append(TrajectoryPoint(
                x=x, y=y, theta=angle,
                velocity=vel, angular_velocity=0.0, time=t
            ))
            
        return trajectory
    
    def generate_arc(
        self,
        start: Waypoint,
        center: Tuple[float, float],
        end_angle: float,
        clockwise: bool = False,
        dt: float = 0.02
    ) -> List[TrajectoryPoint]:
        """
        Generate circular arc trajectory
        """
        cx, cy = center
        radius = math.sqrt((start.x - cx)**2 + (start.y - cy)**2)
        
        if radius < 0.001:
            return []
            
        start_angle = math.atan2(start.y - cy, start.x - cx)
        
        # Calculate arc length
        if clockwise:
            arc_angle = start_angle - end_angle
            if arc_angle < 0:
                arc_angle += 2 * math.pi
        else:
            arc_angle = end_angle - start_angle
            if arc_angle < 0:
                arc_angle += 2 * math.pi
                
        arc_length = radius * abs(arc_angle)
        
        trajectory = []
        
        for t, pos, vel in self.profiler.generate(arc_length, dt=dt):
            # Angle along the arc
            if clockwise:
                theta = start_angle - (pos / radius)
            else:
                theta = start_angle + (pos / radius)
                
            # Position
            x = cx + radius * math.cos(theta)
            y = cy + radius * math.sin(theta)
            
            # Tangent angle
            if clockwise:
                heading = theta - math.pi / 2
            else:
                heading = theta + math.pi / 2
                
            # Angular velocity
            angular_vel = vel / radius
            if clockwise:
                angular_vel = -angular_vel
                
            trajectory.append(TrajectoryPoint(
                x=x, y=y, theta=heading,
                velocity=vel, angular_velocity=angular_vel, time=t
            ))
            
        return trajectory
    
    def generate_path(
        self,
        waypoints: List[Waypoint],
        dt: float = 0.02
    ) -> List[TrajectoryPoint]:
        """
        Generate trajectory through multiple waypoints
        """
        if len(waypoints) < 2:
            return []
            
        full_trajectory = []
        time_offset = 0.0
        
        for i in range(len(waypoints) - 1):
            segment = self.generate_line(
                waypoints[i],
                waypoints[i + 1],
                dt
            )
            
            # Adjust time and append
            for point in segment:
                point.time += time_offset
                full_trajectory.append(point)
                
            if segment:
                time_offset = segment[-1].time
                
        return full_trajectory


# ============================================================================
# Motion Planner
# ============================================================================

class MotionPlanner:
    """
    Motion planner with trajectory following
    """
    
    def __init__(
        self,
        params: RobotPhysicalParams,
        lookahead_distance: float = 0.2,
        goal_tolerance: float = 0.1
    ):
        self.params = params
        self.lookahead_distance = lookahead_distance
        self.goal_tolerance = goal_tolerance
        
        self.trajectory_generator = TrajectoryGenerator(params)
        
        self._trajectory: List[TrajectoryPoint] = []
        self._current_index = 0
        self._active = False
        
    def set_trajectory(self, trajectory: List[TrajectoryPoint]) -> None:
        """Set trajectory to follow"""
        self._trajectory = trajectory
        self._current_index = 0
        self._active = True
        logger.info(f"Trajectory set with {len(trajectory)} points")
        
    def set_waypoints(self, waypoints: List[Waypoint]) -> None:
        """Generate and set trajectory from waypoints"""
        trajectory = self.trajectory_generator.generate_path(waypoints)
        self.set_trajectory(trajectory)
        
    def get_velocity_command(
        self,
        current_pose: Tuple[float, float, float]
    ) -> Optional[RobotVelocity]:
        """
        Get velocity command for current pose
        
        Uses pure pursuit algorithm
        """
        if not self._active or not self._trajectory:
            return None
            
        current_x, current_y, current_theta = current_pose
        
        # Find lookahead point
        lookahead_point = self._find_lookahead_point(current_x, current_y)
        
        if lookahead_point is None:
            # Goal reached
            self._active = False
            return RobotVelocity()
            
        # Pure pursuit steering
        dx = lookahead_point.x - current_x
        dy = lookahead_point.y - current_y
        
        # Transform to robot frame
        local_x = dx * math.cos(current_theta) + dy * math.sin(current_theta)
        local_y = -dx * math.sin(current_theta) + dy * math.cos(current_theta)
        
        # Calculate curvature
        l_sq = local_x**2 + local_y**2
        if l_sq < 0.001:
            curvature = 0.0
        else:
            curvature = 2 * local_y / l_sq
            
        # Velocity from trajectory
        velocity = lookahead_point.velocity
        angular_velocity = velocity * curvature
        
        # Clamp
        angular_velocity = max(-self.params.max_angular_velocity,
                              min(self.params.max_angular_velocity, angular_velocity))
        
        return RobotVelocity(
            linear_x=velocity,
            angular_z=angular_velocity
        )
    
    def _find_lookahead_point(
        self,
        current_x: float,
        current_y: float
    ) -> Optional[TrajectoryPoint]:
        """Find lookahead point on trajectory"""
        # Find closest point
        min_dist = float('inf')
        closest_idx = self._current_index
        
        for i in range(self._current_index, len(self._trajectory)):
            point = self._trajectory[i]
            dist = math.sqrt((point.x - current_x)**2 + (point.y - current_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        self._current_index = closest_idx
        
        # Check if at goal
        final_point = self._trajectory[-1]
        dist_to_goal = math.sqrt(
            (final_point.x - current_x)**2 +
            (final_point.y - current_y)**2
        )
        
        if dist_to_goal < self.goal_tolerance:
            return None
            
        # Find lookahead point
        for i in range(closest_idx, len(self._trajectory)):
            point = self._trajectory[i]
            dist = math.sqrt((point.x - current_x)**2 + (point.y - current_y)**2)
            
            if dist >= self.lookahead_distance:
                return point
                
        # Return last point if lookahead not found
        return self._trajectory[-1]
    
    def is_active(self) -> bool:
        """Check if following trajectory"""
        return self._active
    
    def cancel(self) -> None:
        """Cancel trajectory following"""
        self._active = False
        self._trajectory = []
        self._current_index = 0
    
    def get_progress(self) -> float:
        """Get trajectory completion progress (0-1)"""
        if not self._trajectory:
            return 1.0
        return self._current_index / len(self._trajectory)
