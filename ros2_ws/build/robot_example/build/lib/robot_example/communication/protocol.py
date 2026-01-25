"""
RoboTwin Communication - Protocol Definitions
Message types and serialization compatible with rosbridge protocol
"""

import time
from dataclasses import dataclass, field, asdict
from typing import Optional, Dict, Any, List, Union
from enum import Enum
import json


class MessageType(Enum):
    """Message types for robot-server communication"""
    # Connection
    AUTH = "auth"
    PING = "ping"
    PONG = "pong"
    
    # Robot state
    TELEMETRY = "telemetry"
    STATUS = "status"
    DIAGNOSTICS = "diagnostics"
    
    # Control
    TELEOP = "teleop"
    COMMAND = "command"
    VELOCITY = "velocity"
    
    # Navigation
    GOAL = "goal"
    PATH = "path"
    CANCEL_GOAL = "cancel_goal"
    
    # Configuration
    CONFIG = "config"
    PARAM = "param"
    
    # ROS Bridge compatible
    SUBSCRIBE = "subscribe"
    UNSUBSCRIBE = "unsubscribe"
    ADVERTISE = "advertise"
    UNADVERTISE = "unadvertise"
    PUBLISH = "publish"
    CALL_SERVICE = "call_service"
    SERVICE_RESPONSE = "service_response"


@dataclass
class Message:
    """Base message class"""
    type: str
    timestamp: int = field(default_factory=lambda: int(time.time() * 1000))
    id: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type,
            "timestamp": self.timestamp,
            "id": self.id
        }
    
    def to_json(self) -> str:
        return json.dumps(self.to_dict())


@dataclass
class TelemetryMessage(Message):
    """Robot telemetry message"""
    robot_id: str = ""
    
    # Position
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    
    # Velocity
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    
    # Battery
    battery_voltage: float = 0.0
    battery_percentage: float = 0.0
    
    # Status
    status: str = "ready"
    mode: str = "idle"
    
    def __post_init__(self):
        self.type = MessageType.TELEMETRY.value
        
    def to_dict(self) -> Dict[str, Any]:
        base = super().to_dict()
        base["payload"] = {
            "robotId": self.robot_id,
            "position": {
                "x": self.x,
                "y": self.y,
                "theta": self.theta
            },
            "velocity": {
                "linear": self.linear_velocity,
                "angular": self.angular_velocity
            },
            "battery": {
                "voltage": self.battery_voltage,
                "percentage": self.battery_percentage
            },
            "status": self.status,
            "mode": self.mode,
            "timestamp": self.timestamp
        }
        return base


@dataclass
class TeleopMessage(Message):
    """Teleop command message"""
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    
    def __post_init__(self):
        self.type = MessageType.TELEOP.value
        
    def to_dict(self) -> Dict[str, Any]:
        base = super().to_dict()
        base["payload"] = {
            "linear": {
                "x": self.linear_x,
                "y": self.linear_y
            },
            "angular": {
                "z": self.angular_z
            }
        }
        return base
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TeleopMessage":
        payload = data.get("payload", data)
        linear = payload.get("linear", {})
        angular = payload.get("angular", {})
        
        return cls(
            linear_x=linear.get("x", linear.get("linear", 0.0)),
            linear_y=linear.get("y", 0.0),
            angular_z=angular.get("z", angular.get("angular", 0.0)),
            timestamp=data.get("timestamp", int(time.time() * 1000))
        )


@dataclass
class CommandMessage(Message):
    """Robot command message"""
    command: str = ""
    args: Dict[str, Any] = field(default_factory=dict)
    
    def __post_init__(self):
        self.type = MessageType.COMMAND.value
        
    def to_dict(self) -> Dict[str, Any]:
        base = super().to_dict()
        base["payload"] = {
            "command": self.command,
            "args": self.args
        }
        return base
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CommandMessage":
        payload = data.get("payload", data)
        return cls(
            command=payload.get("command", ""),
            args=payload.get("args", {}),
            timestamp=data.get("timestamp", int(time.time() * 1000))
        )


@dataclass
class GoalMessage(Message):
    """Navigation goal message"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    frame_id: str = "map"
    
    def __post_init__(self):
        self.type = MessageType.GOAL.value
        
    def to_dict(self) -> Dict[str, Any]:
        base = super().to_dict()
        base["payload"] = {
            "pose": {
                "position": {"x": self.x, "y": self.y, "z": 0.0},
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": self._sin_half_theta(),
                    "w": self._cos_half_theta()
                }
            },
            "frame_id": self.frame_id
        }
        return base
    
    def _sin_half_theta(self) -> float:
        import math
        return math.sin(self.theta / 2)
    
    def _cos_half_theta(self) -> float:
        import math
        return math.cos(self.theta / 2)


@dataclass
class DiagnosticsMessage(Message):
    """Robot diagnostics message"""
    robot_id: str = ""
    cpu_percent: float = 0.0
    memory_percent: float = 0.0
    disk_percent: float = 0.0
    temperature: Optional[float] = None
    uptime: float = 0.0
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    
    def __post_init__(self):
        self.type = MessageType.DIAGNOSTICS.value
        
    def to_dict(self) -> Dict[str, Any]:
        base = super().to_dict()
        base["payload"] = {
            "robotId": self.robot_id,
            "cpu": self.cpu_percent,
            "memory": self.memory_percent,
            "disk": self.disk_percent,
            "temperature": self.temperature,
            "uptime": self.uptime,
            "errors": self.errors,
            "warnings": self.warnings
        }
        return base


# ============================================================================
# ROS Bridge Protocol Messages
# ============================================================================

@dataclass
class ROSPublishMessage:
    """ROS bridge publish message"""
    topic: str
    msg: Dict[str, Any]
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "op": "publish",
            "topic": self.topic,
            "msg": self.msg
        }


@dataclass
class ROSSubscribeMessage:
    """ROS bridge subscribe message"""
    topic: str
    msg_type: str
    throttle_rate: int = 0
    queue_length: int = 1
    fragment_size: Optional[int] = None
    compression: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        data = {
            "op": "subscribe",
            "topic": self.topic,
            "type": self.msg_type,
            "throttle_rate": self.throttle_rate,
            "queue_length": self.queue_length
        }
        if self.fragment_size:
            data["fragment_size"] = self.fragment_size
        if self.compression:
            data["compression"] = self.compression
        return data


@dataclass
class ROSServiceCall:
    """ROS bridge service call message"""
    service: str
    args: Dict[str, Any]
    id: Optional[str] = None
    fragment_size: Optional[int] = None
    compression: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        data = {
            "op": "call_service",
            "service": self.service,
            "args": self.args
        }
        if self.id:
            data["id"] = self.id
        return data


# ============================================================================
# Factory Functions
# ============================================================================

def create_message(
    msg_type: MessageType,
    **kwargs
) -> Dict[str, Any]:
    """
    Create message dictionary
    
    Args:
        msg_type: Message type
        **kwargs: Message fields
        
    Returns:
        Message dictionary
    """
    message = {
        "type": msg_type.value,
        "timestamp": int(time.time() * 1000)
    }
    
    if "id" in kwargs:
        message["id"] = kwargs.pop("id")
        
    if "payload" in kwargs:
        message["payload"] = kwargs["payload"]
    else:
        message["payload"] = kwargs
        
    return message


def parse_message(data: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
    """
    Parse message from JSON or dict
    
    Args:
        data: JSON string or dictionary
        
    Returns:
        Parsed message dictionary
    """
    if isinstance(data, str):
        data = json.loads(data)
        
    return data


def parse_teleop(data: Dict[str, Any]) -> TeleopMessage:
    """Parse teleop message"""
    return TeleopMessage.from_dict(data)


def parse_command(data: Dict[str, Any]) -> CommandMessage:
    """Parse command message"""
    return CommandMessage.from_dict(data)


# ============================================================================
# Standard ROS Message Types
# ============================================================================

class ROSMessageTypes:
    """Standard ROS message type names"""
    # Geometry
    TWIST = "geometry_msgs/Twist"
    POSE = "geometry_msgs/Pose"
    POSE_STAMPED = "geometry_msgs/PoseStamped"
    POSE_WITH_COVARIANCE = "geometry_msgs/PoseWithCovariance"
    POINT = "geometry_msgs/Point"
    QUATERNION = "geometry_msgs/Quaternion"
    TRANSFORM = "geometry_msgs/Transform"
    TRANSFORM_STAMPED = "geometry_msgs/TransformStamped"
    
    # Sensor
    LASER_SCAN = "sensor_msgs/LaserScan"
    IMAGE = "sensor_msgs/Image"
    COMPRESSED_IMAGE = "sensor_msgs/CompressedImage"
    IMU = "sensor_msgs/Imu"
    BATTERY_STATE = "sensor_msgs/BatteryState"
    POINT_CLOUD2 = "sensor_msgs/PointCloud2"
    
    # Navigation
    ODOMETRY = "nav_msgs/Odometry"
    PATH = "nav_msgs/Path"
    OCCUPANCY_GRID = "nav_msgs/OccupancyGrid"
    
    # Standard
    STRING = "std_msgs/String"
    BOOL = "std_msgs/Bool"
    INT32 = "std_msgs/Int32"
    FLOAT32 = "std_msgs/Float32"
    HEADER = "std_msgs/Header"


def create_twist_message(
    linear_x: float = 0.0,
    linear_y: float = 0.0,
    linear_z: float = 0.0,
    angular_x: float = 0.0,
    angular_y: float = 0.0,
    angular_z: float = 0.0
) -> Dict[str, Any]:
    """Create geometry_msgs/Twist message"""
    return {
        "linear": {"x": linear_x, "y": linear_y, "z": linear_z},
        "angular": {"x": angular_x, "y": angular_y, "z": angular_z}
    }


def create_pose_message(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
    qw: float = 1.0
) -> Dict[str, Any]:
    """Create geometry_msgs/Pose message"""
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}
    }


def create_header(
    frame_id: str = "",
    timestamp_secs: int = 0,
    timestamp_nsecs: int = 0
) -> Dict[str, Any]:
    """Create std_msgs/Header message"""
    if timestamp_secs == 0:
        import time
        now = time.time()
        timestamp_secs = int(now)
        timestamp_nsecs = int((now - timestamp_secs) * 1e9)
        
    return {
        "stamp": {
            "sec": timestamp_secs,
            "nanosec": timestamp_nsecs
        },
        "frame_id": frame_id
    }


def create_laser_scan_message(
    ranges: List[float],
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float = 0.1,
    range_max: float = 12.0,
    frame_id: str = "laser"
) -> Dict[str, Any]:
    """Create sensor_msgs/LaserScan message"""
    return {
        "header": create_header(frame_id),
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": angle_increment,
        "time_increment": 0.0,
        "scan_time": 0.1,
        "range_min": range_min,
        "range_max": range_max,
        "ranges": ranges,
        "intensities": []
    }


def create_odometry_message(
    x: float,
    y: float,
    theta: float,
    linear_vel: float,
    angular_vel: float,
    frame_id: str = "odom",
    child_frame_id: str = "base_link"
) -> Dict[str, Any]:
    """Create nav_msgs/Odometry message"""
    import math
    
    return {
        "header": create_header(frame_id),
        "child_frame_id": child_frame_id,
        "pose": {
            "pose": create_pose_message(
                x=x, y=y,
                qz=math.sin(theta / 2),
                qw=math.cos(theta / 2)
            ),
            "covariance": [0.0] * 36
        },
        "twist": {
            "twist": create_twist_message(
                linear_x=linear_vel,
                angular_z=angular_vel
            ),
            "covariance": [0.0] * 36
        }
    }
