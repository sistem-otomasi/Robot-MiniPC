"""
RoboTwin Utils - Configuration Manager
Load and validate robot configuration
"""

import os
from pathlib import Path
from typing import Optional, Dict, Any, List, Union
from dataclasses import dataclass, field

import yaml
from pydantic import BaseModel, Field, validator


# ============================================================================
# Configuration Models (with validation)
# ============================================================================

class PhysicsConfig(BaseModel):
    """Robot physical parameters"""
    wheel_radius: float = 0.05
    wheel_base: float = 0.3
    track_width: float = 0.3
    mass: float = 10.0
    max_linear_velocity: float = 1.0
    max_angular_velocity: float = 2.0
    max_linear_acceleration: float = 0.5
    max_angular_acceleration: float = 1.0


class MotorConfig(BaseModel):
    """Motor controller configuration"""
    type: str = "serial"
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    num_motors: int = 4
    max_rpm: float = 200.0
    encoder_cpr: int = 1000
    gear_ratio: float = 1.0
    
    # PID settings
    pid_kp: float = 1.0
    pid_ki: float = 0.1
    pid_kd: float = 0.01


class LidarConfig(BaseModel):
    """LiDAR sensor configuration"""
    enabled: bool = True
    type: str = "rplidar"
    port: str = "/dev/ttyUSB1"
    baudrate: int = 115200
    frame_id: str = "laser"
    range_min: float = 0.1
    range_max: float = 12.0
    angle_min: float = -3.14159
    angle_max: float = 3.14159


class CameraConfig(BaseModel):
    """Camera configuration"""
    enabled: bool = True
    type: str = "usb"
    device: Union[int, str] = 0
    width: int = 640
    height: int = 480
    fps: int = 30
    frame_id: str = "camera"


class IMUConfig(BaseModel):
    """IMU sensor configuration"""
    enabled: bool = True
    type: str = "mpu6050"
    bus: int = 1
    address: int = 0x68
    frame_id: str = "imu"
    update_rate: float = 100.0


class PIDConfig(BaseModel):
    """PID controller configuration"""
    kp: float = 1.0
    ki: float = 0.1
    kd: float = 0.01
    max_output: float = 1.0
    min_output: float = -1.0
    anti_windup: bool = True
    windup_limit: float = 1.0


class FilterConfig(BaseModel):
    """Filter configuration"""
    type: str = "kalman"
    # Kalman filter
    process_noise: float = 0.01
    measurement_noise: float = 0.1
    # Low pass filter
    cutoff_freq: float = 5.0
    # Complementary filter
    alpha: float = 0.98


class TransformConfig(BaseModel):
    """Coordinate transform configuration"""
    frame_id: str
    child_frame_id: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


class ServerConfig(BaseModel):
    """Server connection configuration"""
    url: str = "ws://localhost:3000/ws/robot"
    robot_id: str = "robot-001"
    robot_secret: str = ""
    reconnect_interval: float = 5.0
    heartbeat_interval: float = 10.0


class KinematicsConfig(BaseModel):
    """Kinematics configuration"""
    type: str = "differential"
    wheel_positions: List[List[float]] = Field(default_factory=list)


class RobotConfig(BaseModel):
    """Complete robot configuration"""
    robot_id: str = "robot-001"
    robot_name: str = "RoboTwin"
    
    # Subsystems
    physics: PhysicsConfig = Field(default_factory=PhysicsConfig)
    motors: MotorConfig = Field(default_factory=MotorConfig)
    lidar: LidarConfig = Field(default_factory=LidarConfig)
    camera: CameraConfig = Field(default_factory=CameraConfig)
    imu: IMUConfig = Field(default_factory=IMUConfig)
    kinematics: KinematicsConfig = Field(default_factory=KinematicsConfig)
    server: ServerConfig = Field(default_factory=ServerConfig)
    
    # Control
    velocity_pid: PIDConfig = Field(default_factory=PIDConfig)
    position_pid: PIDConfig = Field(default_factory=PIDConfig)
    heading_pid: PIDConfig = Field(default_factory=PIDConfig)
    
    # Filters
    velocity_filter: FilterConfig = Field(default_factory=FilterConfig)
    position_filter: FilterConfig = Field(default_factory=FilterConfig)
    imu_filter: FilterConfig = Field(default_factory=FilterConfig)
    
    # Transforms
    transforms: List[TransformConfig] = Field(default_factory=list)
    
    # Control loop rate
    control_rate: float = 50.0


# ============================================================================
# Configuration Loader
# ============================================================================

class Config:
    """
    Configuration manager
    
    Loads configuration from YAML file with environment variable override
    """
    
    def __init__(self, config_path: Optional[str] = None):
        self._config_path = config_path
        self._data: Dict[str, Any] = {}
        self._robot_config: Optional[RobotConfig] = None
        
        if config_path:
            self.load(config_path)
    
    def load(self, config_path: str) -> None:
        """
        Load configuration from YAML file
        
        Args:
            config_path: Path to configuration file
        """
        path = Path(config_path)
        
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")
            
        with open(path, 'r') as f:
            self._data = yaml.safe_load(f) or {}
            
        # Apply environment variable overrides
        self._apply_env_overrides()
        
        # Parse into validated model
        self._robot_config = RobotConfig(**self._data)
        
        self._config_path = config_path
        
    def _apply_env_overrides(self) -> None:
        """Apply environment variable overrides"""
        env_mappings = {
            "ROBOT_ID": ["robot_id"],
            "ROBOT_SECRET": ["server", "robot_secret"],
            "SERVER_URL": ["server", "url"],
            "MOTOR_PORT": ["motors", "port"],
            "LIDAR_PORT": ["lidar", "port"],
        }
        
        for env_var, keys in env_mappings.items():
            value = os.environ.get(env_var)
            if value:
                self._set_nested(self._data, keys, value)
                
    def _set_nested(
        self,
        data: Dict[str, Any],
        keys: List[str],
        value: Any
    ) -> None:
        """Set nested dictionary value"""
        for key in keys[:-1]:
            data = data.setdefault(key, {})
        data[keys[-1]] = value
        
    def _get_nested(
        self,
        data: Dict[str, Any],
        keys: List[str],
        default: Any = None
    ) -> Any:
        """Get nested dictionary value"""
        for key in keys:
            if isinstance(data, dict):
                data = data.get(key, default)
            else:
                return default
        return data
    
    @property
    def robot(self) -> RobotConfig:
        """Get validated robot configuration"""
        if self._robot_config is None:
            self._robot_config = RobotConfig()
        return self._robot_config
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        Get configuration value by dot-separated key
        
        Args:
            key: Configuration key (e.g., "motors.port")
            default: Default value if not found
            
        Returns:
            Configuration value
        """
        keys = key.split('.')
        return self._get_nested(self._data, keys, default)
    
    def set(self, key: str, value: Any) -> None:
        """
        Set configuration value by dot-separated key
        
        Args:
            key: Configuration key
            value: Value to set
        """
        keys = key.split('.')
        self._set_nested(self._data, keys, value)
        # Regenerate validated config
        self._robot_config = RobotConfig(**self._data)
        
    def save(self, config_path: Optional[str] = None) -> None:
        """
        Save configuration to YAML file
        
        Args:
            config_path: Path to save (uses original path if None)
        """
        path = config_path or self._config_path
        if not path:
            raise ValueError("No config path specified")
            
        with open(path, 'w') as f:
            yaml.dump(self._data, f, default_flow_style=False)
            
    def to_dict(self) -> Dict[str, Any]:
        """Get configuration as dictionary"""
        return self._data.copy()


def load_config(
    config_path: Optional[str] = None,
    default_paths: List[str] = None
) -> Config:
    """
    Load configuration from file
    
    Args:
        config_path: Explicit config path
        default_paths: List of default paths to search
        
    Returns:
        Config instance
    """
    if default_paths is None:
        default_paths = [
            "config/robot_config.yaml",
            "robot_config.yaml",
            "/etc/robotwin/config.yaml",
            os.path.expanduser("~/.robotwin/config.yaml")
        ]
        
    # Check explicit path
    if config_path:
        if os.path.exists(config_path):
            return Config(config_path)
        raise FileNotFoundError(f"Config file not found: {config_path}")
        
    # Search default paths
    for path in default_paths:
        if os.path.exists(path):
            return Config(path)
            
    # Return default config
    return Config()


def create_default_config(output_path: str) -> None:
    """
    Create default configuration file
    
    Args:
        output_path: Path to create config file
    """
    config = RobotConfig()
    
    # Convert to dictionary
    data = config.dict()
    
    # Write to file
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
