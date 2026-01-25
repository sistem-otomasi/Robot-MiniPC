"""
RoboTwin Hardware - Sensor Manager and Sensor Implementations
Supports: LiDAR, Camera, IMU, Encoders, Battery, Ultrasonic
"""

import asyncio
import struct
import time
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List, Tuple, Callable
from enum import Enum

from robotwin.utils.logger import get_logger
from robotwin.core.state import LaserScan, IMUState, BatteryState

logger = get_logger(__name__)


# ============================================================================
# Sensor Base Classes
# ============================================================================

class SensorStatus(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


@dataclass
class SensorData:
    """Base sensor data"""
    timestamp: float = field(default_factory=time.time)
    valid: bool = True


class SensorBase(ABC):
    """Abstract base class for all sensors"""
    
    def __init__(self, name: str, frame_id: str = "sensor"):
        self.name = name
        self.frame_id = frame_id
        self.status = SensorStatus.DISCONNECTED
        self._running = False
        self._callbacks: List[Callable] = []
        self._last_data: Optional[SensorData] = None
        
    @abstractmethod
    async def connect(self) -> bool:
        """Connect to sensor hardware"""
        pass
    
    @abstractmethod
    async def disconnect(self) -> None:
        """Disconnect from sensor"""
        pass
    
    @abstractmethod
    async def read(self) -> Optional[SensorData]:
        """Read sensor data"""
        pass
    
    def add_callback(self, callback: Callable) -> None:
        """Add data callback"""
        self._callbacks.append(callback)
        
    def remove_callback(self, callback: Callable) -> None:
        """Remove data callback"""
        if callback in self._callbacks:
            self._callbacks.remove(callback)
            
    async def _notify_callbacks(self, data: SensorData) -> None:
        """Notify all callbacks with new data"""
        self._last_data = data
        for callback in self._callbacks:
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(data)
                else:
                    callback(data)
            except Exception as e:
                logger.error(f"Sensor callback error: {e}")
                
    def get_last_data(self) -> Optional[SensorData]:
        """Get last received data"""
        return self._last_data


# ============================================================================
# LiDAR Sensor
# ============================================================================

@dataclass
class LidarData(SensorData):
    """LiDAR scan data"""
    ranges: List[float] = field(default_factory=list)
    intensities: List[float] = field(default_factory=list)
    angle_min: float = 0.0
    angle_max: float = 2 * math.pi
    angle_increment: float = 0.0
    range_min: float = 0.1
    range_max: float = 12.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "ranges": self.ranges,
            "intensities": self.intensities,
            "angle_min": self.angle_min,
            "angle_max": self.angle_max,
            "angle_increment": self.angle_increment,
            "range_min": self.range_min,
            "range_max": self.range_max,
            "timestamp": self.timestamp
        }


class LidarSensor(SensorBase):
    """
    LiDAR sensor implementation
    Supports: RPLidar, YDLidar, Hokuyo
    """
    
    def __init__(
        self,
        name: str = "lidar",
        frame_id: str = "laser",
        lidar_type: str = "rplidar",
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200
    ):
        super().__init__(name, frame_id)
        self.lidar_type = lidar_type.lower()
        self.port = port
        self.baudrate = baudrate
        self._lidar = None
        self._read_task: Optional[asyncio.Task] = None
        
    async def connect(self) -> bool:
        """Connect to LiDAR"""
        self.status = SensorStatus.CONNECTING
        
        try:
            if self.lidar_type == "rplidar":
                from rplidar import RPLidar
                self._lidar = RPLidar(self.port, baudrate=self.baudrate)
                self._lidar.connect()
                self._lidar.start_motor()
                
            elif self.lidar_type == "ydlidar":
                # YDLidar SDK integration
                import ydlidar
                self._lidar = ydlidar.CYdLidar()
                self._lidar.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
                self._lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.baudrate)
                self._lidar.initialize()
                self._lidar.turnOn()
                
            self.status = SensorStatus.CONNECTED
            self._running = True
            self._read_task = asyncio.create_task(self._read_loop())
            
            logger.info(f"LiDAR connected: {self.lidar_type} on {self.port}")
            return True
            
        except Exception as e:
            logger.error(f"LiDAR connection failed: {e}")
            self.status = SensorStatus.ERROR
            return False
            
    async def disconnect(self) -> None:
        """Disconnect LiDAR"""
        self._running = False
        
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
                
        if self._lidar:
            try:
                if self.lidar_type == "rplidar":
                    self._lidar.stop_motor()
                    self._lidar.disconnect()
                elif self.lidar_type == "ydlidar":
                    self._lidar.turnOff()
                    self._lidar.disconnecting()
            except Exception as e:
                logger.error(f"LiDAR disconnect error: {e}")
                
        self.status = SensorStatus.DISCONNECTED
        logger.info("LiDAR disconnected")
        
    async def _read_loop(self) -> None:
        """Continuous read loop"""
        while self._running:
            try:
                data = await self.read()
                if data:
                    await self._notify_callbacks(data)
            except Exception as e:
                logger.error(f"LiDAR read error: {e}")
            await asyncio.sleep(0.01)  # ~100Hz max
            
    async def read(self) -> Optional[LidarData]:
        """Read single scan"""
        if not self._lidar:
            return None
            
        try:
            if self.lidar_type == "rplidar":
                # RPLidar returns iterator
                scan = next(self._lidar.iter_scans(max_buf_meas=500))
                
                ranges = []
                intensities = []
                
                # Sort by angle
                scan = sorted(scan, key=lambda x: x[1])
                
                angle_min = math.radians(scan[0][1]) if scan else 0.0
                angle_max = math.radians(scan[-1][1]) if scan else 2 * math.pi
                
                for quality, angle, distance in scan:
                    ranges.append(distance / 1000.0)  # Convert to meters
                    intensities.append(float(quality))
                    
                angle_increment = (angle_max - angle_min) / len(ranges) if ranges else 0.0
                
                return LidarData(
                    ranges=ranges,
                    intensities=intensities,
                    angle_min=angle_min,
                    angle_max=angle_max,
                    angle_increment=angle_increment,
                    range_min=0.15,
                    range_max=12.0,
                    timestamp=time.time()
                )
                
        except StopIteration:
            return None
        except Exception as e:
            logger.debug(f"LiDAR read error: {e}")
            return None


class MockLidarSensor(SensorBase):
    """Mock LiDAR for testing"""
    
    def __init__(self, name: str = "mock_lidar", frame_id: str = "laser"):
        super().__init__(name, frame_id)
        self._read_task: Optional[asyncio.Task] = None
        
    async def connect(self) -> bool:
        self.status = SensorStatus.CONNECTED
        self._running = True
        self._read_task = asyncio.create_task(self._read_loop())
        logger.info("Mock LiDAR connected")
        return True
        
    async def disconnect(self) -> None:
        self._running = False
        if self._read_task:
            self._read_task.cancel()
        self.status = SensorStatus.DISCONNECTED
        
    async def _read_loop(self) -> None:
        while self._running:
            data = await self.read()
            if data:
                await self._notify_callbacks(data)
            await asyncio.sleep(0.1)  # 10Hz
            
    async def read(self) -> Optional[LidarData]:
        """Generate mock scan data"""
        num_points = 360
        ranges = []
        
        for i in range(num_points):
            angle = math.radians(i)
            # Simulate room with some objects
            base_dist = 3.0 + 0.5 * math.sin(angle * 4)
            ranges.append(base_dist)
            
        return LidarData(
            ranges=ranges,
            intensities=[50.0] * num_points,
            angle_min=0.0,
            angle_max=2 * math.pi,
            angle_increment=2 * math.pi / num_points,
            range_min=0.1,
            range_max=12.0,
            timestamp=time.time()
        )


# ============================================================================
# Camera Sensor
# ============================================================================

@dataclass
class CameraData(SensorData):
    """Camera frame data"""
    frame: Any = None  # numpy array
    width: int = 640
    height: int = 480
    encoding: str = "bgr8"
    compressed: Optional[bytes] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "width": self.width,
            "height": self.height,
            "encoding": self.encoding,
            "has_frame": self.frame is not None,
            "timestamp": self.timestamp
        }


class CameraSensor(SensorBase):
    """
    Camera sensor implementation
    Supports: USB cameras, CSI cameras, RTSP streams
    """
    
    def __init__(
        self,
        name: str = "camera",
        frame_id: str = "camera",
        camera_type: str = "usb",
        device_id: int = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        rtsp_url: Optional[str] = None
    ):
        super().__init__(name, frame_id)
        self.camera_type = camera_type.lower()
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.rtsp_url = rtsp_url
        self._cap = None
        self._read_task: Optional[asyncio.Task] = None
        
    async def connect(self) -> bool:
        """Connect to camera"""
        self.status = SensorStatus.CONNECTING
        
        try:
            import cv2
            
            if self.camera_type == "usb":
                self._cap = cv2.VideoCapture(self.device_id)
            elif self.camera_type == "csi":
                # GStreamer pipeline for CSI cameras (Jetson/Pi)
                gst_pipeline = (
                    f"nvarguscamerasrc sensor-id={self.device_id} ! "
                    f"video/x-raw(memory:NVMM), width={self.width}, height={self.height}, "
                    f"framerate={self.fps}/1 ! nvvidconv ! video/x-raw, format=BGRx ! "
                    f"videoconvert ! video/x-raw, format=BGR ! appsink"
                )
                self._cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            elif self.camera_type == "rtsp":
                self._cap = cv2.VideoCapture(self.rtsp_url)
                
            if self._cap and self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self._cap.set(cv2.CAP_PROP_FPS, self.fps)
                
                self.status = SensorStatus.CONNECTED
                self._running = True
                self._read_task = asyncio.create_task(self._read_loop())
                
                logger.info(f"Camera connected: {self.camera_type}")
                return True
            else:
                raise Exception("Failed to open camera")
                
        except Exception as e:
            logger.error(f"Camera connection failed: {e}")
            self.status = SensorStatus.ERROR
            return False
            
    async def disconnect(self) -> None:
        """Disconnect camera"""
        self._running = False
        
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
                
        if self._cap:
            self._cap.release()
            
        self.status = SensorStatus.DISCONNECTED
        logger.info("Camera disconnected")
        
    async def _read_loop(self) -> None:
        """Continuous read loop"""
        interval = 1.0 / self.fps
        while self._running:
            try:
                data = await self.read()
                if data and data.valid:
                    await self._notify_callbacks(data)
            except Exception as e:
                logger.error(f"Camera read error: {e}")
            await asyncio.sleep(interval)
            
    async def read(self) -> Optional[CameraData]:
        """Read single frame"""
        if not self._cap:
            return None
            
        try:
            ret, frame = self._cap.read()
            
            if ret:
                return CameraData(
                    frame=frame,
                    width=frame.shape[1],
                    height=frame.shape[0],
                    encoding="bgr8",
                    timestamp=time.time()
                )
            else:
                return CameraData(valid=False)
                
        except Exception as e:
            logger.debug(f"Camera read error: {e}")
            return None
            
    async def get_compressed_frame(
        self,
        quality: int = 80
    ) -> Optional[bytes]:
        """Get JPEG compressed frame"""
        data = await self.read()
        
        if data and data.frame is not None:
            import cv2
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
            _, compressed = cv2.imencode('.jpg', data.frame, encode_params)
            return compressed.tobytes()
            
        return None


# ============================================================================
# IMU Sensor
# ============================================================================

@dataclass
class IMUData(SensorData):
    """IMU sensor data"""
    # Linear acceleration (m/s^2)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 9.81
    
    # Angular velocity (rad/s)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    
    # Magnetometer (optional, uT)
    mag_x: Optional[float] = None
    mag_y: Optional[float] = None
    mag_z: Optional[float] = None
    
    # Orientation quaternion (from fusion)
    quat_w: float = 1.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    
    # Euler angles (from fusion, radians)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    # Temperature
    temperature: Optional[float] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "acceleration": {"x": self.accel_x, "y": self.accel_y, "z": self.accel_z},
            "angular_velocity": {"x": self.gyro_x, "y": self.gyro_y, "z": self.gyro_z},
            "orientation": {"w": self.quat_w, "x": self.quat_x, "y": self.quat_y, "z": self.quat_z},
            "euler": {"roll": self.roll, "pitch": self.pitch, "yaw": self.yaw},
            "timestamp": self.timestamp
        }


class IMUSensor(SensorBase):
    """
    IMU sensor implementation
    Supports: MPU6050, MPU9250, BNO055
    """
    
    def __init__(
        self,
        name: str = "imu",
        frame_id: str = "imu",
        imu_type: str = "mpu6050",
        i2c_bus: int = 1,
        address: int = 0x68,
        publish_frequency: int = 100
    ):
        super().__init__(name, frame_id)
        self.imu_type = imu_type.lower()
        self.i2c_bus = i2c_bus
        self.address = address
        self.publish_frequency = publish_frequency
        self._imu = None
        self._read_task: Optional[asyncio.Task] = None
        
        # Calibration offsets
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.accel_offset = [0.0, 0.0, 0.0]
        
        # Complementary filter state
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._last_time = None
        self._alpha = 0.98  # Complementary filter weight
        
    async def connect(self) -> bool:
        """Connect to IMU"""
        self.status = SensorStatus.CONNECTING
        
        try:
            if self.imu_type == "mpu6050":
                import smbus2
                self._smbus = smbus2.SMBus(self.i2c_bus)
                # Wake up MPU6050
                self._smbus.write_byte_data(self.address, 0x6B, 0x00)
                # Set gyro range to ±250°/s
                self._smbus.write_byte_data(self.address, 0x1B, 0x00)
                # Set accel range to ±2g
                self._smbus.write_byte_data(self.address, 0x1C, 0x00)
                
            elif self.imu_type == "bno055":
                import adafruit_bno055
                import board
                i2c = board.I2C()
                self._imu = adafruit_bno055.BNO055_I2C(i2c)
                
            self.status = SensorStatus.CONNECTED
            self._running = True
            self._read_task = asyncio.create_task(self._read_loop())
            
            logger.info(f"IMU connected: {self.imu_type}")
            return True
            
        except Exception as e:
            logger.error(f"IMU connection failed: {e}")
            self.status = SensorStatus.ERROR
            return False
            
    async def disconnect(self) -> None:
        """Disconnect IMU"""
        self._running = False
        
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
                
        if hasattr(self, '_smbus'):
            self._smbus.close()
            
        self.status = SensorStatus.DISCONNECTED
        logger.info("IMU disconnected")
        
    async def _read_loop(self) -> None:
        """Continuous read loop"""
        interval = 1.0 / self.publish_frequency
        while self._running:
            try:
                data = await self.read()
                if data and data.valid:
                    await self._notify_callbacks(data)
            except Exception as e:
                logger.error(f"IMU read error: {e}")
            await asyncio.sleep(interval)
            
    async def read(self) -> Optional[IMUData]:
        """Read IMU data"""
        try:
            if self.imu_type == "mpu6050":
                return await self._read_mpu6050()
            elif self.imu_type == "bno055":
                return await self._read_bno055()
        except Exception as e:
            logger.debug(f"IMU read error: {e}")
            return None
            
    async def _read_mpu6050(self) -> Optional[IMUData]:
        """Read data from MPU6050"""
        # Read raw data
        raw_data = self._smbus.read_i2c_block_data(self.address, 0x3B, 14)
        
        # Parse accelerometer (16-bit signed, ±2g range -> 16384 LSB/g)
        accel_x = self._parse_int16(raw_data[0], raw_data[1]) / 16384.0 * 9.81
        accel_y = self._parse_int16(raw_data[2], raw_data[3]) / 16384.0 * 9.81
        accel_z = self._parse_int16(raw_data[4], raw_data[5]) / 16384.0 * 9.81
        
        # Parse temperature
        temp_raw = self._parse_int16(raw_data[6], raw_data[7])
        temperature = temp_raw / 340.0 + 36.53
        
        # Parse gyroscope (16-bit signed, ±250°/s range -> 131 LSB/°/s)
        gyro_x = math.radians(self._parse_int16(raw_data[8], raw_data[9]) / 131.0)
        gyro_y = math.radians(self._parse_int16(raw_data[10], raw_data[11]) / 131.0)
        gyro_z = math.radians(self._parse_int16(raw_data[12], raw_data[13]) / 131.0)
        
        # Apply calibration offsets
        accel_x -= self.accel_offset[0]
        accel_y -= self.accel_offset[1]
        accel_z -= self.accel_offset[2]
        gyro_x -= self.gyro_offset[0]
        gyro_y -= self.gyro_offset[1]
        gyro_z -= self.gyro_offset[2]
        
        # Complementary filter for orientation
        current_time = time.time()
        if self._last_time is not None:
            dt = current_time - self._last_time
            
            # Calculate roll and pitch from accelerometer
            accel_roll = math.atan2(accel_y, accel_z)
            accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
            
            # Integrate gyroscope
            self._roll += gyro_x * dt
            self._pitch += gyro_y * dt
            self._yaw += gyro_z * dt
            
            # Complementary filter fusion
            self._roll = self._alpha * self._roll + (1 - self._alpha) * accel_roll
            self._pitch = self._alpha * self._pitch + (1 - self._alpha) * accel_pitch
            
        self._last_time = current_time
        
        # Convert to quaternion
        qw, qx, qy, qz = self._euler_to_quaternion(self._roll, self._pitch, self._yaw)
        
        return IMUData(
            accel_x=accel_x, accel_y=accel_y, accel_z=accel_z,
            gyro_x=gyro_x, gyro_y=gyro_y, gyro_z=gyro_z,
            roll=self._roll, pitch=self._pitch, yaw=self._yaw,
            quat_w=qw, quat_x=qx, quat_y=qy, quat_z=qz,
            temperature=temperature,
            timestamp=current_time
        )
        
    async def _read_bno055(self) -> Optional[IMUData]:
        """Read data from BNO055"""
        if not self._imu:
            return None
            
        # BNO055 provides fused orientation
        accel = self._imu.acceleration
        gyro = self._imu.gyro
        quat = self._imu.quaternion
        euler = self._imu.euler
        
        return IMUData(
            accel_x=accel[0] if accel else 0,
            accel_y=accel[1] if accel else 0,
            accel_z=accel[2] if accel else 9.81,
            gyro_x=gyro[0] if gyro else 0,
            gyro_y=gyro[1] if gyro else 0,
            gyro_z=gyro[2] if gyro else 0,
            quat_w=quat[0] if quat else 1,
            quat_x=quat[1] if quat else 0,
            quat_y=quat[2] if quat else 0,
            quat_z=quat[3] if quat else 0,
            yaw=math.radians(euler[0]) if euler else 0,
            roll=math.radians(euler[1]) if euler else 0,
            pitch=math.radians(euler[2]) if euler else 0,
            temperature=self._imu.temperature,
            timestamp=time.time()
        )
        
    @staticmethod
    def _parse_int16(high: int, low: int) -> int:
        """Parse 16-bit signed integer"""
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value
        
    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """Convert Euler angles to quaternion"""
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qw, qx, qy, qz
        
    async def calibrate(self, samples: int = 200) -> None:
        """Calibrate gyroscope offsets"""
        logger.info("Calibrating IMU... Keep robot stationary")
        
        gyro_sum = [0.0, 0.0, 0.0]
        accel_sum = [0.0, 0.0, 0.0]
        
        for _ in range(samples):
            data = await self.read()
            if data:
                gyro_sum[0] += data.gyro_x
                gyro_sum[1] += data.gyro_y
                gyro_sum[2] += data.gyro_z
                accel_sum[0] += data.accel_x
                accel_sum[1] += data.accel_y
                accel_sum[2] += data.accel_z - 9.81  # Subtract gravity
            await asyncio.sleep(0.01)
            
        self.gyro_offset = [g / samples for g in gyro_sum]
        self.accel_offset = [a / samples for a in accel_sum]
        
        logger.info(f"IMU calibrated. Gyro offset: {self.gyro_offset}")


# ============================================================================
# Encoder Sensor
# ============================================================================

@dataclass
class EncoderData(SensorData):
    """Encoder data"""
    count: int = 0
    velocity: float = 0.0  # rad/s
    position: float = 0.0  # radians
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "count": self.count,
            "velocity": self.velocity,
            "position": self.position,
            "timestamp": self.timestamp
        }


class EncoderSensor(SensorBase):
    """
    Quadrature encoder sensor using GPIO
    """
    
    def __init__(
        self,
        name: str = "encoder",
        frame_id: str = "encoder",
        pin_a: int = 17,
        pin_b: int = 27,
        ppr: int = 1200,
        gear_ratio: float = 1.0
    ):
        super().__init__(name, frame_id)
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ppr = ppr  # Pulses per revolution
        self.gear_ratio = gear_ratio
        
        self._count = 0
        self._prev_count = 0
        self._prev_time = time.time()
        self._gpio = None
        
    async def connect(self) -> bool:
        """Setup encoder GPIO"""
        try:
            import RPi.GPIO as GPIO
            self._gpio = GPIO
            
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Setup interrupt on pin A
            GPIO.add_event_detect(
                self.pin_a,
                GPIO.BOTH,
                callback=self._encoder_callback
            )
            
            self.status = SensorStatus.CONNECTED
            self._running = True
            logger.info(f"Encoder connected on pins {self.pin_a}, {self.pin_b}")
            return True
            
        except Exception as e:
            logger.error(f"Encoder connection failed: {e}")
            self.status = SensorStatus.ERROR
            return False
            
    async def disconnect(self) -> None:
        """Cleanup encoder GPIO"""
        if self._gpio:
            self._gpio.remove_event_detect(self.pin_a)
            self._gpio.cleanup([self.pin_a, self.pin_b])
        self.status = SensorStatus.DISCONNECTED
        
    def _encoder_callback(self, channel) -> None:
        """GPIO interrupt callback for encoder"""
        if self._gpio:
            a = self._gpio.input(self.pin_a)
            b = self._gpio.input(self.pin_b)
            
            if a == b:
                self._count += 1
            else:
                self._count -= 1
                
    async def read(self) -> Optional[EncoderData]:
        """Read encoder data"""
        current_time = time.time()
        dt = current_time - self._prev_time
        
        if dt > 0:
            # Calculate velocity
            delta = self._count - self._prev_count
            counts_per_rad = (self.ppr * self.gear_ratio * 4) / (2 * math.pi)  # x4 for quadrature
            velocity = delta / counts_per_rad / dt
            position = self._count / counts_per_rad
            
            self._prev_count = self._count
            self._prev_time = current_time
            
            return EncoderData(
                count=self._count,
                velocity=velocity,
                position=position,
                timestamp=current_time
            )
            
        return None
        
    def reset(self) -> None:
        """Reset encoder count"""
        self._count = 0
        self._prev_count = 0


# ============================================================================
# Battery Sensor
# ============================================================================

@dataclass  
class BatteryData(SensorData):
    """Battery state data"""
    voltage: float = 0.0
    current: float = 0.0
    percentage: float = 0.0
    charging: bool = False
    temperature: Optional[float] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "voltage": self.voltage,
            "current": self.current,
            "percentage": self.percentage,
            "charging": self.charging,
            "temperature": self.temperature,
            "timestamp": self.timestamp
        }


class BatterySensor(SensorBase):
    """
    Battery monitoring sensor
    Supports ADC-based voltage monitoring
    """
    
    def __init__(
        self,
        name: str = "battery",
        frame_id: str = "battery",
        adc_channel: int = 0,
        voltage_divider_ratio: float = 5.7,
        min_voltage: float = 10.0,
        max_voltage: float = 12.6
    ):
        super().__init__(name, frame_id)
        self.adc_channel = adc_channel
        self.voltage_divider_ratio = voltage_divider_ratio
        self.min_voltage = min_voltage
        self.max_voltage = max_voltage
        self._adc = None
        self._read_task: Optional[asyncio.Task] = None
        
    async def connect(self) -> bool:
        """Connect to ADC"""
        try:
            # Try ADS1115 ADC
            import board
            import busio
            import adafruit_ads1x15.ads1115 as ADS
            from adafruit_ads1x15.analog_in import AnalogIn
            
            i2c = busio.I2C(board.SCL, board.SDA)
            ads = ADS.ADS1115(i2c)
            self._adc = AnalogIn(ads, self.adc_channel)
            
            self.status = SensorStatus.CONNECTED
            self._running = True
            self._read_task = asyncio.create_task(self._read_loop())
            
            logger.info("Battery sensor connected")
            return True
            
        except Exception as e:
            logger.warning(f"ADC not available, using mock: {e}")
            # Use mock values
            self.status = SensorStatus.CONNECTED
            self._running = True
            self._read_task = asyncio.create_task(self._read_loop())
            return True
            
    async def disconnect(self) -> None:
        """Disconnect battery sensor"""
        self._running = False
        if self._read_task:
            self._read_task.cancel()
        self.status = SensorStatus.DISCONNECTED
        
    async def _read_loop(self) -> None:
        """Read battery at 1Hz"""
        while self._running:
            try:
                data = await self.read()
                if data:
                    await self._notify_callbacks(data)
            except Exception as e:
                logger.error(f"Battery read error: {e}")
            await asyncio.sleep(1.0)
            
    async def read(self) -> Optional[BatteryData]:
        """Read battery voltage"""
        try:
            if self._adc:
                raw_voltage = self._adc.voltage
                voltage = raw_voltage * self.voltage_divider_ratio
            else:
                # Mock value
                voltage = 11.8
                
            percentage = self._calculate_percentage(voltage)
            
            return BatteryData(
                voltage=voltage,
                percentage=percentage,
                timestamp=time.time()
            )
            
        except Exception as e:
            logger.debug(f"Battery read error: {e}")
            return None
            
    def _calculate_percentage(self, voltage: float) -> float:
        """Calculate battery percentage from voltage"""
        if voltage >= self.max_voltage:
            return 100.0
        if voltage <= self.min_voltage:
            return 0.0
            
        # Linear interpolation (simplification, real curve is non-linear)
        return ((voltage - self.min_voltage) / 
                (self.max_voltage - self.min_voltage) * 100.0)


# ============================================================================
# Sensor Manager
# ============================================================================

class SensorManager:
    """
    Manages all robot sensors
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.sensors: Dict[str, SensorBase] = {}
        self._running = False
        
    async def initialize(self) -> None:
        """Initialize all configured sensors"""
        sensor_config = self.config.get("sensors", {})
        
        # LiDAR
        if sensor_config.get("lidar", {}).get("enabled", False):
            lidar_cfg = sensor_config["lidar"]
            self.sensors["lidar"] = LidarSensor(
                name="lidar",
                frame_id=lidar_cfg.get("frame_id", "laser"),
                lidar_type=lidar_cfg.get("type", "rplidar"),
                port=lidar_cfg.get("port", "/dev/ttyUSB1"),
                baudrate=lidar_cfg.get("baudrate", 115200)
            )
            
        # Camera
        if sensor_config.get("camera", {}).get("enabled", False):
            cam_cfg = sensor_config["camera"]
            self.sensors["camera"] = CameraSensor(
                name="camera",
                frame_id=cam_cfg.get("frame_id", "camera"),
                camera_type=cam_cfg.get("type", "usb"),
                device_id=cam_cfg.get("device_id", 0),
                width=cam_cfg.get("width", 640),
                height=cam_cfg.get("height", 480),
                fps=cam_cfg.get("fps", 30)
            )
            
        # IMU
        if sensor_config.get("imu", {}).get("enabled", False):
            imu_cfg = sensor_config["imu"]
            self.sensors["imu"] = IMUSensor(
                name="imu",
                frame_id=imu_cfg.get("frame_id", "imu"),
                imu_type=imu_cfg.get("type", "mpu6050"),
                i2c_bus=imu_cfg.get("i2c_bus", 1),
                address=imu_cfg.get("address", 0x68),
                publish_frequency=imu_cfg.get("publish_frequency", 100)
            )
            
        # Battery
        if sensor_config.get("battery", {}).get("enabled", False):
            bat_cfg = sensor_config["battery"]
            self.sensors["battery"] = BatterySensor(
                name="battery",
                adc_channel=bat_cfg.get("adc_channel", 0),
                voltage_divider_ratio=bat_cfg.get("voltage_divider_ratio", 5.7),
                min_voltage=bat_cfg.get("min_voltage", 10.0),
                max_voltage=bat_cfg.get("max_voltage", 12.6)
            )
            
        logger.info(f"Initialized {len(self.sensors)} sensors")
        
    async def connect_all(self) -> Dict[str, bool]:
        """Connect all sensors"""
        results = {}
        
        for name, sensor in self.sensors.items():
            results[name] = await sensor.connect()
            
        self._running = True
        return results
        
    async def disconnect_all(self) -> None:
        """Disconnect all sensors"""
        self._running = False
        
        for sensor in self.sensors.values():
            await sensor.disconnect()
            
    def get_sensor(self, name: str) -> Optional[SensorBase]:
        """Get sensor by name"""
        return self.sensors.get(name)
        
    def add_callback(self, sensor_name: str, callback: Callable) -> bool:
        """Add callback to sensor"""
        sensor = self.sensors.get(sensor_name)
        if sensor:
            sensor.add_callback(callback)
            return True
        return False
        
    def get_all_status(self) -> Dict[str, str]:
        """Get status of all sensors"""
        return {
            name: sensor.status.value
            for name, sensor in self.sensors.items()
        }
