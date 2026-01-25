"""
RoboTwin Hardware - Motor Controllers
Supports various motor drivers: Serial, I2C, CAN, GPIO PWM
"""

import asyncio
import struct
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Tuple, List
from enum import Enum
import time

from robotwin.utils.logger import get_logger
from robotwin.control.pid import PIDController

logger = get_logger(__name__)


class MotorDriverType(Enum):
    SERIAL = "serial"
    I2C = "i2c"
    CAN = "can"
    GPIO = "gpio"
    MOCK = "mock"


@dataclass
class MotorConfig:
    """Motor configuration"""
    id: int
    reversed: bool = False
    encoder_ppr: int = 1200  # Pulses per revolution
    gear_ratio: float = 30.0
    max_rpm: float = 300.0
    max_pwm: int = 255
    

@dataclass
class MotorState:
    """Motor state"""
    id: int
    position: float = 0.0  # radians
    velocity: float = 0.0  # rad/s
    current: float = 0.0   # amps
    pwm: int = 0
    encoder_count: int = 0
    target_velocity: float = 0.0
    timestamp: float = field(default_factory=time.time)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "position": self.position,
            "velocity": self.velocity,
            "current": self.current,
            "pwm": self.pwm,
            "encoder_count": self.encoder_count,
            "target_velocity": self.target_velocity,
            "timestamp": self.timestamp
        }


class MotorController(ABC):
    """Abstract base class for motor controllers"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.motors: Dict[int, MotorConfig] = {}
        self.states: Dict[int, MotorState] = {}
        self.pids: Dict[int, PIDController] = {}
        self._running = False
        self._lock = asyncio.Lock()
        
    @abstractmethod
    async def connect(self) -> bool:
        """Connect to motor driver"""
        pass
    
    @abstractmethod
    async def disconnect(self) -> None:
        """Disconnect from motor driver"""
        pass
    
    @abstractmethod
    async def set_velocity(self, motor_id: int, velocity: float) -> None:
        """Set motor velocity in rad/s"""
        pass
    
    @abstractmethod
    async def set_pwm(self, motor_id: int, pwm: int) -> None:
        """Set motor PWM directly (-255 to 255)"""
        pass
    
    @abstractmethod
    async def get_encoder(self, motor_id: int) -> int:
        """Get encoder count"""
        pass
    
    @abstractmethod
    async def reset_encoder(self, motor_id: int) -> None:
        """Reset encoder count to zero"""
        pass
    
    def add_motor(self, motor_id: int, config: MotorConfig) -> None:
        """Add motor configuration"""
        self.motors[motor_id] = config
        self.states[motor_id] = MotorState(id=motor_id)
        
        # Create PID controller for velocity control
        pid_config = self.config.get("velocity_pid", {})
        self.pids[motor_id] = PIDController(
            kp=pid_config.get("kp", 1.0),
            ki=pid_config.get("ki", 0.1),
            kd=pid_config.get("kd", 0.01),
            output_min=pid_config.get("output_min", -255),
            output_max=pid_config.get("output_max", 255)
        )
        logger.debug(f"Added motor {motor_id} with config: {config}")
        
    def get_state(self, motor_id: int) -> Optional[MotorState]:
        """Get motor state"""
        return self.states.get(motor_id)
    
    def get_all_states(self) -> Dict[int, MotorState]:
        """Get all motor states"""
        return self.states.copy()
    
    async def stop_all(self) -> None:
        """Emergency stop all motors"""
        logger.warning("Emergency stop - stopping all motors")
        for motor_id in self.motors:
            await self.set_pwm(motor_id, 0)
            if motor_id in self.states:
                self.states[motor_id].target_velocity = 0.0
                
    async def update_velocities(self) -> None:
        """Update motor velocities based on PID control"""
        for motor_id, motor_config in self.motors.items():
            state = self.states.get(motor_id)
            pid = self.pids.get(motor_id)
            
            if state and pid:
                # Calculate PWM from PID
                error = state.target_velocity - state.velocity
                output = pid.update(error, state.timestamp)
                
                # Apply direction reversal if configured
                if motor_config.reversed:
                    output = -output
                    
                await self.set_pwm(motor_id, int(output))


class SerialMotorController(MotorController):
    """Motor controller using serial communication"""
    
    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.serial_config = config.get("serial", {})
        self.port = self.serial_config.get("port", "/dev/ttyUSB0")
        self.baudrate = self.serial_config.get("baudrate", 115200)
        self._serial = None
        self._read_task: Optional[asyncio.Task] = None
        
    async def connect(self) -> bool:
        """Connect to serial motor driver"""
        try:
            import serial_asyncio
            
            self._reader, self._writer = await serial_asyncio.open_serial_connection(
                url=self.port,
                baudrate=self.baudrate
            )
            
            self._running = True
            self._read_task = asyncio.create_task(self._read_loop())
            
            logger.info(f"Connected to motor driver on {self.port}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to motor driver: {e}")
            return False
            
    async def disconnect(self) -> None:
        """Disconnect from serial motor driver"""
        self._running = False
        
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
                
        if self._writer:
            self._writer.close()
            
        logger.info("Disconnected from motor driver")
        
    async def _read_loop(self) -> None:
        """Read encoder data from motor driver"""
        while self._running:
            try:
                line = await asyncio.wait_for(
                    self._reader.readline(),
                    timeout=1.0
                )
                await self._parse_response(line)
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                logger.error(f"Serial read error: {e}")
                await asyncio.sleep(0.1)
                
    async def _parse_response(self, data: bytes) -> None:
        """Parse response from motor driver"""
        try:
            # Protocol: "E<id>:<count>\n" for encoder
            # Protocol: "V<id>:<velocity>\n" for velocity
            text = data.decode().strip()
            
            if text.startswith("E"):
                parts = text[1:].split(":")
                motor_id = int(parts[0])
                count = int(parts[1])
                
                if motor_id in self.states:
                    state = self.states[motor_id]
                    motor_config = self.motors.get(motor_id)
                    
                    if motor_config:
                        # Calculate velocity from encoder change
                        dt = time.time() - state.timestamp
                        if dt > 0:
                            delta_count = count - state.encoder_count
                            # Convert to rad/s
                            counts_per_rad = (motor_config.encoder_ppr * motor_config.gear_ratio) / (2 * 3.14159)
                            state.velocity = delta_count / counts_per_rad / dt
                            
                        state.encoder_count = count
                        state.position = count / ((motor_config.encoder_ppr * motor_config.gear_ratio) / (2 * 3.14159))
                        state.timestamp = time.time()
                        
        except Exception as e:
            logger.debug(f"Failed to parse motor response: {e}")
            
    async def set_velocity(self, motor_id: int, velocity: float) -> None:
        """Set target velocity for motor"""
        if motor_id in self.states:
            self.states[motor_id].target_velocity = velocity
            
    async def set_pwm(self, motor_id: int, pwm: int) -> None:
        """Set motor PWM"""
        pwm = max(-255, min(255, pwm))
        
        if motor_id in self.states:
            self.states[motor_id].pwm = pwm
            
        if self._writer:
            # Protocol: "M<id>:<pwm>\n"
            command = f"M{motor_id}:{pwm}\n"
            self._writer.write(command.encode())
            await self._writer.drain()
            
    async def get_encoder(self, motor_id: int) -> int:
        """Get encoder count"""
        if motor_id in self.states:
            return self.states[motor_id].encoder_count
        return 0
    
    async def reset_encoder(self, motor_id: int) -> None:
        """Reset encoder count"""
        if self._writer:
            command = f"R{motor_id}\n"
            self._writer.write(command.encode())
            await self._writer.drain()
            
        if motor_id in self.states:
            self.states[motor_id].encoder_count = 0
            self.states[motor_id].position = 0.0


class I2CMotorController(MotorController):
    """Motor controller using I2C communication"""
    
    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.i2c_config = config.get("i2c", {})
        self.bus = self.i2c_config.get("bus", 1)
        self.address = self.i2c_config.get("address", 0x10)
        self._smbus = None
        
    async def connect(self) -> bool:
        """Connect to I2C motor driver"""
        try:
            import smbus2
            self._smbus = smbus2.SMBus(self.bus)
            logger.info(f"Connected to I2C motor driver at address 0x{self.address:02x}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to I2C motor driver: {e}")
            return False
            
    async def disconnect(self) -> None:
        """Disconnect from I2C motor driver"""
        if self._smbus:
            self._smbus.close()
        logger.info("Disconnected from I2C motor driver")
        
    async def set_velocity(self, motor_id: int, velocity: float) -> None:
        """Set target velocity"""
        if motor_id in self.states:
            self.states[motor_id].target_velocity = velocity
            
    async def set_pwm(self, motor_id: int, pwm: int) -> None:
        """Set motor PWM via I2C"""
        pwm = max(-255, min(255, pwm))
        
        if motor_id in self.states:
            self.states[motor_id].pwm = pwm
            
        if self._smbus:
            try:
                # Register: 0x10 + motor_id for PWM
                # Data: [direction, abs(pwm)]
                direction = 1 if pwm >= 0 else 0
                data = [direction, abs(pwm)]
                self._smbus.write_i2c_block_data(
                    self.address,
                    0x10 + motor_id,
                    data
                )
            except Exception as e:
                logger.error(f"I2C write error: {e}")
                
    async def get_encoder(self, motor_id: int) -> int:
        """Get encoder count via I2C"""
        if self._smbus:
            try:
                # Register: 0x20 + motor_id for encoder
                data = self._smbus.read_i2c_block_data(
                    self.address,
                    0x20 + motor_id,
                    4
                )
                count = struct.unpack('<i', bytes(data))[0]
                
                if motor_id in self.states:
                    state = self.states[motor_id]
                    motor_config = self.motors.get(motor_id)
                    
                    if motor_config:
                        dt = time.time() - state.timestamp
                        if dt > 0:
                            delta = count - state.encoder_count
                            counts_per_rad = (motor_config.encoder_ppr * motor_config.gear_ratio) / (2 * 3.14159)
                            state.velocity = delta / counts_per_rad / dt
                            
                        state.encoder_count = count
                        state.timestamp = time.time()
                        
                return count
            except Exception as e:
                logger.error(f"I2C read error: {e}")
                
        return self.states.get(motor_id, MotorState(motor_id)).encoder_count
    
    async def reset_encoder(self, motor_id: int) -> None:
        """Reset encoder via I2C"""
        if self._smbus:
            try:
                self._smbus.write_byte_data(
                    self.address,
                    0x30 + motor_id,
                    1
                )
            except Exception as e:
                logger.error(f"I2C write error: {e}")
                
        if motor_id in self.states:
            self.states[motor_id].encoder_count = 0
            self.states[motor_id].position = 0.0


class CANMotorController(MotorController):
    """Motor controller using CAN bus communication"""
    
    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.can_config = config.get("can", {})
        self.interface = self.can_config.get("interface", "can0")
        self.bitrate = self.can_config.get("bitrate", 500000)
        self._bus = None
        self._read_task: Optional[asyncio.Task] = None
        
    async def connect(self) -> bool:
        """Connect to CAN bus"""
        try:
            import can
            self._bus = can.interface.Bus(
                channel=self.interface,
                bustype='socketcan'
            )
            
            self._running = True
            self._read_task = asyncio.create_task(self._read_loop())
            
            logger.info(f"Connected to CAN bus on {self.interface}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to CAN bus: {e}")
            return False
            
    async def disconnect(self) -> None:
        """Disconnect from CAN bus"""
        self._running = False
        
        if self._read_task:
            self._read_task.cancel()
            
        if self._bus:
            self._bus.shutdown()
            
        logger.info("Disconnected from CAN bus")
        
    async def _read_loop(self) -> None:
        """Read messages from CAN bus"""
        while self._running:
            try:
                if self._bus:
                    msg = self._bus.recv(timeout=0.1)
                    if msg:
                        await self._parse_can_message(msg)
            except Exception as e:
                logger.error(f"CAN read error: {e}")
                await asyncio.sleep(0.1)
                
    async def _parse_can_message(self, msg) -> None:
        """Parse CAN message"""
        try:
            # CAN ID format: 0x100 + motor_id for encoder feedback
            motor_id = msg.arbitration_id - 0x100
            
            if motor_id in self.states and len(msg.data) >= 8:
                state = self.states[motor_id]
                motor_config = self.motors.get(motor_id)
                
                # Data: [encoder_count (4 bytes), velocity (4 bytes)]
                count = struct.unpack('<i', msg.data[0:4])[0]
                velocity_raw = struct.unpack('<f', msg.data[4:8])[0]
                
                if motor_config:
                    counts_per_rad = (motor_config.encoder_ppr * motor_config.gear_ratio) / (2 * 3.14159)
                    state.position = count / counts_per_rad
                    state.velocity = velocity_raw
                    state.encoder_count = count
                    state.timestamp = time.time()
                    
        except Exception as e:
            logger.debug(f"Failed to parse CAN message: {e}")
            
    async def set_velocity(self, motor_id: int, velocity: float) -> None:
        """Set target velocity"""
        if motor_id in self.states:
            self.states[motor_id].target_velocity = velocity
            
    async def set_pwm(self, motor_id: int, pwm: int) -> None:
        """Set motor PWM via CAN"""
        pwm = max(-255, min(255, pwm))
        
        if motor_id in self.states:
            self.states[motor_id].pwm = pwm
            
        if self._bus:
            try:
                import can
                # CAN ID format: 0x200 + motor_id for motor command
                data = struct.pack('<h', pwm) + bytes(6)
                msg = can.Message(
                    arbitration_id=0x200 + motor_id,
                    data=data,
                    is_extended_id=False
                )
                self._bus.send(msg)
            except Exception as e:
                logger.error(f"CAN send error: {e}")
                
    async def get_encoder(self, motor_id: int) -> int:
        """Get encoder count"""
        if motor_id in self.states:
            return self.states[motor_id].encoder_count
        return 0
    
    async def reset_encoder(self, motor_id: int) -> None:
        """Reset encoder via CAN"""
        if self._bus:
            try:
                import can
                msg = can.Message(
                    arbitration_id=0x300 + motor_id,
                    data=bytes([1]) + bytes(7),
                    is_extended_id=False
                )
                self._bus.send(msg)
            except Exception as e:
                logger.error(f"CAN send error: {e}")
                
        if motor_id in self.states:
            self.states[motor_id].encoder_count = 0
            self.states[motor_id].position = 0.0


class MockMotorController(MotorController):
    """Mock motor controller for testing"""
    
    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self._simulation_task: Optional[asyncio.Task] = None
        
    async def connect(self) -> bool:
        """Connect mock controller"""
        self._running = True
        self._simulation_task = asyncio.create_task(self._simulate())
        logger.info("Mock motor controller connected")
        return True
        
    async def disconnect(self) -> None:
        """Disconnect mock controller"""
        self._running = False
        if self._simulation_task:
            self._simulation_task.cancel()
        logger.info("Mock motor controller disconnected")
        
    async def _simulate(self) -> None:
        """Simulate motor behavior"""
        while self._running:
            for motor_id, state in self.states.items():
                motor_config = self.motors.get(motor_id)
                if motor_config:
                    # Simulate velocity approaching target
                    state.velocity += (state.target_velocity - state.velocity) * 0.1
                    # Simulate encoder
                    dt = 0.02
                    counts_per_rad = (motor_config.encoder_ppr * motor_config.gear_ratio) / (2 * 3.14159)
                    state.encoder_count += int(state.velocity * counts_per_rad * dt)
                    state.position = state.encoder_count / counts_per_rad
                    state.timestamp = time.time()
                    
            await asyncio.sleep(0.02)
            
    async def set_velocity(self, motor_id: int, velocity: float) -> None:
        """Set target velocity"""
        if motor_id in self.states:
            self.states[motor_id].target_velocity = velocity
            
    async def set_pwm(self, motor_id: int, pwm: int) -> None:
        """Set motor PWM (simulated)"""
        pwm = max(-255, min(255, pwm))
        if motor_id in self.states:
            self.states[motor_id].pwm = pwm
            # Convert PWM to approximate velocity for simulation
            motor_config = self.motors.get(motor_id)
            if motor_config:
                max_rad_s = motor_config.max_rpm * 2 * 3.14159 / 60
                self.states[motor_id].target_velocity = (pwm / 255) * max_rad_s
                
    async def get_encoder(self, motor_id: int) -> int:
        """Get encoder count"""
        if motor_id in self.states:
            return self.states[motor_id].encoder_count
        return 0
    
    async def reset_encoder(self, motor_id: int) -> None:
        """Reset encoder"""
        if motor_id in self.states:
            self.states[motor_id].encoder_count = 0
            self.states[motor_id].position = 0.0


def create_motor_controller(config: Dict[str, Any]) -> MotorController:
    """Factory function to create motor controller"""
    driver_type = config.get("driver", "serial").lower()
    
    controllers = {
        "serial": SerialMotorController,
        "i2c": I2CMotorController,
        "can": CANMotorController,
        "mock": MockMotorController,
    }
    
    controller_class = controllers.get(driver_type, MockMotorController)
    logger.info(f"Creating motor controller: {driver_type}")
    
    return controller_class(config)
