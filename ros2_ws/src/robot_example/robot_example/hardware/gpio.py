"""
RoboTwin Hardware - GPIO Manager
Handles GPIO pin management for various hardware interfaces
"""

import asyncio
from typing import Dict, Any, Optional, Callable, List
from enum import Enum
from dataclasses import dataclass

from robot_example.utils.logger import get_logger

logger = get_logger(__name__)


class PinMode(Enum):
    INPUT = "input"
    OUTPUT = "output"
    PWM = "pwm"
    INPUT_PULLUP = "input_pullup"
    INPUT_PULLDOWN = "input_pulldown"


class PinState(Enum):
    LOW = 0
    HIGH = 1


@dataclass
class PinConfig:
    """GPIO pin configuration"""
    pin: int
    mode: PinMode
    name: str = ""
    initial_state: PinState = PinState.LOW
    pwm_frequency: int = 1000
    

class GPIOManager:
    """
    GPIO Manager for robot hardware control
    Supports: RPi.GPIO, gpiozero, or mock for testing
    """
    
    def __init__(self, use_mock: bool = False):
        self.use_mock = use_mock
        self._gpio = None
        self._pins: Dict[int, PinConfig] = {}
        self._pwm_channels: Dict[int, Any] = {}
        self._callbacks: Dict[int, List[Callable]] = {}
        self._initialized = False
        
    async def initialize(self) -> bool:
        """Initialize GPIO"""
        if self._initialized:
            return True
            
        try:
            if self.use_mock:
                logger.info("Using mock GPIO")
                self._gpio = MockGPIO()
            else:
                import RPi.GPIO as GPIO
                self._gpio = GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                
            self._initialized = True
            logger.info("GPIO initialized")
            return True
            
        except ImportError:
            logger.warning("RPi.GPIO not available, using mock")
            self._gpio = MockGPIO()
            self._initialized = True
            return True
        except Exception as e:
            logger.error(f"GPIO initialization failed: {e}")
            return False
            
    async def cleanup(self) -> None:
        """Cleanup GPIO"""
        if self._gpio and not self.use_mock:
            try:
                # Stop all PWM
                for pwm in self._pwm_channels.values():
                    pwm.stop()
                    
                self._gpio.cleanup()
            except Exception as e:
                logger.error(f"GPIO cleanup error: {e}")
                
        self._pins.clear()
        self._pwm_channels.clear()
        self._initialized = False
        logger.info("GPIO cleaned up")
        
    def setup_pin(self, config: PinConfig) -> bool:
        """Setup a GPIO pin"""
        if not self._initialized:
            logger.error("GPIO not initialized")
            return False
            
        try:
            pin = config.pin
            
            if config.mode == PinMode.OUTPUT:
                self._gpio.setup(pin, self._gpio.OUT)
                self._gpio.output(pin, config.initial_state.value)
                
            elif config.mode == PinMode.INPUT:
                self._gpio.setup(pin, self._gpio.IN)
                
            elif config.mode == PinMode.INPUT_PULLUP:
                self._gpio.setup(pin, self._gpio.IN, pull_up_down=self._gpio.PUD_UP)
                
            elif config.mode == PinMode.INPUT_PULLDOWN:
                self._gpio.setup(pin, self._gpio.IN, pull_up_down=self._gpio.PUD_DOWN)
                
            elif config.mode == PinMode.PWM:
                self._gpio.setup(pin, self._gpio.OUT)
                pwm = self._gpio.PWM(pin, config.pwm_frequency)
                pwm.start(0)
                self._pwm_channels[pin] = pwm
                
            self._pins[pin] = config
            logger.debug(f"Setup pin {pin} as {config.mode.value}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to setup pin {config.pin}: {e}")
            return False
            
    def write(self, pin: int, state: PinState) -> bool:
        """Write to output pin"""
        if pin not in self._pins:
            logger.error(f"Pin {pin} not configured")
            return False
            
        if self._pins[pin].mode != PinMode.OUTPUT:
            logger.error(f"Pin {pin} is not an output")
            return False
            
        try:
            self._gpio.output(pin, state.value)
            return True
        except Exception as e:
            logger.error(f"Failed to write pin {pin}: {e}")
            return False
            
    def read(self, pin: int) -> Optional[PinState]:
        """Read from input pin"""
        if pin not in self._pins:
            logger.error(f"Pin {pin} not configured")
            return None
            
        try:
            value = self._gpio.input(pin)
            return PinState.HIGH if value else PinState.LOW
        except Exception as e:
            logger.error(f"Failed to read pin {pin}: {e}")
            return None
            
    def set_pwm_duty(self, pin: int, duty_cycle: float) -> bool:
        """Set PWM duty cycle (0-100)"""
        if pin not in self._pwm_channels:
            logger.error(f"Pin {pin} is not a PWM pin")
            return False
            
        try:
            duty = max(0, min(100, duty_cycle))
            self._pwm_channels[pin].ChangeDutyCycle(duty)
            return True
        except Exception as e:
            logger.error(f"Failed to set PWM on pin {pin}: {e}")
            return False
            
    def set_pwm_frequency(self, pin: int, frequency: int) -> bool:
        """Set PWM frequency"""
        if pin not in self._pwm_channels:
            logger.error(f"Pin {pin} is not a PWM pin")
            return False
            
        try:
            self._pwm_channels[pin].ChangeFrequency(frequency)
            return True
        except Exception as e:
            logger.error(f"Failed to set PWM frequency on pin {pin}: {e}")
            return False
            
    def add_event_callback(
        self,
        pin: int,
        callback: Callable,
        edge: str = "both",
        bouncetime: int = 200
    ) -> bool:
        """Add event detection callback"""
        if pin not in self._pins:
            logger.error(f"Pin {pin} not configured")
            return False
            
        try:
            edge_map = {
                "rising": self._gpio.RISING,
                "falling": self._gpio.FALLING,
                "both": self._gpio.BOTH
            }
            
            gpio_edge = edge_map.get(edge.lower(), self._gpio.BOTH)
            
            if pin not in self._callbacks:
                self._gpio.add_event_detect(
                    pin,
                    gpio_edge,
                    bouncetime=bouncetime
                )
                self._callbacks[pin] = []
                
            self._gpio.add_event_callback(pin, callback)
            self._callbacks[pin].append(callback)
            
            logger.debug(f"Added callback for pin {pin}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to add callback for pin {pin}: {e}")
            return False
            
    def remove_event_callback(self, pin: int) -> bool:
        """Remove event detection"""
        if pin not in self._callbacks:
            return True
            
        try:
            self._gpio.remove_event_detect(pin)
            del self._callbacks[pin]
            return True
        except Exception as e:
            logger.error(f"Failed to remove callback for pin {pin}: {e}")
            return False


class MockGPIO:
    """Mock GPIO for testing without hardware"""
    
    BCM = "BCM"
    OUT = "OUT"
    IN = "IN"
    PUD_UP = "PUD_UP"
    PUD_DOWN = "PUD_DOWN"
    RISING = "RISING"
    FALLING = "FALLING"
    BOTH = "BOTH"
    
    def __init__(self):
        self._pins: Dict[int, Dict[str, Any]] = {}
        self._callbacks: Dict[int, List[Callable]] = {}
        
    def setmode(self, mode: str) -> None:
        pass
        
    def setwarnings(self, value: bool) -> None:
        pass
        
    def setup(self, pin: int, mode: str, pull_up_down: str = None) -> None:
        self._pins[pin] = {"mode": mode, "value": 0}
        
    def output(self, pin: int, value: int) -> None:
        if pin in self._pins:
            self._pins[pin]["value"] = value
            
    def input(self, pin: int) -> int:
        return self._pins.get(pin, {}).get("value", 0)
        
    def PWM(self, pin: int, frequency: int) -> "MockPWM":
        return MockPWM(pin, frequency)
        
    def add_event_detect(self, pin: int, edge: str, bouncetime: int = 200) -> None:
        self._callbacks[pin] = []
        
    def add_event_callback(self, pin: int, callback: Callable) -> None:
        if pin in self._callbacks:
            self._callbacks[pin].append(callback)
            
    def remove_event_detect(self, pin: int) -> None:
        if pin in self._callbacks:
            del self._callbacks[pin]
            
    def cleanup(self, pins: List[int] = None) -> None:
        if pins:
            for pin in pins:
                self._pins.pop(pin, None)
        else:
            self._pins.clear()


class MockPWM:
    """Mock PWM channel"""
    
    def __init__(self, pin: int, frequency: int):
        self.pin = pin
        self.frequency = frequency
        self.duty_cycle = 0
        
    def start(self, duty_cycle: float) -> None:
        self.duty_cycle = duty_cycle
        
    def stop(self) -> None:
        self.duty_cycle = 0
        
    def ChangeDutyCycle(self, duty_cycle: float) -> None:
        self.duty_cycle = duty_cycle
        
    def ChangeFrequency(self, frequency: int) -> None:
        self.frequency = frequency
