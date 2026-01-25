"""
RoboTwin Robot Core - Main Robot Class
Handles robot initialization, state management, and main control loop
"""

import asyncio
import signal
import sys
from pathlib import Path
from typing import Optional, Dict, Any
from dataclasses import dataclass, field
from datetime import datetime
import yaml

from robotwin.utils.logger import get_logger
from robotwin.utils.config import RobotConfig, load_config
from robotwin.core.state import RobotState, RobotStatus
from robotwin.communication.websocket_client import WebSocketClient
from robotwin.hardware.motors import MotorController, create_motor_controller
from robotwin.hardware.sensors import SensorManager
from robotwin.control.controller import MainController

logger = get_logger(__name__)


@dataclass
class RobotContext:
    """Runtime context for robot"""
    config: RobotConfig
    state: RobotState
    websocket: Optional[WebSocketClient] = None
    motors: Optional[MotorController] = None
    sensors: Optional[SensorManager] = None
    controller: Optional[MainController] = None
    start_time: datetime = field(default_factory=datetime.now)
    

class Robot:
    """
    Main Robot class
    Coordinates all robot subsystems and communication
    """
    
    def __init__(self, config_path: str = "config/robot_config.yaml"):
        self.config_path = Path(config_path)
        self.config: Optional[RobotConfig] = None
        self.ctx: Optional[RobotContext] = None
        
        self._running = False
        self._shutdown_event = asyncio.Event()
        self._tasks: list[asyncio.Task] = []
        
    async def initialize(self) -> None:
        """Initialize all robot subsystems"""
        logger.info("Initializing robot...")
        
        # Load configuration
        self.config = load_config(self.config_path)
        logger.info(f"Loaded config for robot: {self.config.robot.name}")
        
        # Create robot state
        state = RobotState(
            robot_id=self.config.robot.id,
            name=self.config.robot.name,
            robot_type=self.config.robot.type,
            status=RobotStatus.INITIALIZING
        )
        
        # Create context
        self.ctx = RobotContext(
            config=self.config,
            state=state
        )
        
        # Initialize subsystems
        await self._init_communication()
        await self._init_hardware()
        await self._init_control()
        
        self.ctx.state.status = RobotStatus.READY
        logger.info("Robot initialization complete")
        
    async def _init_communication(self) -> None:
        """Initialize communication layer"""
        logger.info("Initializing communication...")
        
        self.ctx.websocket = WebSocketClient(
            url=self.config.server.url,
            robot_id=self.config.robot.id,
            reconnect_interval=self.config.server.reconnect_interval,
            heartbeat_interval=self.config.server.heartbeat_interval
        )
        
        # Setup message handlers
        self.ctx.websocket.on_message("teleop", self._handle_teleop)
        self.ctx.websocket.on_message("command", self._handle_command)
        self.ctx.websocket.on_message("subscribe", self._handle_subscribe)
        self.ctx.websocket.on_message("config", self._handle_config)
        
    async def _init_hardware(self) -> None:
        """Initialize hardware interfaces"""
        logger.info("Initializing hardware...")
        
        # Initialize motor controller
        self.ctx.motors = create_motor_controller(
            self.config.motors,
            self.config.robot.physical
        )
        await self.ctx.motors.initialize()
        
        # Initialize sensors
        self.ctx.sensors = SensorManager(self.config.sensors)
        await self.ctx.sensors.initialize()
        
    async def _init_control(self) -> None:
        """Initialize control systems"""
        logger.info("Initializing control systems...")
        
        self.ctx.controller = MainController(
            config=self.config,
            motors=self.ctx.motors,
            sensors=self.ctx.sensors
        )
        await self.ctx.controller.initialize()
        
    async def start(self) -> None:
        """Start the robot"""
        if self._running:
            logger.warning("Robot already running")
            return
            
        logger.info("Starting robot...")
        self._running = True
        self.ctx.state.status = RobotStatus.RUNNING
        
        # Setup signal handlers
        loop = asyncio.get_event_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, lambda: asyncio.create_task(self.shutdown()))
            
        # Start all tasks
        self._tasks = [
            asyncio.create_task(self.ctx.websocket.connect()),
            asyncio.create_task(self._control_loop()),
            asyncio.create_task(self._telemetry_loop()),
            asyncio.create_task(self._sensor_loop()),
            asyncio.create_task(self._diagnostics_loop()),
        ]
        
        logger.info("Robot started successfully")
        
        # Wait for shutdown
        await self._shutdown_event.wait()
        
    async def _control_loop(self) -> None:
        """Main control loop"""
        period = 1.0 / self.config.robot.control_frequency
        
        while self._running:
            try:
                start_time = asyncio.get_event_loop().time()
                
                # Run control update
                await self.ctx.controller.update()
                
                # Calculate sleep time
                elapsed = asyncio.get_event_loop().time() - start_time
                sleep_time = max(0, period - elapsed)
                await asyncio.sleep(sleep_time)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Control loop error: {e}")
                await asyncio.sleep(0.1)
                
    async def _telemetry_loop(self) -> None:
        """Telemetry publishing loop"""
        period = 1.0 / self.config.telemetry.publish_frequency
        
        while self._running:
            try:
                # Gather telemetry data
                telemetry = await self._gather_telemetry()
                
                # Send to server
                if self.ctx.websocket.is_connected:
                    await self.ctx.websocket.send_telemetry(telemetry)
                    
                await asyncio.sleep(period)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Telemetry loop error: {e}")
                await asyncio.sleep(1)
                
    async def _sensor_loop(self) -> None:
        """Sensor data acquisition loop"""
        while self._running:
            try:
                await self.ctx.sensors.update()
                await asyncio.sleep(0.01)  # 100Hz max
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Sensor loop error: {e}")
                await asyncio.sleep(0.1)
                
    async def _diagnostics_loop(self) -> None:
        """Diagnostics publishing loop"""
        period = 1.0 / self.config.diagnostics.publish_frequency
        
        while self._running:
            try:
                diagnostics = await self._gather_diagnostics()
                
                if self.ctx.websocket.is_connected:
                    await self.ctx.websocket.send_diagnostics(diagnostics)
                    
                await asyncio.sleep(period)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Diagnostics loop error: {e}")
                await asyncio.sleep(5)
                
    async def _gather_telemetry(self) -> Dict[str, Any]:
        """Gather all telemetry data"""
        telemetry = {
            "timestamp": datetime.now().isoformat(),
            "robotId": self.config.robot.id,
        }
        
        # Odometry
        if self.config.telemetry.publish.odometry:
            odom = self.ctx.controller.get_odometry()
            telemetry["odometry"] = odom
            telemetry["pose"] = {
                "position": {"x": odom.x, "y": odom.y, "z": 0},
                "orientation": {
                    "x": 0, "y": 0,
                    "z": odom.theta_sin,
                    "w": odom.theta_cos
                }
            }
            
        # Velocity
        if self.config.telemetry.publish.velocity:
            vel = self.ctx.controller.get_velocity()
            telemetry["velocity"] = {
                "linear": {"x": vel.linear, "y": 0, "z": 0},
                "angular": {"x": 0, "y": 0, "z": vel.angular}
            }
            
        # Battery
        if self.config.telemetry.publish.battery:
            battery = await self.ctx.sensors.get_battery()
            if battery:
                telemetry["battery"] = battery
                
        # IMU
        if self.config.telemetry.publish.imu:
            imu = await self.ctx.sensors.get_imu()
            if imu:
                telemetry["imu"] = imu
                
        return telemetry
        
    async def _gather_diagnostics(self) -> Dict[str, Any]:
        """Gather diagnostics data"""
        import psutil
        
        return {
            "timestamp": datetime.now().isoformat(),
            "robotId": self.config.robot.id,
            "status": self.ctx.state.status.value,
            "uptime": (datetime.now() - self.ctx.start_time).total_seconds(),
            "system": {
                "cpu_percent": psutil.cpu_percent(),
                "memory_percent": psutil.virtual_memory().percent,
                "temperature": self._get_cpu_temp(),
            },
            "subsystems": {
                "motors": self.ctx.motors.get_status() if self.ctx.motors else None,
                "sensors": self.ctx.sensors.get_status() if self.ctx.sensors else None,
                "communication": self.ctx.websocket.get_status() if self.ctx.websocket else None,
            }
        }
        
    def _get_cpu_temp(self) -> Optional[float]:
        """Get CPU temperature"""
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                return float(f.read()) / 1000.0
        except:
            return None
            
    # Message handlers
    async def _handle_teleop(self, message: Dict[str, Any]) -> None:
        """Handle teleop command from server"""
        twist = message.get("payload", {})
        linear = twist.get("linear", {}).get("x", 0)
        angular = twist.get("angular", {}).get("z", 0)
        
        await self.ctx.controller.set_velocity(linear, angular)
        
    async def _handle_command(self, message: Dict[str, Any]) -> None:
        """Handle command from server"""
        command = message.get("payload", {})
        cmd_type = command.get("type")
        
        logger.info(f"Received command: {cmd_type}")
        
        if cmd_type == "STOP":
            await self.emergency_stop()
        elif cmd_type == "NAVIGATE":
            goal = command.get("goal")
            if goal:
                await self.ctx.controller.navigate_to(goal)
        elif cmd_type == "PAUSE":
            await self.pause()
        elif cmd_type == "RESUME":
            await self.resume()
            
    async def _handle_subscribe(self, message: Dict[str, Any]) -> None:
        """Handle subscription request"""
        topics = message.get("payload", {}).get("topics", [])
        logger.info(f"Subscribe request for topics: {topics}")
        
        # Enable/disable sensor streaming based on topics
        for topic in topics:
            if topic == "scan":
                self.ctx.sensors.enable_lidar_stream(True)
            elif topic == "image":
                self.ctx.sensors.enable_camera_stream(True)
                
    async def _handle_config(self, message: Dict[str, Any]) -> None:
        """Handle config update"""
        new_config = message.get("payload", {})
        logger.info(f"Config update received")
        # Apply runtime config updates
        
    # Public control methods
    async def move(self, linear: float = 0.0, angular: float = 0.0) -> None:
        """Set robot velocity"""
        await self.ctx.controller.set_velocity(linear, angular)
        
    async def stop(self) -> None:
        """Stop robot movement"""
        await self.ctx.controller.set_velocity(0, 0)
        
    async def emergency_stop(self) -> None:
        """Emergency stop"""
        logger.warning("EMERGENCY STOP activated")
        self.ctx.state.status = RobotStatus.EMERGENCY_STOP
        await self.ctx.motors.emergency_stop()
        await self.ctx.controller.reset()
        
    async def pause(self) -> None:
        """Pause robot"""
        self.ctx.state.status = RobotStatus.PAUSED
        await self.stop()
        
    async def resume(self) -> None:
        """Resume robot"""
        if self.ctx.state.status == RobotStatus.PAUSED:
            self.ctx.state.status = RobotStatus.RUNNING
            
    async def shutdown(self) -> None:
        """Shutdown robot gracefully"""
        if not self._running:
            return
            
        logger.info("Shutting down robot...")
        self._running = False
        self.ctx.state.status = RobotStatus.SHUTDOWN
        
        # Stop all movement
        await self.stop()
        
        # Cancel all tasks
        for task in self._tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
                
        # Cleanup subsystems
        if self.ctx.websocket:
            await self.ctx.websocket.disconnect()
        if self.ctx.motors:
            await self.ctx.motors.shutdown()
        if self.ctx.sensors:
            await self.ctx.sensors.shutdown()
            
        self._shutdown_event.set()
        logger.info("Robot shutdown complete")


async def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description="RoboTwin Robot Core")
    parser.add_argument(
        "--config", "-c",
        default="config/robot_config.yaml",
        help="Path to config file"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug mode"
    )
    args = parser.parse_args()
    
    # Configure logging
    if args.debug:
        import logging
        logging.getLogger().setLevel(logging.DEBUG)
        
    # Create and run robot
    robot = Robot(args.config)
    
    try:
        await robot.initialize()
        await robot.start()
    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt")
    except Exception as e:
        logger.error(f"Robot error: {e}")
        raise
    finally:
        await robot.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
