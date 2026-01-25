#!/usr/bin/env python3
"""
RoboTwin CLI - Command Line Interface
Main entry point for robot control
"""

import argparse
import asyncio
import signal
import sys
from pathlib import Path

from robot_example.utils.logger import setup_logging, get_logger
from robot_example.utils.config import load_config, create_default_config


logger = get_logger(__name__)


def create_parser() -> argparse.ArgumentParser:
    """Create command line argument parser"""
    parser = argparse.ArgumentParser(
        prog="robotwin",
        description="RoboTwin Robot Controller"
    )
    
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output"
    )
    
    parser.add_argument(
        "-c", "--config",
        type=str,
        default=None,
        help="Path to configuration file"
    )
    
    subparsers = parser.add_subparsers(dest="command", help="Commands")
    
    # Run command
    run_parser = subparsers.add_parser("run", help="Start robot controller")
    run_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run without connecting to hardware"
    )
    run_parser.add_argument(
        "--no-server",
        action="store_true",
        help="Run without connecting to server"
    )
    
    # Config commands
    config_parser = subparsers.add_parser("config", help="Configuration management")
    config_subparsers = config_parser.add_subparsers(dest="config_command")
    
    config_show = config_subparsers.add_parser("show", help="Show current config")
    config_create = config_subparsers.add_parser("create", help="Create default config")
    config_create.add_argument(
        "-o", "--output",
        type=str,
        default="config/robot_config.yaml",
        help="Output path"
    )
    
    # Test commands
    test_parser = subparsers.add_parser("test", help="Test hardware")
    test_parser.add_argument(
        "component",
        choices=["motors", "lidar", "camera", "imu", "all"],
        help="Component to test"
    )
    
    # Status command
    status_parser = subparsers.add_parser("status", help="Show robot status")
    
    return parser


async def run_robot(args: argparse.Namespace) -> None:
    """Run the robot controller"""
    from robot_example.core.robot import Robot
    
    # Load configuration
    config = load_config(args.config)
    
    # Override from args
    if args.dry_run:
        config.set("motors.type", "mock")
        config.set("lidar.enabled", False)
        config.set("camera.enabled", False)
        config.set("imu.enabled", False)
        
    # Create robot
    robot = Robot(config)
    
    # Setup signal handlers
    loop = asyncio.get_event_loop()
    
    def shutdown_handler():
        logger.info("Shutdown signal received")
        asyncio.create_task(robot.stop())
        
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, shutdown_handler)
    
    try:
        # Initialize robot
        await robot.initialize()
        
        # Connect to server (unless disabled)
        if not args.no_server:
            await robot.connect()
            
        # Run main loop
        await robot.run()
        
    except Exception as e:
        logger.error(f"Robot error: {e}")
        raise
    finally:
        await robot.stop()


async def test_component(args: argparse.Namespace) -> None:
    """Test hardware components"""
    from robot_example.utils.config import load_config
    
    config = load_config(args.config)
    
    if args.component in ["motors", "all"]:
        await test_motors(config)
        
    if args.component in ["lidar", "all"]:
        await test_lidar(config)
        
    if args.component in ["camera", "all"]:
        await test_camera(config)
        
    if args.component in ["imu", "all"]:
        await test_imu(config)


async def test_motors(config) -> None:
    """Test motor controllers"""
    from robot_example.hardware.motors import create_motor_controller
    
    logger.info("Testing motors...")
    
    motor_config = config.robot.motors
    controller = create_motor_controller(
        motor_type=motor_config.type,
        port=motor_config.port,
        baudrate=motor_config.baudrate,
        num_motors=motor_config.num_motors
    )
    
    try:
        await controller.connect()
        logger.info("Motor controller connected")
        
        # Test each motor
        for i in range(motor_config.num_motors):
            logger.info(f"Testing motor {i}...")
            
            # Forward
            await controller.set_velocity(i, 0.5)
            await asyncio.sleep(1)
            
            # Stop
            await controller.set_velocity(i, 0)
            await asyncio.sleep(0.5)
            
            # Reverse
            await controller.set_velocity(i, -0.5)
            await asyncio.sleep(1)
            
            # Stop
            await controller.set_velocity(i, 0)
            
        logger.info("Motor test complete")
        
    finally:
        await controller.disconnect()


async def test_lidar(config) -> None:
    """Test LiDAR sensor"""
    from robot_example.hardware.sensors import LidarSensor
    
    logger.info("Testing LiDAR...")
    
    lidar_config = config.robot.lidar
    if not lidar_config.enabled:
        logger.warning("LiDAR is disabled in config")
        return
        
    lidar = LidarSensor(
        port=lidar_config.port,
        baudrate=lidar_config.baudrate,
        lidar_type=lidar_config.type
    )
    
    scan_count = 0
    
    def on_scan(ranges, angles, intensities):
        nonlocal scan_count
        scan_count += 1
        if scan_count <= 5:
            logger.info(f"Scan {scan_count}: {len(ranges)} points, "
                       f"min={min(ranges):.2f}m, max={max(ranges):.2f}m")
    
    try:
        await lidar.start()
        lidar.on_scan(on_scan)
        
        # Collect 5 scans
        await asyncio.sleep(5)
        
        logger.info(f"LiDAR test complete: {scan_count} scans received")
        
    finally:
        await lidar.stop()


async def test_camera(config) -> None:
    """Test camera"""
    from robot_example.hardware.sensors import CameraSensor
    
    logger.info("Testing camera...")
    
    camera_config = config.robot.camera
    if not camera_config.enabled:
        logger.warning("Camera is disabled in config")
        return
        
    camera = CameraSensor(
        device=camera_config.device,
        width=camera_config.width,
        height=camera_config.height,
        fps=camera_config.fps
    )
    
    frame_count = 0
    
    def on_frame(frame):
        nonlocal frame_count
        frame_count += 1
        if frame_count <= 5:
            logger.info(f"Frame {frame_count}: {frame.shape}")
    
    try:
        await camera.start()
        camera.on_frame(on_frame)
        
        # Capture 5 frames
        await asyncio.sleep(2)
        
        logger.info(f"Camera test complete: {frame_count} frames received")
        
    finally:
        await camera.stop()


async def test_imu(config) -> None:
    """Test IMU sensor"""
    from robot_example.hardware.sensors import IMUSensor
    
    logger.info("Testing IMU...")
    
    imu_config = config.robot.imu
    if not imu_config.enabled:
        logger.warning("IMU is disabled in config")
        return
        
    imu = IMUSensor(
        bus=imu_config.bus,
        address=imu_config.address,
        imu_type=imu_config.type
    )
    
    sample_count = 0
    
    def on_data(accel, gyro, orientation):
        nonlocal sample_count
        sample_count += 1
        if sample_count <= 5:
            logger.info(f"IMU {sample_count}: accel={accel}, gyro={gyro}")
    
    try:
        await imu.start()
        imu.on_data(on_data)
        
        # Collect samples
        await asyncio.sleep(2)
        
        logger.info(f"IMU test complete: {sample_count} samples received")
        
    finally:
        await imu.stop()


def show_config(args: argparse.Namespace) -> None:
    """Show current configuration"""
    import yaml
    
    config = load_config(args.config)
    print(yaml.dump(config.to_dict(), default_flow_style=False))


def create_config(args: argparse.Namespace) -> None:
    """Create default configuration file"""
    create_default_config(args.output)
    print(f"Configuration created at: {args.output}")


def show_status(args: argparse.Namespace) -> None:
    """Show robot status"""
    import subprocess
    
    # Check if service is running
    result = subprocess.run(
        ["systemctl", "is-active", "robotwin"],
        capture_output=True,
        text=True
    )
    
    service_status = result.stdout.strip()
    print(f"Service Status: {service_status}")
    
    # Show system info
    try:
        import psutil
        print(f"CPU Usage: {psutil.cpu_percent()}%")
        print(f"Memory Usage: {psutil.virtual_memory().percent}%")
        print(f"Disk Usage: {psutil.disk_usage('/').percent}%")
    except ImportError:
        pass


def main() -> int:
    """Main entry point"""
    parser = create_parser()
    args = parser.parse_args()
    
    # Setup logging
    import logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    setup_logging(level=log_level, log_file="robotwin")
    
    # Execute command
    if args.command == "run":
        asyncio.run(run_robot(args))
        
    elif args.command == "test":
        asyncio.run(test_component(args))
        
    elif args.command == "config":
        if args.config_command == "show":
            show_config(args)
        elif args.config_command == "create":
            create_config(args)
        else:
            parser.print_help()
            
    elif args.command == "status":
        show_status(args)
        
    else:
        parser.print_help()
        
    return 0


if __name__ == "__main__":
    sys.exit(main())
