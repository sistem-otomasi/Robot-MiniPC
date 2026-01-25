"""
RoboTwin Utils - Logging Configuration
Structured logging for robot systems
"""

import logging
import sys
from pathlib import Path
from typing import Optional
from datetime import datetime


class ColoredFormatter(logging.Formatter):
    """Colored log formatter for terminal output"""
    
    COLORS = {
        logging.DEBUG: "\033[36m",     # Cyan
        logging.INFO: "\033[32m",      # Green
        logging.WARNING: "\033[33m",   # Yellow
        logging.ERROR: "\033[31m",     # Red
        logging.CRITICAL: "\033[35m",  # Magenta
    }
    RESET = "\033[0m"
    
    def format(self, record: logging.LogRecord) -> str:
        color = self.COLORS.get(record.levelno, self.RESET)
        record.levelname = f"{color}{record.levelname}{self.RESET}"
        return super().format(record)


def setup_logging(
    level: int = logging.INFO,
    log_file: Optional[str] = None,
    log_dir: str = "logs",
    enable_colors: bool = True
) -> None:
    """
    Setup logging configuration
    
    Args:
        level: Logging level
        log_file: Log file name (auto-generated if None)
        log_dir: Directory for log files
        enable_colors: Enable colored console output
    """
    # Create log directory
    if log_file:
        log_path = Path(log_dir)
        log_path.mkdir(parents=True, exist_ok=True)
        
        if not log_file.endswith('.log'):
            log_file = f"{log_file}.log"
        full_log_path = log_path / log_file
    else:
        # Auto-generate log file name
        log_path = Path(log_dir)
        log_path.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        full_log_path = log_path / f"robotwin_{timestamp}.log"
    
    # Root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)
    
    # Clear existing handlers
    root_logger.handlers.clear()
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    
    if enable_colors and sys.stdout.isatty():
        console_format = ColoredFormatter(
            "%(asctime)s | %(levelname)s | %(name)s | %(message)s",
            datefmt="%H:%M:%S"
        )
    else:
        console_format = logging.Formatter(
            "%(asctime)s | %(levelname)s | %(name)s | %(message)s",
            datefmt="%H:%M:%S"
        )
    
    console_handler.setFormatter(console_format)
    root_logger.addHandler(console_handler)
    
    # File handler
    if log_file:
        file_handler = logging.FileHandler(full_log_path)
        file_handler.setLevel(logging.DEBUG)
        file_format = logging.Formatter(
            "%(asctime)s | %(levelname)s | %(name)s | %(funcName)s:%(lineno)d | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )
        file_handler.setFormatter(file_format)
        root_logger.addHandler(file_handler)
    
    # Reduce noise from third-party libraries
    logging.getLogger("websockets").setLevel(logging.WARNING)
    logging.getLogger("asyncio").setLevel(logging.WARNING)
    logging.getLogger("urllib3").setLevel(logging.WARNING)


def get_logger(name: str) -> logging.Logger:
    """
    Get logger for a module
    
    Args:
        name: Logger name (usually __name__)
        
    Returns:
        Logger instance
    """
    return logging.getLogger(name)


class RobotLogger:
    """
    Robot-specific logger with structured data logging
    """
    
    def __init__(self, name: str):
        self.logger = get_logger(name)
        self._context: dict = {}
        
    def set_context(self, **kwargs) -> None:
        """Set persistent context for all log messages"""
        self._context.update(kwargs)
        
    def clear_context(self) -> None:
        """Clear logging context"""
        self._context.clear()
        
    def _format_message(self, msg: str, **kwargs) -> str:
        """Format message with context"""
        context = {**self._context, **kwargs}
        if context:
            ctx_str = " | ".join(f"{k}={v}" for k, v in context.items())
            return f"{msg} | {ctx_str}"
        return msg
    
    def debug(self, msg: str, **kwargs) -> None:
        self.logger.debug(self._format_message(msg, **kwargs))
        
    def info(self, msg: str, **kwargs) -> None:
        self.logger.info(self._format_message(msg, **kwargs))
        
    def warning(self, msg: str, **kwargs) -> None:
        self.logger.warning(self._format_message(msg, **kwargs))
        
    def error(self, msg: str, **kwargs) -> None:
        self.logger.error(self._format_message(msg, **kwargs))
        
    def critical(self, msg: str, **kwargs) -> None:
        self.logger.critical(self._format_message(msg, **kwargs))
    
    # Alias methods
    warn = warning
    
    def telemetry(self, **data) -> None:
        """Log telemetry data"""
        self.debug("Telemetry", **data)
        
    def motor(self, motor_id: str, **data) -> None:
        """Log motor data"""
        self.debug(f"Motor[{motor_id}]", **data)
        
    def sensor(self, sensor_type: str, **data) -> None:
        """Log sensor data"""
        self.debug(f"Sensor[{sensor_type}]", **data)
        
    def control(self, controller: str, **data) -> None:
        """Log control loop data"""
        self.debug(f"Control[{controller}]", **data)
        
    def navigation(self, **data) -> None:
        """Log navigation data"""
        self.debug("Navigation", **data)
