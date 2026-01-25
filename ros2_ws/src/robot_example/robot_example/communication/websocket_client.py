"""
RoboTwin Communication - WebSocket Client
Connects robot to the central server (Web-Server)
"""

import asyncio
import json
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Callable, List
from enum import Enum
import ssl

import websockets
from websockets.client import WebSocketClientProtocol

from robot_example.utils.logger import get_logger
from robot_example.communication.protocol import (
    MessageType,
    Message,
    create_message,
    parse_message
)

logger = get_logger(__name__)


class ConnectionState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    AUTHENTICATED = "authenticated"
    RECONNECTING = "reconnecting"
    ERROR = "error"


@dataclass
class ClientConfig:
    """WebSocket client configuration"""
    url: str = "ws://localhost:3000/ws/robot"
    robot_id: str = "robot-001"
    robot_secret: str = ""
    reconnect_interval: float = 5.0
    max_reconnect_attempts: int = 10
    heartbeat_interval: float = 10.0
    connection_timeout: float = 30.0
    ssl_verify: bool = True


class WebSocketClient:
    """
    WebSocket client for robot-server communication
    
    Implements rosbridge-compatible protocol for interoperability
    with ROS2 web bridge
    """
    
    def __init__(
        self,
        url: str,
        robot_id: str,
        robot_secret: str = "",
        reconnect_interval: float = 5.0,
        heartbeat_interval: float = 10.0,
        max_reconnect_attempts: int = 10
    ):
        self.config = ClientConfig(
            url=url,
            robot_id=robot_id,
            robot_secret=robot_secret,
            reconnect_interval=reconnect_interval,
            heartbeat_interval=heartbeat_interval,
            max_reconnect_attempts=max_reconnect_attempts
        )
        
        self._ws: Optional[WebSocketClientProtocol] = None
        self._state = ConnectionState.DISCONNECTED
        self._reconnect_attempts = 0
        
        # Message handlers
        self._handlers: Dict[str, List[Callable]] = {}
        
        # Pending requests (for request-response pattern)
        self._pending_requests: Dict[str, asyncio.Future] = {}
        self._request_counter = 0
        
        # Background tasks
        self._receive_task: Optional[asyncio.Task] = None
        self._heartbeat_task: Optional[asyncio.Task] = None
        self._reconnect_task: Optional[asyncio.Task] = None
        
        # Connection event
        self._connected_event = asyncio.Event()
        
        # Subscribed topics
        self._subscriptions: Dict[str, Dict[str, Any]] = {}
        
        # Advertised topics
        self._advertisements: Dict[str, str] = {}
        
    @property
    def state(self) -> ConnectionState:
        """Get current connection state"""
        return self._state
    
    @property
    def is_connected(self) -> bool:
        """Check if connected and authenticated"""
        return self._state == ConnectionState.AUTHENTICATED
    
    async def connect(self) -> bool:
        """
        Connect to the server
        
        Returns:
            True if connected successfully
        """
        if self._state in [ConnectionState.CONNECTED, ConnectionState.AUTHENTICATED]:
            return True
            
        self._state = ConnectionState.CONNECTING
        
        try:
            # Build SSL context if using wss://
            ssl_context = None
            if self.config.url.startswith("wss://"):
                ssl_context = ssl.create_default_context()
                if not self.config.ssl_verify:
                    ssl_context.check_hostname = False
                    ssl_context.verify_mode = ssl.CERT_NONE
                    
            self._ws = await asyncio.wait_for(
                websockets.connect(
                    self.config.url,
                    ssl=ssl_context,
                    ping_interval=20,
                    ping_timeout=10,
                    close_timeout=5
                ),
                timeout=self.config.connection_timeout
            )
            
            self._state = ConnectionState.CONNECTED
            self._reconnect_attempts = 0
            
            # Start receive loop
            self._receive_task = asyncio.create_task(self._receive_loop())
            
            # Authenticate
            await self._authenticate()
            
            # Start heartbeat
            self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())
            
            logger.info(f"Connected to server: {self.config.url}")
            self._connected_event.set()
            
            return True
            
        except asyncio.TimeoutError:
            logger.error("Connection timeout")
            self._state = ConnectionState.ERROR
            return False
            
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            self._state = ConnectionState.ERROR
            return False
    
    async def disconnect(self) -> None:
        """Disconnect from server"""
        # Cancel tasks
        for task in [self._receive_task, self._heartbeat_task, self._reconnect_task]:
            if task:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
                    
        # Close WebSocket
        if self._ws:
            await self._ws.close()
            self._ws = None
            
        self._state = ConnectionState.DISCONNECTED
        self._connected_event.clear()
        
        logger.info("Disconnected from server")
        
    async def _authenticate(self) -> None:
        """Send authentication message"""
        auth_msg = create_message(
            MessageType.AUTH,
            robot_id=self.config.robot_id,
            payload={"secret": self.config.robot_secret}
        )
        
        await self.send_raw(auth_msg)
        
    async def _receive_loop(self) -> None:
        """Receive messages from server"""
        try:
            async for message in self._ws:
                try:
                    data = json.loads(message)
                    await self._handle_message(data)
                except json.JSONDecodeError:
                    logger.warning("Received invalid JSON")
                except Exception as e:
                    logger.error(f"Error handling message: {e}")
                    
        except websockets.exceptions.ConnectionClosed as e:
            logger.warning(f"Connection closed: {e}")
            self._state = ConnectionState.DISCONNECTED
            self._connected_event.clear()
            await self._schedule_reconnect()
            
        except Exception as e:
            logger.error(f"Receive loop error: {e}")
            self._state = ConnectionState.ERROR
            await self._schedule_reconnect()
            
    async def _heartbeat_loop(self) -> None:
        """Send periodic heartbeat"""
        while True:
            try:
                await asyncio.sleep(self.config.heartbeat_interval)
                
                if self._state == ConnectionState.AUTHENTICATED:
                    await self.send_raw({
                        "type": "ping",
                        "timestamp": int(time.time() * 1000)
                    })
                    
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Heartbeat error: {e}")
                
    async def _schedule_reconnect(self) -> None:
        """Schedule reconnection attempt"""
        if self._reconnect_attempts >= self.config.max_reconnect_attempts:
            logger.error("Max reconnect attempts reached")
            return
            
        self._reconnect_attempts += 1
        self._state = ConnectionState.RECONNECTING
        
        logger.info(
            f"Reconnecting in {self.config.reconnect_interval}s "
            f"(attempt {self._reconnect_attempts})"
        )
        
        await asyncio.sleep(self.config.reconnect_interval)
        await self.connect()
        
    async def _handle_message(self, data: Dict[str, Any]) -> None:
        """Handle incoming message"""
        msg_type = data.get("type", data.get("op", ""))
        
        # Handle authentication response
        if msg_type == "auth":
            payload = data.get("payload", {})
            if payload.get("status") == "authenticated":
                self._state = ConnectionState.AUTHENTICATED
                logger.info("Authenticated with server")
                
                # Re-subscribe to topics
                for topic, info in self._subscriptions.items():
                    await self._send_subscribe(topic, info.get("type"))
                    
                # Re-advertise topics
                for topic, msg_type in self._advertisements.items():
                    await self._send_advertise(topic, msg_type)
            else:
                logger.error(f"Authentication failed: {payload}")
                self._state = ConnectionState.ERROR
            return
            
        # Handle pong
        if msg_type == "pong":
            return
            
        # Handle ROS bridge messages
        if msg_type == "publish":
            topic = data.get("topic", "")
            msg = data.get("msg", {})
            await self._dispatch_topic_message(topic, msg)
            return
            
        # Handle service response
        if msg_type == "service_response":
            request_id = data.get("id")
            if request_id and request_id in self._pending_requests:
                self._pending_requests[request_id].set_result(data)
            return
            
        # Dispatch to registered handlers
        handlers = self._handlers.get(msg_type, [])
        for handler in handlers:
            try:
                if asyncio.iscoroutinefunction(handler):
                    await handler(data.get("payload", data))
                else:
                    handler(data.get("payload", data))
            except Exception as e:
                logger.error(f"Handler error for {msg_type}: {e}")
                
    async def _dispatch_topic_message(
        self,
        topic: str,
        message: Dict[str, Any]
    ) -> None:
        """Dispatch message to topic subscribers"""
        if topic in self._subscriptions:
            callback = self._subscriptions[topic].get("callback")
            if callback:
                try:
                    if asyncio.iscoroutinefunction(callback):
                        await callback(message)
                    else:
                        callback(message)
                except Exception as e:
                    logger.error(f"Topic callback error for {topic}: {e}")
                    
    def on_message(self, msg_type: str, handler: Callable) -> None:
        """
        Register message handler
        
        Args:
            msg_type: Message type to handle
            handler: Callback function
        """
        if msg_type not in self._handlers:
            self._handlers[msg_type] = []
        self._handlers[msg_type].append(handler)
        
    def remove_handler(self, msg_type: str, handler: Callable) -> None:
        """Remove message handler"""
        if msg_type in self._handlers:
            if handler in self._handlers[msg_type]:
                self._handlers[msg_type].remove(handler)
                
    async def send_raw(self, data: Dict[str, Any]) -> None:
        """Send raw message"""
        if self._ws:
            await self._ws.send(json.dumps(data))
            
    async def send(
        self,
        msg_type: str,
        payload: Dict[str, Any],
        **kwargs
    ) -> None:
        """
        Send message to server
        
        Args:
            msg_type: Message type
            payload: Message payload
        """
        message = {
            "type": msg_type,
            "payload": {
                "robotId": self.config.robot_id,
                **payload,
                "timestamp": int(time.time() * 1000)
            },
            "timestamp": int(time.time() * 1000),
            **kwargs
        }
        
        await self.send_raw(message)
        
    # ========================================================================
    # ROS Bridge Compatible Methods
    # ========================================================================
    
    async def subscribe(
        self,
        topic: str,
        msg_type: str,
        callback: Callable,
        throttle_rate: int = 0,
        queue_size: int = 1
    ) -> None:
        """
        Subscribe to a topic (rosbridge protocol)
        
        Args:
            topic: Topic name
            msg_type: Message type
            callback: Callback for received messages
            throttle_rate: Throttle rate in ms (0 = no throttle)
            queue_size: Queue size
        """
        self._subscriptions[topic] = {
            "type": msg_type,
            "callback": callback,
            "throttle_rate": throttle_rate,
            "queue_size": queue_size
        }
        
        if self.is_connected:
            await self._send_subscribe(topic, msg_type, throttle_rate, queue_size)
            
    async def _send_subscribe(
        self,
        topic: str,
        msg_type: str,
        throttle_rate: int = 0,
        queue_size: int = 1
    ) -> None:
        """Send subscribe message"""
        await self.send_raw({
            "op": "subscribe",
            "topic": topic,
            "type": msg_type,
            "throttle_rate": throttle_rate,
            "queue_length": queue_size
        })
        logger.debug(f"Subscribed to topic: {topic}")
        
    async def unsubscribe(self, topic: str) -> None:
        """Unsubscribe from a topic"""
        if topic in self._subscriptions:
            del self._subscriptions[topic]
            
        if self.is_connected:
            await self.send_raw({
                "op": "unsubscribe",
                "topic": topic
            })
            logger.debug(f"Unsubscribed from topic: {topic}")
            
    async def advertise(self, topic: str, msg_type: str) -> None:
        """
        Advertise a topic for publishing (rosbridge protocol)
        
        Args:
            topic: Topic name
            msg_type: Message type
        """
        self._advertisements[topic] = msg_type
        
        if self.is_connected:
            await self._send_advertise(topic, msg_type)
            
    async def _send_advertise(self, topic: str, msg_type: str) -> None:
        """Send advertise message"""
        await self.send_raw({
            "op": "advertise",
            "topic": topic,
            "type": msg_type
        })
        logger.debug(f"Advertised topic: {topic}")
        
    async def unadvertise(self, topic: str) -> None:
        """Unadvertise a topic"""
        if topic in self._advertisements:
            del self._advertisements[topic]
            
        if self.is_connected:
            await self.send_raw({
                "op": "unadvertise",
                "topic": topic
            })
            
    async def publish(self, topic: str, message: Dict[str, Any]) -> None:
        """
        Publish message to a topic (rosbridge protocol)
        
        Args:
            topic: Topic name
            message: Message data
        """
        if not self.is_connected:
            logger.warning(f"Cannot publish to {topic}: not connected")
            return
            
        await self.send_raw({
            "op": "publish",
            "topic": topic,
            "msg": message
        })
        
    async def call_service(
        self,
        service: str,
        args: Dict[str, Any] = None,
        timeout: float = 10.0
    ) -> Dict[str, Any]:
        """
        Call a ROS service (rosbridge protocol)
        
        Args:
            service: Service name
            args: Service arguments
            timeout: Response timeout
            
        Returns:
            Service response
        """
        self._request_counter += 1
        request_id = f"call_service:{service}:{self._request_counter}"
        
        # Create future for response
        future = asyncio.get_event_loop().create_future()
        self._pending_requests[request_id] = future
        
        try:
            # Send request
            await self.send_raw({
                "op": "call_service",
                "id": request_id,
                "service": service,
                "args": args or {}
            })
            
            # Wait for response
            response = await asyncio.wait_for(future, timeout=timeout)
            return response.get("values", {})
            
        except asyncio.TimeoutError:
            logger.error(f"Service call timeout: {service}")
            raise
            
        finally:
            self._pending_requests.pop(request_id, None)
            
    # ========================================================================
    # Convenience Methods
    # ========================================================================
    
    async def publish_telemetry(self, data: Dict[str, Any]) -> None:
        """Publish robot telemetry"""
        await self.send("telemetry", data)
        
    async def publish_odometry(
        self,
        x: float,
        y: float,
        theta: float,
        linear_vel: float,
        angular_vel: float
    ) -> None:
        """Publish odometry data"""
        await self.publish("/odom", {
            "pose": {
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": math.sin(theta / 2),
                        "w": math.cos(theta / 2)
                    }
                }
            },
            "twist": {
                "twist": {
                    "linear": {"x": linear_vel, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": angular_vel}
                }
            }
        })
        
    async def publish_laser_scan(
        self,
        ranges: List[float],
        angle_min: float,
        angle_max: float,
        angle_increment: float,
        range_min: float = 0.1,
        range_max: float = 12.0
    ) -> None:
        """Publish laser scan data"""
        await self.publish("/scan", {
            "ranges": ranges,
            "angle_min": angle_min,
            "angle_max": angle_max,
            "angle_increment": angle_increment,
            "range_min": range_min,
            "range_max": range_max,
            "time_increment": 0.0,
            "scan_time": 0.1
        })
        
    async def publish_battery(
        self,
        voltage: float,
        percentage: float,
        charging: bool = False
    ) -> None:
        """Publish battery state"""
        await self.send("battery", {
            "voltage": voltage,
            "percentage": percentage,
            "charging": charging
        })
        
    async def wait_connected(self, timeout: float = 30.0) -> bool:
        """Wait for connection"""
        try:
            await asyncio.wait_for(
                self._connected_event.wait(),
                timeout=timeout
            )
            return True
        except asyncio.TimeoutError:
            return False


# Import math for quaternion conversion
import math
