"""
RoboTwin Communication - ROS Bridge
Bridge for ROS2 integration using rosbridge protocol
"""

import asyncio
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Callable, List
from enum import Enum

from robotwin.utils.logger import get_logger
from robotwin.communication.websocket_client import WebSocketClient

logger = get_logger(__name__)


@dataclass
class ROSTopic:
    """ROS topic information"""
    name: str
    msg_type: str
    publisher: bool = False
    subscriber: bool = False
    callback: Optional[Callable] = None


@dataclass
class ROSService:
    """ROS service information"""
    name: str
    srv_type: str


class ROSBridge:
    """
    ROS Bridge for ROS2 integration
    
    Provides ROS-compatible interface using rosbridge WebSocket protocol
    Can work with:
    - rosbridge_suite (ROS2)
    - Custom server implementing rosbridge protocol
    """
    
    def __init__(
        self,
        ws_client: WebSocketClient,
        namespace: str = ""
    ):
        self.ws = ws_client
        self.namespace = namespace
        
        # Track topics
        self._publishers: Dict[str, ROSTopic] = {}
        self._subscribers: Dict[str, ROSTopic] = {}
        
        # Track services
        self._services: Dict[str, ROSService] = {}
        
    def _resolve_name(self, name: str) -> str:
        """Resolve topic/service name with namespace"""
        if name.startswith("/"):
            return name
        if self.namespace:
            return f"/{self.namespace}/{name}"
        return f"/{name}"
    
    # ========================================================================
    # Publishers
    # ========================================================================
    
    async def create_publisher(
        self,
        topic: str,
        msg_type: str
    ) -> "Publisher":
        """
        Create a publisher for a topic
        
        Args:
            topic: Topic name
            msg_type: Message type (e.g., "geometry_msgs/Twist")
            
        Returns:
            Publisher object
        """
        resolved_topic = self._resolve_name(topic)
        
        # Advertise topic
        await self.ws.advertise(resolved_topic, msg_type)
        
        # Track publisher
        self._publishers[resolved_topic] = ROSTopic(
            name=resolved_topic,
            msg_type=msg_type,
            publisher=True
        )
        
        return Publisher(self.ws, resolved_topic, msg_type)
    
    async def destroy_publisher(self, topic: str) -> None:
        """Destroy a publisher"""
        resolved_topic = self._resolve_name(topic)
        
        if resolved_topic in self._publishers:
            await self.ws.unadvertise(resolved_topic)
            del self._publishers[resolved_topic]
    
    # ========================================================================
    # Subscribers
    # ========================================================================
    
    async def create_subscription(
        self,
        topic: str,
        msg_type: str,
        callback: Callable,
        throttle_rate: int = 0,
        queue_size: int = 1
    ) -> "Subscription":
        """
        Create a subscription to a topic
        
        Args:
            topic: Topic name
            msg_type: Message type
            callback: Callback function for messages
            throttle_rate: Throttle rate in ms (0 = no throttle)
            queue_size: Message queue size
            
        Returns:
            Subscription object
        """
        resolved_topic = self._resolve_name(topic)
        
        # Subscribe
        await self.ws.subscribe(
            resolved_topic,
            msg_type,
            callback,
            throttle_rate,
            queue_size
        )
        
        # Track subscription
        self._subscribers[resolved_topic] = ROSTopic(
            name=resolved_topic,
            msg_type=msg_type,
            subscriber=True,
            callback=callback
        )
        
        return Subscription(self.ws, resolved_topic, msg_type)
    
    async def destroy_subscription(self, topic: str) -> None:
        """Destroy a subscription"""
        resolved_topic = self._resolve_name(topic)
        
        if resolved_topic in self._subscribers:
            await self.ws.unsubscribe(resolved_topic)
            del self._subscribers[resolved_topic]
    
    # ========================================================================
    # Services
    # ========================================================================
    
    async def call_service(
        self,
        service: str,
        request: Dict[str, Any],
        timeout: float = 10.0
    ) -> Dict[str, Any]:
        """
        Call a service
        
        Args:
            service: Service name
            request: Service request arguments
            timeout: Response timeout
            
        Returns:
            Service response
        """
        resolved_service = self._resolve_name(service)
        return await self.ws.call_service(resolved_service, request, timeout)
    
    # ========================================================================
    # Utility Methods
    # ========================================================================
    
    def get_topic_list(self) -> List[str]:
        """Get list of known topics"""
        topics = set(self._publishers.keys()) | set(self._subscribers.keys())
        return list(topics)
    
    def get_publishers(self) -> Dict[str, ROSTopic]:
        """Get all publishers"""
        return self._publishers.copy()
    
    def get_subscribers(self) -> Dict[str, ROSTopic]:
        """Get all subscribers"""
        return self._subscribers.copy()


class Publisher:
    """
    ROS-compatible publisher
    """
    
    def __init__(
        self,
        ws_client: WebSocketClient,
        topic: str,
        msg_type: str
    ):
        self._ws = ws_client
        self.topic = topic
        self.msg_type = msg_type
        
    async def publish(self, msg: Dict[str, Any]) -> None:
        """Publish a message"""
        await self._ws.publish(self.topic, msg)
        
    def publish_sync(self, msg: Dict[str, Any]) -> None:
        """Publish synchronously (schedules async publish)"""
        asyncio.create_task(self.publish(msg))


class Subscription:
    """
    ROS-compatible subscription
    """
    
    def __init__(
        self,
        ws_client: WebSocketClient,
        topic: str,
        msg_type: str
    ):
        self._ws = ws_client
        self.topic = topic
        self.msg_type = msg_type
        
    async def unsubscribe(self) -> None:
        """Unsubscribe from topic"""
        await self._ws.unsubscribe(self.topic)


# ============================================================================
# Standard Topic Helpers
# ============================================================================

class StandardTopics:
    """Standard ROS topic names"""
    CMD_VEL = "/cmd_vel"
    ODOM = "/odom"
    SCAN = "/scan"
    IMU = "/imu"
    CAMERA = "/camera/image_raw"
    CAMERA_COMPRESSED = "/camera/image/compressed"
    BATTERY = "/battery_state"
    TF = "/tf"
    TF_STATIC = "/tf_static"
    GOAL_POSE = "/goal_pose"
    PATH = "/path"
    MAP = "/map"
    DIAGNOSTICS = "/diagnostics"


async def create_cmd_vel_publisher(bridge: ROSBridge) -> Publisher:
    """Create cmd_vel publisher"""
    return await bridge.create_publisher(
        StandardTopics.CMD_VEL,
        "geometry_msgs/Twist"
    )


async def create_odom_publisher(bridge: ROSBridge) -> Publisher:
    """Create odometry publisher"""
    return await bridge.create_publisher(
        StandardTopics.ODOM,
        "nav_msgs/Odometry"
    )


async def create_scan_publisher(bridge: ROSBridge) -> Publisher:
    """Create laser scan publisher"""
    return await bridge.create_publisher(
        StandardTopics.SCAN,
        "sensor_msgs/LaserScan"
    )


async def create_cmd_vel_subscription(
    bridge: ROSBridge,
    callback: Callable
) -> Subscription:
    """Create cmd_vel subscription"""
    return await bridge.create_subscription(
        StandardTopics.CMD_VEL,
        "geometry_msgs/Twist",
        callback
    )
