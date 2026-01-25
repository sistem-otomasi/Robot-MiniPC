# ROS 2 Robot Package - Quick Start

## Setup (Satu Kali)

```bash
cd /root/Otomasi/Sistem-Otomasi-Robot/Robot/ros2_ws
source install/setup.bash
```

## Available Commands

### 1. Hardware Monitor
```bash
hardware_monitor
```
Publishes:
- `/hardware/battery` - Battery level
- `/hardware/motor_speeds` - Motor speed data
- `/hardware/temperature` - Temperature
- `/hardware/status` - Combined status

Monitor:
```bash
ros2 topic echo /hardware/battery
```

### 2. Kinematics Processor
```bash
kinematics_processor
```
Subscribes to: `/cmd_vel`
Publishes to: `/odom` (odometry)

Send velocity command:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.3}}"
```

### 3. Config Manager
```bash
config_manager
```
Publishes robot configuration as JSON to `/robot/config`

Get config:
```bash
ros2 service call /robot/get_config std_srvs/Trigger
```

### 4. Basic Publisher/Subscriber (Examples)
```bash
# Terminal 1
robot_talker

# Terminal 2
robot_listener
```

## List All Topics
```bash
ros2 topic list -t
```

## Echo a Topic
```bash
ros2 topic echo /topic_name
```

## ROS 2 Node Info
```bash
ros2 node list
ros2 node info /node_name
```

## Build After Changes
```bash
cd /root/Otomasi/Sistem-Otomasi-Robot/Robot/ros2_ws
colcon build --packages-select robot_example
source install/setup.bash
```

---

**All robotwin functionality integrated into ROS 2!**
