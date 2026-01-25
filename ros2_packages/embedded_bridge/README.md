# ROS2 Embedded Bridge Package

Bridge untuk komunikasi antara embedded devices (ESP32/STM32) dengan ROS2 system via MQTT.

## Overview

Package ini menyediakan:
- MQTT client yang terhubung dengan embedded devices
- Translasi message dari MQTT ke ROS2 topics
- Translasi command dari ROS2 ke MQTT
- Real-time sensor data publishing
- Device heartbeat monitoring

## Instalasi

### Prerequisites

```bash
# Install ROS2 dependencies
sudo apt install ros-$ROS_DISTRO-mosquitto-*
sudo apt install nlohmann-json3-dev
sudo apt install libmosquitto-dev

# atau jika menggunakan rosdep
rosdep install --from-paths src --ignore-src -y
```

### Build

```bash
cd ~/robot_ws
colcon build --packages-select embedded_bridge
source install/setup.bash
```

## Usage

### Launch dengan default config
```bash
ros2 launch embedded_bridge embedded_bridge.launch.py
```

### Launch dengan custom MQTT broker
```bash
ros2 launch embedded_bridge embedded_bridge.launch.py mqtt_broker:=192.168.1.100
```

### Test

#### Subscribe ke sensor data
```bash
ros2 topic echo /robot/embedded/sensors
```

#### Publish control command
```bash
ros2 topic pub /robot/control/command std_msgs/String "data: '{\"led\": true}'"
```

#### Monitor dengan rqt
```bash
rqt
# Tools > Topic Monitor
```

## Topics

### Publishers (dari embedded -> ROS)
- `/robot/embedded/heartbeat` (String) - Device heartbeat signal
- `/robot/embedded/sensors` (JointState) - Sensor readings
- `/robot/embedded/status` (String) - Device status

### Subscribers (dari ROS -> embedded)
- `/robot/control/command` (String) - Control commands (JSON)

## Message Format

### Heartbeat
```json
{
  "device_id": "esp32_embedded_01",
  "timestamp": 120000,
  "uptime_ms": 120000,
  "rssi": -45,
  "ip": "192.168.1.50"
}
```

### Sensors
```json
{
  "device_id": "esp32_embedded_01",
  "timestamp": 120000,
  "adc_raw": 2048,
  "voltage": 1.65,
  "temperature": 25.5,
  "humidity": 60.0
}
```

### Status
```json
{
  "action": "led_set",
  "state": true,
  "timestamp": 120000
}
```

### Control Command
```json
{
  "led": true,
  "blink": 5,
  "delay": 500
}
```

## Configuration

Edit `config/embedded_bridge.yaml`:

```yaml
mqtt_broker: "192.168.1.100"  # MQTT broker IP
mqtt_port: 1883
mqtt_qos: 1                   # Quality of Service (0, 1, or 2)
```

## Troubleshooting

### MQTT Connection Refused
```bash
# Check MQTT broker status
mosquitto -v

# atau cek dengan mosquitto_pub
mosquitto_pub -h 192.168.1.100 -t test/topic -m "hello"
```

### No data from embedded device
```bash
# Subscribe ke MQTT topics directly
mosquitto_sub -h 192.168.1.100 -t "robot/embedded/#"

# Verify device telah publish
mosquitto_pub -h 192.168.1.100 -t "robot/embedded/test" -m "test"
```

### ROS not receiving data
```bash
# Check ROS topics
ros2 topic list
ros2 topic echo /robot/embedded/sensors

# Check node status
ros2 node list
ros2 node info /embedded_bridge
```

## Performance

- MQTT Loop: 100ms interval
- Heartbeat: Dari embedded setiap 5 detik
- Sensor Reading: Dari embedded setiap 2 detik
- QoS: 1 (at least once delivery)

## Future Enhancements

- [ ] Support multiple embedded devices
- [ ] Device discovery dan auto-registration
- [ ] Advanced filtering dan routing
- [ ] Data logging ke database
- [ ] Web dashboard integration
- [ ] TLS/SSL encryption
- [ ] Authentication support
