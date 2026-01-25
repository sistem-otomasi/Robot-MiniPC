# RoboTwin Robot Core System

Sistem kontrol robot untuk Mini PC Ubuntu yang terintegrasi dengan RoboTwin Platform.

## Arsitektur Sistem

```
┌─────────────────────────────────────────────────────────────────┐
│                        MINI PC UBUNTU                            │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                    RoboTwin Core                         │    │
│  │  ┌─────────────┬──────────────┬─────────────────────┐   │    │
│  │  │  Sensors    │   Control    │   Communication     │   │    │
│  │  ├─────────────┼──────────────┼─────────────────────┤   │    │
│  │  │ • LIDAR     │ • PID        │ • WebSocket Client  │   │    │
│  │  │ • Camera    │ • Kinematics │ • MQTT              │   │    │
│  │  │ • IMU       │ • Filters    │ • Serial/UART       │   │    │
│  │  │ • Encoders  │ • SLAM       │ • ROS2 Bridge       │   │    │
│  │  │ • GPS       │ • Navigation │                     │   │    │
│  │  └─────────────┴──────────────┴─────────────────────┘   │    │
│  └─────────────────────────────────────────────────────────┘    │
│                              │                                   │
│                              ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                   Hardware Interface                     │    │
│  │  ┌───────────┬──────────────┬─────────────────────┐     │    │
│  │  │  Motors   │  Actuators   │    GPIO/I2C/SPI     │     │    │
│  │  └───────────┴──────────────┴─────────────────────┘     │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ WebSocket
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    RoboTwin Web Server                          │
│                (Cloud / On-Premise Server)                      │
└─────────────────────────────────────────────────────────────────┘
```

## Fitur Utama

### 🎯 Kontrol Motor
- Differential drive, Mecanum, Omni-directional
- Motor DC dengan encoder
- Stepper motor
- Servo motor

### 📡 Sensor
- LIDAR (RPLidar, Hokuyo, etc.)
- Kamera (USB, CSI, IP Camera)
- IMU (MPU6050, BNO055, etc.)
- Encoder motor
- GPS (optional)

### 🧮 Kontrol & Algoritma
- PID Controller (posisi, kecepatan, heading)
- Kalman Filter, Extended Kalman Filter
- Complementary Filter
- Low-pass, High-pass Filter
- Kinematika robot (forward/inverse)

### 🗺️ Navigasi
- SLAM integration
- Path planning
- Obstacle avoidance
- Localization

### 🔌 Komunikasi
- WebSocket ke Web Server
- MQTT untuk IoT
- Serial/UART untuk hardware
- ROS2 integration (optional)

## Instalasi

### Quick Install (Recommended)

```bash
# Clone repository (jika belum)
cd /root/Otomasi/Sistem-Otomasi-Robot/Robot

# Jalankan script instalasi
./scripts/install.sh
```

### Manual Install

```bash
# 1. System dependencies
sudo apt update
sudo apt install -y python3-pip python3-venv \
    python3-dev python3-opencv \
    libyaml-dev libhidapi-dev \
    i2c-tools

# 2. Create virtual environment
python3 -m venv venv
source venv/bin/activate

# 3. Install package
pip install -e .

# 4. (Optional) Install ML dependencies
pip install -e ".[ml]"
```

### ROS2 Integration (Optional)

```bash
# Install ROS2 Humble
./scripts/install_ros2.sh

# Build ROS2 workspace
./scripts/build_ros2.sh
```

## Konfigurasi

Edit file `config/robot_config.yaml`:

```yaml
robot:
  name: "robot-001"
  type: "differential"
  
server:
  url: "ws://your-server:3000/ws/robot"
  
motors:
  type: "dc_encoder"
  controller: "serial"
  port: "/dev/ttyUSB0"
```

## Menjalankan

```bash
# Aktifkan virtual environment
source venv/bin/activate

# Jalankan robot
robotwin start

# Atau dengan config tertentu
robotwin start --config /path/to/config.yaml

# Mode debug
robotwin start --debug

# Jalankan sebagai service
sudo systemctl start robotwin
```

## Struktur Direktori

```
Robot/
├── src/robotwin/           # Source code utama
│   ├── core/               # Core modules
│   │   ├── robot.py        # Robot class utama
│   │   └── state.py        # State management
│   ├── hardware/           # Hardware interface
│   │   ├── motors/         # Motor controllers
│   │   ├── sensors/        # Sensor drivers
│   │   └── gpio/           # GPIO interface
│   ├── control/            # Control algorithms
│   │   ├── pid.py          # PID controller
│   │   ├── kinematics/     # Robot kinematics
│   │   └── filters/        # Signal filters
│   ├── navigation/         # Navigation modules
│   │   ├── slam.py         # SLAM integration
│   │   └── planner.py      # Path planning
│   ├── communication/      # Communication layer
│   │   ├── websocket.py    # WebSocket client
│   │   └── mqtt.py         # MQTT client
│   └── utils/              # Utilities
├── config/                 # Configuration files
├── scripts/                # Installation & utility scripts
├── tests/                  # Unit & integration tests
└── ros2_ws/               # ROS2 workspace (optional)
```

## API Documentation

### Robot Control

```python
from robotwin.core import Robot

# Initialize robot
robot = Robot("config/robot_config.yaml")

# Start robot
await robot.start()

# Move robot
await robot.move(linear=0.5, angular=0.1)

# Stop robot
await robot.stop()
```

### Motor Control

```python
from robotwin.hardware.motors import DifferentialDrive

drive = DifferentialDrive(config)
await drive.set_velocity(left=0.5, right=0.5)
```

### Sensor Access

```python
from robotwin.hardware.sensors import Lidar, Camera

lidar = Lidar(config)
scan = await lidar.get_scan()

camera = Camera(config)
frame = await camera.capture()
```

## Service Management

```bash
# Install service
sudo ./scripts/install_service.sh

# Start service
sudo systemctl start robotwin

# Enable on boot
sudo systemctl enable robotwin

# Check status
sudo systemctl status robotwin

# View logs
journalctl -u robotwin -f
```

## Troubleshooting

### Serial Port Permission
```bash
sudo usermod -a -G dialout $USER
# Logout dan login kembali
```

### I2C Permission
```bash
sudo usermod -a -G i2c $USER
```

### Camera Permission
```bash
sudo usermod -a -G video $USER
```

## License

MIT License
