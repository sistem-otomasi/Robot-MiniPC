# ROS 2 Robot Example Package

Paket contoh untuk demonstrasi ROS 2 Python dengan ament dan colcon.

## Struktur

```
robot_example/
├── robot_example/
│   ├── __init__.py
│   ├── talker.py      # Publisher node
│   └── listener.py    # Subscriber node
├── test/
├── package.xml        # Package metadata (ament)
├── setup.py          # Python setup file
├── setup.cfg         # Setup configuration
└── README.md
```

## Dependencies

- `rclpy` - ROS 2 Python client library
- `std_msgs` - Standard message types

## Building

Dari direktori `ros2_ws`:

```bash
colcon build
```

## Testing

### 1. Source setup file:
```bash
source install/setup.bash
```

### 2. Run talker (di terminal 1):
```bash
ros2 run robot_example robot_talker
```

### 3. Run listener (di terminal 2):
```bash
ros2 run robot_example robot_listener
```

## Development

### Struktur ROS 2 Python Package (ament):
- `package.xml` - Metadata dan dependencies
- `setup.py` - Python package configuration
- `setup.cfg` - Python setup configuration
- `robot_example/` - Modul Python

### Colcon workflow:
```bash
# Build semua packages
colcon build

# Build satu package
colcon build --packages-select robot_example

# Build dengan testing
colcon build --packages-select robot_example --symlink-install

# Clean build artifacts
colcon clean
```
