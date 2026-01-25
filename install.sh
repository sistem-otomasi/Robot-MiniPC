#!/bin/bash
#
# RoboTwin Installation Script
# Installs all dependencies for Ubuntu Mini PC
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
check_root() {
    if [ "$EUID" -ne 0 ]; then
        print_warning "Script not running as root. Some operations may fail."
        SUDO="sudo"
    else
        SUDO=""
    fi
}

# Update system packages
update_system() {
    print_status "Updating system packages..."
    $SUDO apt-get update
    $SUDO apt-get upgrade -y
}

# Install system dependencies
install_system_deps() {
    print_status "Installing system dependencies..."
    
    $SUDO apt-get install -y \
        build-essential \
        cmake \
        git \
        wget \
        curl \
        python3 \
        python3-pip \
        python3-venv \
        python3-dev \
        libopencv-dev \
        libusb-1.0-0-dev \
        libudev-dev \
        i2c-tools \
        can-utils \
        socat \
        screen \
        htop \
        net-tools \
        v4l-utils \
        ffmpeg
}

# Setup udev rules for hardware
setup_udev_rules() {
    print_status "Setting up udev rules..."
    
    # RPLidar rules
    cat << 'EOF' | $SUDO tee /etc/udev/rules.d/90-rplidar.rules
# RPLidar A1/A2/A3
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="rplidar"
# YDLidar
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"
EOF

    # Camera rules
    cat << 'EOF' | $SUDO tee /etc/udev/rules.d/90-camera.rules
# USB Camera
SUBSYSTEM=="video4linux", ATTR{index}=="0", MODE="0666", GROUP="video"
EOF

    # Reload udev rules
    $SUDO udevadm control --reload-rules
    $SUDO udevadm trigger
    
    print_status "Udev rules installed"
}

# Setup user permissions
setup_permissions() {
    print_status "Setting up user permissions..."
    
    # Add user to required groups
    $SUDO usermod -a -G dialout,video,i2c,gpio $USER || true
    
    print_warning "You may need to logout and login again for group changes to take effect"
}

# Setup Python environment
setup_python() {
    print_status "Setting up Python environment..."
    
    # Create virtual environment
    VENV_DIR="${HOME}/.venvs/robotwin"
    
    if [ ! -d "$VENV_DIR" ]; then
        python3 -m venv "$VENV_DIR"
    fi
    
    # Activate venv and install packages
    source "$VENV_DIR/bin/activate"
    
    pip install --upgrade pip setuptools wheel
    
    # Install robotwin package
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    cd "$SCRIPT_DIR"
    
    pip install -e .
    
    print_status "Python environment ready at: $VENV_DIR"
    print_status "Activate with: source $VENV_DIR/bin/activate"
}

# Install ROS2 (optional)
install_ros2() {
    print_status "Installing ROS2 Humble..."
    
    # Check if ROS2 already installed
    if [ -d "/opt/ros/humble" ]; then
        print_status "ROS2 Humble already installed"
        return
    fi
    
    # Setup sources
    $SUDO apt-get install -y software-properties-common
    $SUDO add-apt-repository universe
    
    $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | $SUDO tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    $SUDO apt-get update
    
    # Install ROS2 base (not full desktop)
    $SUDO apt-get install -y \
        ros-humble-ros-base \
        ros-humble-rosbridge-suite \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-tf2-ros \
        python3-colcon-common-extensions
    
    # Setup environment
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    
    print_status "ROS2 Humble installed"
}

# Setup CAN bus
setup_can() {
    print_status "Setting up CAN bus..."
    
    # Load CAN modules
    $SUDO modprobe can
    $SUDO modprobe can_raw
    $SUDO modprobe mcp251x || true
    $SUDO modprobe slcan || true
    
    # Create CAN interface setup script
    cat << 'EOF' | $SUDO tee /usr/local/bin/setup-can.sh
#!/bin/bash
# Setup CAN interface

CAN_INTERFACE="${1:-can0}"
BITRATE="${2:-500000}"

# For USB CAN adapter (slcan)
if [ -e /dev/ttyACM0 ]; then
    sudo slcand -o -c -f -s6 /dev/ttyACM0 $CAN_INTERFACE
fi

# Bring up interface
sudo ip link set $CAN_INTERFACE type can bitrate $BITRATE
sudo ip link set up $CAN_INTERFACE

echo "CAN interface $CAN_INTERFACE ready"
EOF
    $SUDO chmod +x /usr/local/bin/setup-can.sh
    
    print_status "CAN setup script installed at /usr/local/bin/setup-can.sh"
}

# Setup I2C
setup_i2c() {
    print_status "Setting up I2C..."
    
    # Enable I2C if on Raspberry Pi
    if [ -f /boot/config.txt ]; then
        if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
            echo "dtparam=i2c_arm=on" | $SUDO tee -a /boot/config.txt
        fi
    fi
    
    # Load I2C modules
    $SUDO modprobe i2c-dev || true
    
    print_status "I2C setup complete"
}

# Create systemd service
create_service() {
    print_status "Creating systemd service..."
    
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    VENV_DIR="${HOME}/.venvs/robotwin"
    
    cat << EOF | $SUDO tee /etc/systemd/system/robotwin.service
[Unit]
Description=RoboTwin Robot Controller
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$SCRIPT_DIR
Environment="PATH=$VENV_DIR/bin:/usr/local/bin:/usr/bin:/bin"
ExecStart=$VENV_DIR/bin/robotwin run
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

    $SUDO systemctl daemon-reload
    
    print_status "Systemd service created"
    print_status "Enable with: sudo systemctl enable robotwin"
    print_status "Start with: sudo systemctl start robotwin"
}

# Print usage
usage() {
    echo "RoboTwin Installation Script"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --all          Install everything (default)"
    echo "  --system       Install system dependencies only"
    echo "  --python       Setup Python environment only"
    echo "  --ros2         Install ROS2 Humble"
    echo "  --udev         Setup udev rules only"
    echo "  --can          Setup CAN bus only"
    echo "  --i2c          Setup I2C only"
    echo "  --service      Create systemd service"
    echo "  --help         Show this help"
}

# Main
main() {
    check_root
    
    case "${1:-all}" in
        --all|all)
            update_system
            install_system_deps
            setup_udev_rules
            setup_permissions
            setup_python
            setup_can
            setup_i2c
            create_service
            ;;
        --system)
            update_system
            install_system_deps
            ;;
        --python)
            setup_python
            ;;
        --ros2)
            install_ros2
            ;;
        --udev)
            setup_udev_rules
            ;;
        --can)
            setup_can
            ;;
        --i2c)
            setup_i2c
            ;;
        --service)
            create_service
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
    
    print_status "Installation complete!"
    print_warning "Please reboot to apply all changes"
}

main "$@"
