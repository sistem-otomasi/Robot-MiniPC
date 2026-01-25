#!/bin/bash
#
# RoboTwin Run Script
# Starts the robot controller with appropriate environment
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${HOME}/.venvs/robotwin"

# Colors
GREEN='\033[0;32m'
NC='\033[0m'

print_status() {
    echo -e "${GREEN}[RoboTwin]${NC} $1"
}

# Activate virtual environment
if [ -d "$VENV_DIR" ]; then
    source "$VENV_DIR/bin/activate"
fi

# Change to script directory
cd "$SCRIPT_DIR"

# Run the robot
print_status "Starting RoboTwin Robot Controller..."

exec python -m robotwin.cli "$@"
