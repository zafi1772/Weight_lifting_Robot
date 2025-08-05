#!/bin/bash
# Raspberry Pi 4 Setup Script for Following Robot
# This script installs all necessary dependencies and configures the system

echo "=== Raspberry Pi 4 Setup for Following Robot ==="
echo "This script will install dependencies and configure your Pi for the robot project"
echo ""

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "Warning: This script is designed for Raspberry Pi. Continue anyway? (y/n)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Setup cancelled."
        exit 1
    fi
fi

# Update system
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install system dependencies
echo "Installing system dependencies..."
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-venv \
    libatlas-base-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libatlas-base-dev \
    libjasper-dev \
    libqtcore4 \
    libqtgui4 \
    libqt4-test \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    libgtk-3-0 \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libatlas-base-dev \
    gfortran \
    libopenblas-dev \
    liblapack-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libhdf5-103 \
    libqtgui4 \
    libqtwebkit4 \
    libqt4-test \
    python3-pyqt5 \
    libgtk-3-0 \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libatlas-base-dev \
    gfortran \
    libopenblas-dev \
    liblapack-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libhdf5-103 \
    libqtgui4 \
    libqtwebkit4 \
    libqt4-test \
    python3-pyqt5

# Enable camera interface
echo "Enabling camera interface..."
sudo raspi-config nonint do_camera 0

# Enable I2C and SPI (for future sensor integration)
echo "Enabling I2C and SPI interfaces..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0

# Enable serial interface for Arduino communication
echo "Enabling serial interface..."
sudo raspi-config nonint do_serial 0

# Remove serial console to free up /dev/ttyAMA0
echo "Removing serial console to free up serial port..."
sudo sed -i 's/console=serial0,115200 //' /boot/config.txt

# Create Python virtual environment
echo "Creating Python virtual environment..."
python3 -m venv robot_env
source robot_env/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install Python dependencies
echo "Installing Python dependencies..."
pip install -r requirements.txt

# Install PiCamera2 (newer PiCamera library)
echo "Installing PiCamera2..."
sudo apt install -y python3-picamera2

# Set up udev rules for Arduino
echo "Setting up udev rules for Arduino..."
sudo tee /etc/udev/rules.d/99-arduino.rules > /dev/null <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="*", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="*", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="*", MODE="0666"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Create startup script
echo "Creating startup script..."
cat > start_robot.sh << 'EOF'
#!/bin/bash
# Startup script for Following Robot on Raspberry Pi

echo "Starting Following Robot..."
echo "Make sure Arduino is connected and camera is ready"

# Activate virtual environment
source robot_env/bin/activate

# Run the robot
python3 main.py
EOF

chmod +x start_robot.sh

# Create systemd service (optional)
echo "Creating systemd service for auto-startup..."
sudo tee /etc/systemd/system/following-robot.service > /dev/null <<EOF
[Unit]
Description=Following Robot Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=$(pwd)
ExecStart=$(pwd)/start_robot.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Enable service (optional - uncomment to enable auto-startup)
# sudo systemctl enable following-robot.service

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Next steps:"
echo "1. Reboot your Raspberry Pi: sudo reboot"
echo "2. Connect your Arduino to a USB port"
echo "3. Connect your camera (PiCamera or USB camera)"
echo "4. Run the robot: ./start_robot.sh"
echo ""
echo "Optional: Enable auto-startup with: sudo systemctl enable following-robot.service"
echo ""
echo "Troubleshooting:"
echo "- Check camera: vcgencmd get_camera"
echo "- Check serial ports: ls -la /dev/tty*"
echo "- Check USB devices: lsusb"
echo "" 