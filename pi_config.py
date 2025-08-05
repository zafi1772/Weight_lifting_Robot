#!/usr/bin/env python3
"""
Raspberry Pi Configuration Settings
Optimized settings for Raspberry Pi 4 (4GB RAM)
"""

import platform
import os

class PiConfig:
    """Raspberry Pi specific configuration settings"""
    
    # System detection
    IS_RASPBERRY_PI = platform.machine().startswith('arm') and os.path.exists('/proc/cpuinfo')
    
    # Camera settings optimized for Pi 4
    CAMERA_RESOLUTION = (640, 480)  # Reduced for better performance
    CAMERA_FRAMERATE = 30
    CAMERA_BUFFER_SIZE = 1  # Minimal buffer for real-time processing
    
    # Performance optimization
    DETECTION_INTERVAL = 3  # Process every 3rd frame
    HISTORY_LENGTH = 10  # Reduced from 15 for memory optimization
    CONFIDENCE_THRESHOLD = 0.3
    
    # Serial communication settings
    SERIAL_BAUD_RATE = 115200
    SERIAL_TIMEOUT = 1
    
    # Common Raspberry Pi serial ports (in order of preference)
    SERIAL_PORTS = [
        '/dev/ttyUSB0',
        '/dev/ttyUSB1', 
        '/dev/ttyACM0',
        '/dev/ttyACM1',
        '/dev/ttyS0',
        '/dev/ttyS1'
    ]
    
    # Display settings
    DEFAULT_WINDOW_SIZE = (640, 480)
    FULLSCREEN_DISABLED_BY_DEFAULT = True  # Better for Pi performance
    
    # Memory optimization
    MAX_FRAME_BUFFER = 2  # Limit frame buffering
    ENABLE_FRAME_SKIPPING = True
    
    # YOLO model settings
    YOLO_MODEL_PRIORITY = ['yolov10n.pt', 'yolov8n.pt']
    
    # Distance estimation calibration for Pi camera
    KNOWN_PERSON_HEIGHT_PIXELS = 200
    KNOWN_DISTANCE_CM = 100
    ACTUAL_PERSON_HEIGHT_CM = 170
    
    # Safety thresholds
    TOO_CLOSE_THRESHOLD_CM = 29
    GOOD_DISTANCE_MIN_CM = 29
    GOOD_DISTANCE_MAX_CM = 31
    TOO_FAR_THRESHOLD_CM = 31
    
    # PWM settings
    PWM_FAR = 200
    PWM_IDLE = 150
    PWM_CLOSE = 0
    
    @classmethod
    def get_optimized_settings(cls):
        """Get settings optimized for current system"""
        if cls.IS_RASPBERRY_PI:
            return {
                'camera_resolution': cls.CAMERA_RESOLUTION,
                'detection_interval': cls.DETECTION_INTERVAL,
                'history_length': cls.HISTORY_LENGTH,
                'frame_skipping': cls.ENABLE_FRAME_SKIPPING,
                'fullscreen_disabled': cls.FULLSCREEN_DISABLED_BY_DEFAULT
            }
        else:
            # Default settings for non-Pi systems
            return {
                'camera_resolution': (1280, 720),
                'detection_interval': 1,
                'history_length': 15,
                'frame_skipping': False,
                'fullscreen_disabled': False
            }
    
    @classmethod
    def get_serial_ports(cls):
        """Get available serial ports for Arduino connection"""
        available_ports = []
        for port in cls.SERIAL_PORTS:
            if os.path.exists(port):
                available_ports.append(port)
        return available_ports
    
    @classmethod
    def print_system_info(cls):
        """Print system information for debugging"""
        print(f"Platform: {platform.system()} {platform.release()}")
        print(f"Architecture: {platform.machine()}")
        print(f"Python: {platform.python_version()}")
        print(f"Raspberry Pi: {cls.IS_RASPBERRY_PI}")
        
        if cls.IS_RASPBERRY_PI:
            print("Raspberry Pi optimizations enabled")
            print(f"Camera resolution: {cls.CAMERA_RESOLUTION}")
            print(f"Detection interval: {cls.DETECTION_INTERVAL}")
            print(f"History length: {cls.HISTORY_LENGTH}")
        else:
            print("Standard settings for non-Pi system")

# Global configuration instance
pi_config = PiConfig() 