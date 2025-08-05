#!/usr/bin/env python3
"""
Raspberry Pi Setup Test Script
Tests all components of the following robot system on Raspberry Pi
"""

import sys
import time
import platform
import os

def test_system_info():
    """Test system information"""
    print("=== System Information ===")
    print(f"Platform: {platform.system()} {platform.release()}")
    print(f"Architecture: {platform.machine()}")
    print(f"Python: {platform.python_version()}")
    
    # Check if running on Raspberry Pi
    is_pi = platform.machine().startswith('arm') and os.path.exists('/proc/cpuinfo')
    print(f"Raspberry Pi: {is_pi}")
    
    if is_pi:
        # Read Pi-specific info
        try:
            with open('/proc/cpuinfo', 'r') as f:
                cpu_info = f.read()
                if 'Raspberry Pi' in cpu_info:
                    print("✅ Raspberry Pi detected")
                else:
                    print("⚠️  ARM system but may not be Raspberry Pi")
        except:
            print("❌ Could not read CPU info")
    
    return is_pi

def test_dependencies():
    """Test Python dependencies"""
    print("\n=== Python Dependencies ===")
    
    dependencies = [
        ('cv2', 'OpenCV'),
        ('numpy', 'NumPy'),
        ('ultralytics', 'Ultralytics (YOLO)'),
        ('serial', 'PySerial'),
        ('picamera', 'PiCamera (optional)'),
        ('picamera2', 'PiCamera2 (optional)')
    ]
    
    missing = []
    for module, name in dependencies:
        try:
            __import__(module)
            print(f"✅ {name}")
        except ImportError:
            print(f"❌ {name} - Missing")
            missing.append(name)
    
    return len(missing) == 0

def test_camera():
    """Test camera access"""
    print("\n=== Camera Test ===")
    
    try:
        import cv2
        
        # Try USB camera first
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"✅ USB Camera working - Resolution: {frame.shape[1]}x{frame.shape[0]}")
                cap.release()
                return True
            else:
                print("❌ USB Camera not reading frames")
                cap.release()
        else:
            print("❌ USB Camera not accessible")
    except Exception as e:
        print(f"❌ Camera test failed: {e}")
    
    # Try PiCamera
    try:
        import picamera
        print("✅ PiCamera module available")
        return True
    except ImportError:
        try:
            import picamera2
            print("✅ PiCamera2 module available")
            return True
        except ImportError:
            print("❌ No camera modules available")
            return False

def test_serial_ports():
    """Test serial port availability"""
    print("\n=== Serial Ports Test ===")
    
    common_ports = [
        '/dev/ttyUSB0',
        '/dev/ttyUSB1', 
        '/dev/ttyACM0',
        '/dev/ttyACM1',
        '/dev/ttyS0',
        '/dev/ttyS1'
    ]
    
    available_ports = []
    for port in common_ports:
        if os.path.exists(port):
            print(f"✅ {port} - Available")
            available_ports.append(port)
        else:
            print(f"❌ {port} - Not found")
    
    if available_ports:
        print(f"\nAvailable serial ports: {available_ports}")
        return True
    else:
        print("❌ No serial ports found")
        return False

def test_yolo_model():
    """Test YOLO model availability"""
    print("\n=== YOLO Model Test ===")
    
    models = ['yolov10n.pt', 'yolov8n.pt']
    
    for model in models:
        if os.path.exists(model):
            size_mb = os.path.getsize(model) / (1024 * 1024)
            print(f"✅ {model} - {size_mb:.1f} MB")
            return True
        else:
            print(f"❌ {model} - Not found")
    
    print("❌ No YOLO models found")
    return False

def test_memory():
    """Test available memory"""
    print("\n=== Memory Test ===")
    
    try:
        import psutil
        memory = psutil.virtual_memory()
        total_gb = memory.total / (1024**3)
        available_gb = memory.available / (1024**3)
        
        print(f"Total RAM: {total_gb:.1f} GB")
        print(f"Available RAM: {available_gb:.1f} GB")
        print(f"Memory usage: {memory.percent}%")
        
        if total_gb >= 3.5:  # 4GB Pi
            print("✅ Sufficient RAM for Pi 4")
            return True
        elif total_gb >= 1.5:  # 2GB Pi
            print("⚠️  Limited RAM - may need optimization")
            return True
        else:
            print("❌ Insufficient RAM")
            return False
    except ImportError:
        print("⚠️  psutil not available - cannot check memory")
        return True

def test_gpu():
    """Test GPU capabilities"""
    print("\n=== GPU Test ===")
    
    try:
        import cv2
        gpu_count = cv2.cuda.getCudaEnabledDeviceCount()
        if gpu_count > 0:
            print(f"✅ CUDA GPU available - {gpu_count} device(s)")
            return True
        else:
            print("ℹ️  No CUDA GPU - will use CPU")
            return True
    except:
        print("ℹ️  OpenCV CUDA not available - will use CPU")
        return True

def main():
    """Run all tests"""
    print("Raspberry Pi Setup Test")
    print("=" * 40)
    
    tests = [
        ("System Information", test_system_info),
        ("Dependencies", test_dependencies),
        ("Camera", test_camera),
        ("Serial Ports", test_serial_ports),
        ("YOLO Model", test_yolo_model),
        ("Memory", test_memory),
        ("GPU", test_gpu)
    ]
    
    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ {test_name} test failed: {e}")
            results.append((test_name, False))
    
    # Summary
    print("\n" + "=" * 40)
    print("TEST SUMMARY")
    print("=" * 40)
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\nPassed: {passed}/{total}")
    
    if passed == total:
        print("🎉 All tests passed! Your Raspberry Pi is ready for the following robot.")
    elif passed >= total - 1:
        print("⚠️  Most tests passed. Check the failed test and try again.")
    else:
        print("❌ Multiple tests failed. Please check your setup.")
    
    print("\nNext steps:")
    if passed >= total - 1:
        print("1. Connect your Arduino")
        print("2. Connect your camera")
        print("3. Run: python3 main.py")
    else:
        print("1. Run the setup script: ./setup_pi.sh")
        print("2. Reboot your Raspberry Pi")
        print("3. Run this test again")

if __name__ == "__main__":
    main() 