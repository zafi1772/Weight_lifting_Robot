import cv2
import time

def test_camera(index):
    """Test if camera at given index works"""
    print(f"Testing camera index {index}...")
    cap = cv2.VideoCapture(index)
    
    if not cap.isOpened():
        print(f"  ❌ Camera {index} failed to open")
        return False
    
    # Try to read a frame
    ret, frame = cap.read()
    if not ret:
        print(f"  ❌ Camera {index} opened but can't read frames")
        cap.release()
        return False
    
    print(f"  ✅ Camera {index} works! Frame size: {frame.shape}")
    cap.release()
    return True

def main():
    print("Testing available cameras...")
    print("=" * 40)
    
    working_cameras = []
    
    # Test indices 0-5
    for i in range(6):
        if test_camera(i):
            working_cameras.append(i)
        time.sleep(0.5)  # Small delay between tests
    
    print("=" * 40)
    if working_cameras:
        print(f"✅ Found {len(working_cameras)} working camera(s): {working_cameras}")
        print(f"Use --source {working_cameras[0]} in your detection script")
    else:
        print("❌ No cameras found!")
        print("Try:")
        print("1. Check if camera is connected")
        print("2. Close other apps using the camera")
        print("3. Check Windows camera permissions")

if __name__ == "__main__":
    main() 