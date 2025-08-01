#!/usr/bin/env python3
"""
Live Human Detection with GUI using YOLO 10
Real-time human detection with position tracking and movement detection
"""

import cv2
import numpy as np
import time
from ultralytics import YOLO
from collections import deque
import math

class PositionTracker:
    def __init__(self, history_length=10):
        self.history = deque(maxlen=history_length)
        self.center_x = None
        self.center_y = None
        self.frame_width = None
        self.frame_height = None
        self.distance_history = deque(maxlen=history_length)
        self.movement_history = deque(maxlen=history_length)
        self.last_update_time = time.time()
        
        # Calibration parameters for distance estimation
        self.known_person_height_pixels = 200  # Reference height in pixels at known distance
        self.known_distance_cm = 100  # Known distance in cm
        self.actual_person_height_cm = 170  # Average person height in cm
        
    def update_frame_size(self, width, height):
        self.frame_width = width
        self.frame_height = height
        self.center_x = width // 2
        self.center_y = height // 2
    
    def calculate_distance_to_center(self, center_x, center_y):
        """Calculate pixel distance from detection center to frame center"""
        dx = center_x - self.center_x
        dy = center_y - self.center_y
        return math.sqrt(dx*dx + dy*dy)
    
    def estimate_real_distance(self, bbox):
        """Estimate real-world distance using bounding box size"""
        x1, y1, x2, y2 = bbox
        person_height_pixels = y2 - y1
        
        if person_height_pixels <= 0:
            return None
            
        # Use inverse relationship: distance = known_distance * (known_height / current_height)
        estimated_distance_cm = self.known_distance_cm * (self.known_person_height_pixels / person_height_pixels)
        return estimated_distance_cm
    
    def calculate_movement_distance(self, pos1, pos2):
        """Calculate movement distance between two positions"""
        dx = pos2['center_x'] - pos1['center_x']
        dy = pos2['center_y'] - pos1['center_y']
        return math.sqrt(dx*dx + dy*dy)
    
    def calculate_movement_speed(self, movement_distance, time_delta):
        """Calculate movement speed in pixels per second"""
        if time_delta > 0:
            return movement_distance / time_delta
        return 0
    
    def add_detection(self, bbox):
        """Add detection to history and return position info with distance measurements"""
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        bbox_area = (x2 - x1) * (y2 - y1)
        
        # Calculate distances
        pixel_distance_to_center = self.calculate_distance_to_center(center_x, center_y)
        estimated_real_distance = self.estimate_real_distance(bbox)
        
        current_time = time.time()
        time_delta = current_time - self.last_update_time
        
        detection_data = {
            'center_x': center_x,
            'center_y': center_y,
            'area': bbox_area,
            'bbox': bbox,
            'pixel_distance_to_center': pixel_distance_to_center,
            'estimated_real_distance_cm': estimated_real_distance,
            'timestamp': current_time,
            'time_delta': time_delta
        }
        
        self.history.append(detection_data)
        self.distance_history.append(pixel_distance_to_center)
        
        # Calculate movement metrics
        movement_metrics = self.calculate_movement_metrics()
        detection_data.update(movement_metrics)
        
        self.last_update_time = current_time
        
        return self.analyze_position(center_x, center_y, bbox_area, detection_data)
    
    def calculate_movement_metrics(self):
        """Calculate comprehensive movement metrics"""
        if len(self.history) < 2:
            return {
                'movement_distance_pixels': 0,
                'movement_speed_pixels_per_sec': 0,
                'total_distance_traveled': 0,
                'average_speed': 0,
                'movement_direction_degrees': 0
            }
        
        # Get current and previous positions
        current = self.history[-1]
        previous = self.history[-2]
        
        # Calculate movement distance
        movement_distance = self.calculate_movement_distance(previous, current)
        
        # Calculate movement speed
        movement_speed = self.calculate_movement_speed(movement_distance, current['time_delta'])
        
        # Calculate total distance traveled
        total_distance = sum([
            self.calculate_movement_distance(self.history[i], self.history[i+1])
            for i in range(len(self.history)-1)
        ])
        
        # Calculate average speed
        total_time = sum([h['time_delta'] for h in list(self.history)[1:]])
        average_speed = total_distance / total_time if total_time > 0 else 0
        
        # Calculate movement direction in degrees
        dx = current['center_x'] - previous['center_x']
        dy = current['center_y'] - previous['center_y']
        direction_degrees = math.degrees(math.atan2(dy, dx))
        if direction_degrees < 0:
            direction_degrees += 360
        
        return {
            'movement_distance_pixels': movement_distance,
            'movement_speed_pixels_per_sec': movement_speed,
            'total_distance_traveled': total_distance,
            'average_speed': average_speed,
            'movement_direction_degrees': direction_degrees
        }
    
    def analyze_position(self, center_x, center_y, area, detection_data):
        """Analyze current position and movement with numerical data using real-world distance thresholds"""
        if self.frame_width is None:
            return "Unknown", "Unknown", "Unknown", detection_data
        
        # Position analysis
        x_zone = self.frame_width // 3
        if center_x < x_zone:
            horizontal_pos = "LEFT"
        elif center_x > 2 * x_zone:
            horizontal_pos = "RIGHT"
        else:
            horizontal_pos = "CENTER"
        
        # Distance analysis based on real-world distance in centimeters
        real_distance = detection_data['estimated_real_distance_cm']
        
        # Real-world distance thresholds:
        # Less than 29 cm = TOO CLOSE
        # 29-31 cm = GOOD DISTANCE (IDLE zone)
        # More than 31 cm = TOO FAR
        if real_distance is None:
            # Fallback to pixel distance if real distance estimation fails
            pixel_distance = detection_data['pixel_distance_to_center']
            if pixel_distance < 27:
                distance = "TOO CLOSE"
            elif pixel_distance <= 50:
                distance = "GOOD DISTANCE"
            else:
                distance = "TOO FAR"
        else:
            if real_distance < 29:
                distance = "TOO CLOSE"
            elif real_distance <= 31:
                distance = "GOOD DISTANCE"
            else:
                distance = "TOO FAR"
        
        # Movement analysis with numerical data
        movement = self.analyze_movement_with_numbers(detection_data)
        
        return horizontal_pos, distance, movement, detection_data
    
    def analyze_movement_with_numbers(self, detection_data):
        """Analyze movement direction based on history with numerical data"""
        if len(self.history) < 3:
            return "IDLE"
        
        # Get recent positions
        recent = list(self.history)[-3:]
        
        # Calculate movement direction
        x_movement = recent[-1]['center_x'] - recent[0]['center_x']
        y_movement = recent[-1]['center_y'] - recent[0]['center_y']
        
        # Threshold for movement detection
        threshold = 20
        
        if abs(x_movement) < threshold and abs(y_movement) < threshold:
            return "IDLE"
        elif abs(x_movement) > abs(y_movement):
            if x_movement > threshold:
                return "GOING RIGHT"
            else:
                return "GOING LEFT"
        else:
            if y_movement > threshold:
                return "GOING DOWN"
            else:
                return "GOING UP"

def detect_humans_yolo(frame, model, tracker):
    """Detect humans using YOLO 10 with position tracking and distance measurement"""
    detections = []
    
    # Update tracker with frame size
    tracker.update_frame_size(frame.shape[1], frame.shape[0])
    
    # Run YOLO detection
    results = model(frame, conf=0.3, verbose=False)
    
    # Process detections
    for result in results:
        if result.boxes is not None:
            for box in result.boxes:
                # Get box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                
                # Get class and confidence
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                
                # Get class name
                class_name = model.names[class_id]
                
                # Only process human-related detections
                if 'person' in class_name.lower():
                    # Analyze position and movement with distance measurements
                    horizontal_pos, distance, movement, detection_data = tracker.add_detection((x1, y1, x2, y2))
                    
                    detections.append({
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': (x1, y1, x2, y2),
                        'position': horizontal_pos,
                        'distance': distance,
                        'movement': movement,
                        'pixel_distance_to_center': detection_data['pixel_distance_to_center'],
                        'estimated_real_distance_cm': detection_data['estimated_real_distance_cm'],
                        'movement_distance_pixels': detection_data['movement_distance_pixels'],
                        'movement_speed_pixels_per_sec': detection_data['movement_speed_pixels_per_sec'],
                        'total_distance_traveled': detection_data['total_distance_traveled'],
                        'average_speed': detection_data['average_speed'],
                        'movement_direction_degrees': detection_data['movement_direction_degrees']
                    })
                    
                    # Draw bounding box with color based on position
                    color = get_position_color(horizontal_pos, distance)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    
                    # Add detailed label with distance information
                    real_distance = detection_data['estimated_real_distance_cm']
                    pixel_distance = detection_data['pixel_distance_to_center']
                    speed = detection_data['movement_speed_pixels_per_sec']
                    
                    if real_distance is not None:
                        label = f"Person: {confidence:.2f} | {horizontal_pos} | {distance} | {movement}"
                        label2 = f"Distance: {real_distance:.1f}cm | Pixel: {pixel_distance:.0f} | Speed: {speed:.1f}px/s"
                    else:
                        label = f"Person: {confidence:.2f} | {horizontal_pos} | {distance} | {movement}"
                        label2 = f"Pixel Distance: {pixel_distance:.0f} | Speed: {speed:.1f}px/s"
                    
                    cv2.putText(frame, label, (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(frame, label2, (x1, y1+10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 2)
    
    return detections

def get_position_color(horizontal_pos, distance):
    """Get color based on position and distance"""
    if distance == "TOO FAR":
        return (0, 0, 255)  # Red
    elif distance == "TOO CLOSE":
        return (0, 165, 255)  # Orange
    elif horizontal_pos == "LEFT":
        return (255, 0, 0)  # Blue
    elif horizontal_pos == "RIGHT":
        return (255, 0, 255)  # Magenta
    else:  # CENTER and GOOD DISTANCE
        return (0, 255, 0)  # Green

def get_robot_command(detection):
    """Generate robot command based on detection"""
    position = detection['position']
    distance = detection['distance']
    movement = detection['movement']
    
    # Safety first - stop if too close
    if distance == "TOO CLOSE":
        return "STOP - TOO CLOSE"
    
    # If person is too far, move forward
    if distance == "TOO FAR":
        return "MOVE FORWARD"
    
    # If person is at good distance, follow based on position
    if distance == "GOOD DISTANCE":
        if position == "LEFT":
            return "TURN LEFT"
        elif position == "RIGHT":
            return "TURN RIGHT"
        elif position == "CENTER":
            if movement == "GOING LEFT":
                return "TURN LEFT SLOWLY"
            elif movement == "GOING RIGHT":
                return "TURN RIGHT SLOWLY"
            else:  # IDLE
                return "FOLLOW - CENTER"
    
    # Default command
    return "FOLLOW"

def get_command_color(command):
    """Get color for robot command"""
    if "STOP" in command:
        return (0, 0, 255)  # Red for stop
    elif "MOVE FORWARD" in command:
        return (0, 255, 255)  # Cyan for forward
    elif "TURN LEFT" in command:
        return (255, 0, 0)  # Blue for left
    elif "TURN RIGHT" in command:
        return (255, 0, 255)  # Magenta for right
    elif "FOLLOW" in command:
        return (0, 255, 0)  # Green for follow
    else:
        return (255, 255, 255)  # White for default

def draw_status_panel(frame, detections, frame_count, total_detections, fps):
    """Draw status panel with position information, distance measurements, and robot commands"""
    # Create semi-transparent overlay
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (500, 350), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
    
    # Draw status information
    y_offset = 35
    line_height = 25
    
    # Basic stats
    cv2.putText(frame, f"Detections: {len(detections)}", (20, y_offset), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    y_offset += line_height
    
    cv2.putText(frame, f"Frame: {frame_count}", (20, y_offset), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    y_offset += line_height
    
    cv2.putText(frame, f"Total: {total_detections}", (20, y_offset), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    y_offset += line_height
    
    cv2.putText(frame, f"FPS: {fps:.1f}", (20, y_offset), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    y_offset += line_height
    
    # Position information
    if detections:
        detection = detections[0]  # Show info for first detection
        cv2.putText(frame, f"Position: {detection['position']}", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, get_position_color(detection['position'], detection['distance']), 2)
        y_offset += line_height
        
        cv2.putText(frame, f"Distance: {detection['distance']}", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, get_position_color(detection['position'], detection['distance']), 2)
        y_offset += line_height
        
        cv2.putText(frame, f"Movement: {detection['movement']}", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += line_height
        
        # Distance measurements
        real_distance = detection['estimated_real_distance_cm']
        pixel_distance = detection['pixel_distance_to_center']
        
        if real_distance is not None:
            cv2.putText(frame, f"Real Distance: {real_distance:.1f} cm", (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        else:
            cv2.putText(frame, f"Real Distance: N/A", (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        y_offset += line_height
        
        cv2.putText(frame, f"Pixel Distance: {pixel_distance:.0f} px", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        y_offset += line_height
        
        # Movement metrics
        movement_distance = detection['movement_distance_pixels']
        movement_speed = detection['movement_speed_pixels_per_sec']
        total_traveled = detection['total_distance_traveled']
        avg_speed = detection['average_speed']
        direction_deg = detection['movement_direction_degrees']
        
        cv2.putText(frame, f"Movement: {movement_distance:.1f} px", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        y_offset += line_height
        
        cv2.putText(frame, f"Speed: {movement_speed:.1f} px/s", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        y_offset += line_height
        
        cv2.putText(frame, f"Total Traveled: {total_traveled:.1f} px", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        y_offset += line_height
        
        cv2.putText(frame, f"Avg Speed: {avg_speed:.1f} px/s", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        y_offset += line_height
        
        cv2.putText(frame, f"Direction: {direction_deg:.0f}°", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        y_offset += line_height
        
        # Robot command
        robot_command = get_robot_command(detection)
        command_color = get_command_color(robot_command)
        cv2.putText(frame, f"Robot: {robot_command}", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, command_color, 2)
        y_offset += line_height
        
        # Safety status
        if detection['distance'] == "TOO CLOSE":
            cv2.putText(frame, "⚠️ SAFETY STOP ACTIVATED", (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    else:
        cv2.putText(frame, "No person detected", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        y_offset += line_height
        cv2.putText(frame, "Robot: SEARCHING", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

def draw_guidance_zones(frame):
    """Draw guidance zones and safety zones on the frame with real-world distance thresholds"""
    height, width = frame.shape[:2]
    
    # Draw center zone
    center_x = width // 2
    center_y = height // 2
    
    # Distance zones based on real-world distance thresholds:
    # Red circle: < 29 cm (TOO CLOSE)
    # Green circle: 29-31 cm (GOOD DISTANCE/IDLE)
    # Blue circle: > 31 cm (TOO FAR)
    
    # Note: We'll use approximate pixel circles for visualization
    # These will be updated based on the actual distance estimation
    
    # Too Far zone (blue circle - outside 31 cm equivalent)
    cv2.circle(frame, (center_x, center_y), 50, (255, 0, 0), 2)
    cv2.putText(frame, "TOO FAR (>31cm)", (center_x - 50, center_y - 60), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
    
    # Good Distance zone (green circle - 29-31 cm equivalent)
    cv2.circle(frame, (center_x, center_y), 30, (0, 255, 0), 2)
    cv2.putText(frame, "GOOD DISTANCE (29-31cm)", (center_x - 70, center_y - 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
    
    # Too Close zone (red circle - inside 29 cm equivalent)
    cv2.circle(frame, (center_x, center_y), 20, (0, 0, 255), 2)
    cv2.putText(frame, "TOO CLOSE (<29cm)", (center_x - 50, center_y + 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)
    
    # Left zone (blue)
    left_x = width // 4
    cv2.circle(frame, (left_x, center_y), 40, (255, 0, 0), 2)
    cv2.putText(frame, "LEFT", (left_x - 20, center_y - 50), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    
    # Right zone (magenta)
    right_x = 3 * width // 4
    cv2.circle(frame, (right_x, center_y), 40, (255, 0, 255), 2)
    cv2.putText(frame, "RIGHT", (right_x - 25, center_y - 50), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
    
    # Distance indicators with real-world values
    cv2.putText(frame, "TOO CLOSE (<29cm)", (10, height - 60), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, "GOOD DISTANCE (29-31cm)", (10, height - 40), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, "TOO FAR (>31cm)", (10, height - 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

def draw_distance_indicators(frame, tracker):
    """Draw distance indicators and movement trails"""
    if len(tracker.history) < 2:
        return
    
    # Draw distance line from center to current position
    if tracker.history:
        current_pos = tracker.history[-1]
        center_x = tracker.center_x
        center_y = tracker.center_y
        
        # Draw line from center to current position
        cv2.line(frame, (center_x, center_y), 
                (current_pos['center_x'], current_pos['center_y']), 
                (0, 255, 255), 2)
        
        # Draw distance circle around current position
        cv2.circle(frame, (current_pos['center_x'], current_pos['center_y']), 
                  5, (0, 255, 255), -1)
        
        # Draw distance text
        real_distance = current_pos['estimated_real_distance_cm']
        if real_distance is not None:
            distance_text = f"{real_distance:.1f}cm"
        else:
            distance_text = f"{current_pos['pixel_distance_to_center']:.0f}px"
        cv2.putText(frame, distance_text, 
                   (current_pos['center_x'] + 10, current_pos['center_y'] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    
    # Draw movement trail (last 10 positions)
    if len(tracker.history) >= 2:
        trail_points = list(tracker.history)[-10:]  # Last 10 positions
        
        for i in range(1, len(trail_points)):
            # Draw line between consecutive points
            pt1 = (trail_points[i-1]['center_x'], trail_points[i-1]['center_y'])
            pt2 = (trail_points[i]['center_x'], trail_points[i]['center_y'])
            
            # Color based on recency (more recent = brighter)
            alpha = i / len(trail_points)
            color = (int(255 * alpha), int(255 * alpha), 0)  # Green to yellow
            
            cv2.line(frame, pt1, pt2, color, 2)
            
            # Draw small circle at each point
            cv2.circle(frame, pt2, 3, color, -1)

def main():
    """Main function for live human detection with GUI"""
    print("Live Human Detection with YOLO 10 - Position Tracking")
    print("=" * 50)
    print("Advanced human detection with position and movement tracking")
    print("Loading YOLO 10 model...")
    
    # Initialize position tracker
    tracker = PositionTracker(history_length=15)
    
    try:
        # Try to load YOLO 10 model
        model = YOLO('yolov10n.pt')
        print("✅ YOLO 10 model loaded successfully!")
    except:
        try:
            # Fallback to YOLO 8 if YOLO 10 not available
            model = YOLO('yolov8n.pt')
            print("✅ YOLO 8 model loaded successfully!")
        except Exception as e:
            print(f"❌ Error loading model: {e}")
            print("Please ensure you have a YOLO model file")
            return
    
    print("Detection features:")
    print("- Position tracking (LEFT, CENTER, RIGHT)")
    print("- Real-world distance monitoring:")
    print("  * TOO CLOSE: < 29 cm")
    print("  * GOOD DISTANCE: 29-31 cm")
    print("  * TOO FAR: > 31 cm")
    print("- Real-world distance estimation (cm)")
    print("- Pixel distance measurement")
    print("- Movement detection (GOING LEFT, GOING RIGHT, IDLE)")
    print("- Movement speed tracking (pixels/second)")
    print("- Total distance traveled tracking")
    print("- Movement direction in degrees")
    print("- Movement trails visualization")
    print("- Real-time guidance zones")
    print("- Full screen display")
    print()
    print("Controls:")
    print("- Press 'q' to quit")
    print("- Press 'f' to toggle fullscreen")
    print("- Press 'r' to reset window size")
    print()
    
    # Initialize camera
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    # Set camera to high resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    # Get actual camera resolution
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera resolution: {actual_width}x{actual_height}")
    
    print("Starting live human detection with position tracking...")
    print("Move around to test different positions and movements!")
    
    frame_count = 0
    total_detections = 0
    start_time = time.time()
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("Failed to grab frame")
            break
        
        # Draw guidance zones
        draw_guidance_zones(frame)
        
        # Detect humans using YOLO with position tracking
        detections = detect_humans_yolo(frame, model, tracker)
        
        # Draw distance indicators and movement trails
        draw_distance_indicators(frame, tracker)
        
        # Draw status panel
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time if elapsed_time > 0 else 0
        draw_status_panel(frame, detections, frame_count, total_detections, fps)
        
        # Add instructions
        cv2.putText(frame, "Press 'q' to quit | 'f' to toggle fullscreen | 'r' to reset size", (10, frame.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Display the frame in full screen
        cv2.namedWindow('Live Human Detection - Position Tracking', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty('Live Human Detection - Position Tracking', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow('Live Human Detection - Position Tracking', frame)
        
        # Update statistics
        total_detections += len(detections)
        frame_count += 1
        
        # Print progress every 30 frames
        if frame_count % 30 == 0:
            if detections:
                detection = detections[0]
                robot_command = get_robot_command(detection)
                real_distance = detection['estimated_real_distance_cm']
                pixel_distance = detection['pixel_distance_to_center']
                speed = detection['movement_speed_pixels_per_sec']
                total_traveled = detection['total_distance_traveled']
                
                if real_distance is not None:
                    print(f"Frame {frame_count}: {detection['position']} | {detection['distance']} | {detection['movement']} | Distance: {real_distance:.1f}cm | Speed: {speed:.1f}px/s | Total: {total_traveled:.1f}px | Robot: {robot_command} | FPS: {fps:.1f}")
                else:
                    print(f"Frame {frame_count}: {detection['position']} | {detection['distance']} | {detection['movement']} | Pixel: {pixel_distance:.0f}px | Speed: {speed:.1f}px/s | Total: {total_traveled:.1f}px | Robot: {robot_command} | FPS: {fps:.1f}")
            else:
                print(f"Frame {frame_count}: No detection | Robot: SEARCHING | FPS: {fps:.1f}")
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('f'):  # Toggle fullscreen
            current_prop = cv2.getWindowProperty('Live Human Detection - Position Tracking', cv2.WND_PROP_FULLSCREEN)
            if current_prop == cv2.WINDOW_FULLSCREEN:
                cv2.setWindowProperty('Live Human Detection - Position Tracking', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
            else:
                cv2.setWindowProperty('Live Human Detection - Position Tracking', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        elif key == ord('r'):  # Reset window size
            cv2.setWindowProperty('Live Human Detection - Position Tracking', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Live Human Detection - Position Tracking', 1280, 720)
    
    cap.release()
    cv2.destroyAllWindows()
    
    # Print final statistics
    elapsed_time = time.time() - start_time
    print(f"\nLive Human Detection Complete!")
    print(f"Total frames processed: {frame_count}")
    print(f"Total detections: {total_detections}")
    print(f"Time elapsed: {elapsed_time:.1f} seconds")
    if frame_count > 0:
        print(f"Average detections per frame: {total_detections/frame_count:.2f}")
        print(f"Average FPS: {frame_count/elapsed_time:.1f}")

if __name__ == "__main__":
    main() 