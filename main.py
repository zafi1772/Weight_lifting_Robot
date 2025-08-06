"""
Real-time human detection from a live camera feed using a pretrained YOLO model (supports YOLOv8 or the newer YOLOv10).

How it works
------------
1. Loads a pretrained YOLO model (default: yolov10n). You can pass the path
   to a custom model checkpoint if you have fine-tuned weights.
2. Captures frames from a webcam (default index 0).
3. Runs object detection on each frame, filtering for the "person" class.
4. Draws bounding-boxes and confidence scores around detected people.
5. Sends motor control commands via serial to Arduino with L298N.
6. Press the **q** key to quit.

Prerequisites
-------------
Install dependencies with:
    pip install -r requirements.txt

Example
-------
    python human_detection_live.py --source 0 --model yolov10n.pt --conf 0.4

Options
-------
--source   Webcam index or video file/stream URL.
--model    Path to a YOLO checkpoint (default: yolov10n.pt).
--conf     Confidence threshold for displaying detections.
--close_cm      Distance < this value → "Close" (default 340)
--far_cm        Distance > this value → "Far"   (default 455)
--center_margin Fraction of frame width that's treated as center band (default 0.15)
--idle_thresh   Pixel displacement below which a person is considered idle (default 20)
--device        Device to use: "cpu", "cuda", or a CUDA device index (e.g. 0).
--serial_port   Serial port for Arduino communication (default: COM3)
--baud_rate     Baud rate for serial communication (default: 9600)
"""
import argparse
from pathlib import Path
import serial
import time
import pygame
import threading

import cv2
import math
import numpy as np
from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Real-time human detection with YOLO")
    parser.add_argument("--source", type=str, default="0", help="Webcam index or video file/stream URL")
    parser.add_argument("--model", type=str, default="yolov10n.pt", help="YOLO model checkpoint (v8/v10)")
    parser.add_argument("--conf", type=float, default=0.4, help="Confidence threshold")
    parser.add_argument("--close_cm", type=float, default=340.0, help="Distance below this (cm) considered 'Close'")
    parser.add_argument("--far_cm", type=float, default=455.0, help="Distance above this (cm) considered 'Far'")
    parser.add_argument("--idle_thresh", type=float, default=20.0, help="Pixel movement below this is considered idle")
    parser.add_argument("--avg_height_cm", type=float, default=170.0, help="Assumed real human height in cm")
    parser.add_argument("--focal_px", type=float, default=900.0, help="Camera focal length in pixels (calibrate for accuracy)")
    parser.add_argument("--center_margin", type=float, default=0.15, help="Fraction of frame width considered center band")
    parser.add_argument("--serial_port", type=str, default="COM3", help="Serial port for Arduino (e.g., COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud_rate", type=int, default=9600, help="Baud rate for serial communication")
    parser.add_argument(
        "--device",
        type=str,
        default="cuda" if cv2.cuda.getCudaEnabledDeviceCount() > 0 else "cpu",
        help='Device to run the model on ("cpu", "cuda", or CUDA device index)',
    )
    return parser.parse_args()


def open_capture(source: str) -> cv2.VideoCapture:
    """Create a cv2.VideoCapture from an integer index or string path/URL."""
    if source.isdigit():
        cap = cv2.VideoCapture(int(source))
    else:
        cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open video source {source}")
    return cap


def open_serial_connection(port: str, baud_rate: int) -> serial.Serial:
    """Open serial connection to Arduino."""
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset
        print(f"Serial connection established on {port}")
        return ser
    except serial.SerialException as e:
        print(f"Failed to open serial port {port}: {e}")
        print("Continuing without motor control...")
        return None


def send_motor_command(ser: serial.Serial, command: str):
    """Send motor command to Arduino."""
    if ser and ser.is_open:
        try:
            # Handle combined speed and movement commands
            if len(command) == 2 and command[0].isdigit():
                # Send speed first, then movement
                speed_cmd = command[0]
                movement_cmd = command[1]
                ser.write(speed_cmd.encode())
                time.sleep(0.1)  # Small delay between commands
                ser.write(movement_cmd.encode())
                print(f"Sent commands: Speed {speed_cmd}, Movement {movement_cmd}")
            else:
                # Single command (like 'S' for stop)
                ser.write(command.encode())
                print(f"Sent command: {command}")
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")






def play_elevator_music():
    """Play mild elevator music using pygame."""
    try:
        # Initialize pygame mixer
        pygame.mixer.init()
        
        # Load and play the elevator music file
        music_file = "Elevator Music - aeiouFU.mp3"
        
        # Load the music file
        pygame.mixer.music.load(music_file)
        
        # Set volume to mild level (0.3 = 30% volume)
        pygame.mixer.music.set_volume(0.3)
        
        # Play the music (loops continuously)
        pygame.mixer.music.play(-1)  # -1 means loop indefinitely
        
    except Exception as e:
        print(f"Elevator music error: {e}")


def stop_elevator_music():
    """Stop elevator music."""
    try:
        pygame.mixer.music.stop()
    except:
        pass


def determine_motor_command(horiz_state: str, dist_state: str, motion_state: str) -> str:
    """Determine motor command based on person's position and state."""
    
    # Position-based movement with speed control
    if "Center" in horiz_state:
        if dist_state == "Far":
            return "8F"  # Speed 8 + Forward if centered and far
        elif dist_state == "Close":
            return "S"  # Stop if centered and close
        else:
            return "6F"  # Speed 6 + Forward if centered and idle
    
    elif "Slightly Left" in horiz_state:
        if dist_state == "Far":
            return "8I"  # Speed 8 + Forward-right to center
        elif dist_state == "Close":
            return "S"  # Stop if close
        else:
            return "6I"  # Speed 6 + Forward-right to center
    
    elif "Slightly Right" in horiz_state:
        if dist_state == "Far":
            return "8G"  # Speed 8 + Forward-left to center
        elif dist_state == "Close":
            return "S"  # Stop if close
        else:
            return "6G"  # Speed 6 + Forward-left to center
    
    elif "Left" in horiz_state or "Very Left" in horiz_state:
        if dist_state == "Far":
            return "8I"  # Speed 8 + Forward-right
        elif dist_state == "Close":
            return "S"  # Stop if close
        else:
            return "6I"  # Speed 6 + Forward-right
    
    elif "Right" in horiz_state or "Very Right" in horiz_state:
        if dist_state == "Far":
            return "8G"  # Speed 8 + Forward-left
        elif dist_state == "Close":
            return "S"  # Stop if close
        else:
            return "6G"  # Speed 6 + Forward-left
    
    elif "Full Left" in horiz_state:
        if dist_state == "Far":
            return "8I"  # Speed 8 + Forward-right (sharp)
        elif dist_state == "Close":
            return "S"  # Stop if close
        else:
            return "6I"  # Speed 6 + Forward-right (sharp)
    
    elif "Full Right" in horiz_state:
        if dist_state == "Far":
            return "8G"  # Speed 8 + Forward-left (sharp)
        elif dist_state == "Close":
            return "S"  # Stop if close
        else:
            return "6G"  # Speed 6 + Forward-left (sharp)
    
    return "S"  # Default to stop


def main() -> None:
    args = parse_args()

    # Load model (YOLOv8/YOLOv10 or other supported weights)
    model_path = Path(args.model).expanduser().resolve()
    model_version = "YOLOv12" if "yolov12" in model_path.name.lower() else "YOLO"
    print(f"Loading {model_version} model from {model_path} …")
    model = YOLO(str(model_path))

    # Set device; ultralytics model has .to() but easiest via predict kwargs later
    device = args.device

    # Open video source
    cap = open_capture(args.source)
    
    # Open serial connection
    ser = open_serial_connection(args.serial_port, args.baud_rate)

    prev_center = None  # track center of the first detected person to decide idle vs moving
    last_command = None
    command_cooldown = 0
    
    # Music control variables
    music_playing = False
    last_motion_state = None
    


    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Frame capture failed — exiting")
                break

            # Inference
            results = model.predict(frame, conf=args.conf, device=device, verbose=False)
            r = results[0]

            person_detected = False
            motor_command = "S"  # Default to stop

            # Draw bounding boxes for class "person" (class id 0 in COCO)
            for box, cls_id, score in zip(
                r.boxes.xyxy.cpu().numpy(),
                r.boxes.cls.cpu().numpy(),
                r.boxes.conf.cpu().numpy(),
            ):
                if int(cls_id) != 0:
                    continue  # not a person

                person_detected = True
                x1, y1, x2, y2 = box.astype(int)
                height_px = y2 - y1

                # Center point of bbox
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                # Estimate distance in cm: D = (H_real * f) / h_px
                if height_px > 0:
                    distance_cm = (args.avg_height_cm * args.focal_px) / height_px
                else:
                    distance_cm = float('inf')

                # Distance category
                if distance_cm < args.close_cm:
                    dist_state = "Close"
                elif distance_cm > args.far_cm:
                    dist_state = "Far"
                else:
                    dist_state = "Idle"  # within band
                


                # Horizontal position with percentage-based indicators
                frame_h, frame_w = frame.shape[:2]
                margin = args.center_margin * frame_w
                
                # Calculate percentage from center (0% = center, 100% = edge)
                center_x = frame_w / 2
                max_offset = frame_w / 2  # Distance from center to edge
                offset_from_center = abs(cx - center_x)
                percentage_from_center = min(100, (offset_from_center / max_offset) * 100)
                
                # Determine position and intensity
                if cx < center_x - margin:
                    # Left side
                    if percentage_from_center < 33:
                        horiz_state = "Slightly Left"
                    elif percentage_from_center < 60:
                        horiz_state = "Left"
                    elif percentage_from_center < 85:
                        horiz_state = "Very Left"
                    else:
                        horiz_state = "Full Left"
                elif cx > center_x + margin:
                    # Right side
                    if percentage_from_center < 33:
                        horiz_state = "Slightly Right"
                    elif percentage_from_center < 60:
                        horiz_state = "Right"
                    elif percentage_from_center < 85:
                        horiz_state = "Very Right"
                    else:
                        horiz_state = "Full Right"
                else:
                    # Center
                    horiz_state = "Center"

                # Movement (idle) check for the first detected person only
                motion_state = "Moving"
                if prev_center is not None:
                    dx = cx - prev_center[0]
                    dy = cy - prev_center[1]
                    dist_moved = math.hypot(dx, dy)
                    if dist_moved < args.idle_thresh:
                        motion_state = "Idle"
                prev_center = (cx, cy)
                
                # Elevator music control
                if motion_state == "Idle" and not music_playing:
                    # Start elevator music when person becomes idle
                    music_thread = threading.Thread(target=play_elevator_music)
                    music_thread.daemon = True
                    music_thread.start()
                    music_playing = True
                    print("🎵 Elevator music started (person idle)")
                elif motion_state == "Moving" and music_playing:
                    # Stop elevator music when person starts moving
                    stop_elevator_music()
                    music_playing = False
                    print("🔇 Elevator music stopped (person moving)")
                
                last_motion_state = motion_state

                # Determine motor command automatically when person is detected
                motor_command = determine_motor_command(horiz_state, dist_state, motion_state)

                label = f"{horiz_state} ({percentage_from_center:.0f}%) | {dist_state} {distance_cm:.0f}cm | {motion_state} | CMD:{motor_command}"

                # Draw rectangle and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (x1, y1 - h - 10), (x1 + w, y1), (0, 255, 0), -1)
                cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

            # Send motor command if changed or if no person detected
            if not person_detected:
                motor_command = "S"  # Stop if no person detected
                # Stop elevator music when no person detected
                if music_playing:
                    stop_elevator_music()
                    music_playing = False
                    print("🔇 Elevator music stopped (no person detected)")
            

            
            if motor_command != last_command or command_cooldown <= 0:
                send_motor_command(ser, motor_command)
                last_command = motor_command
                command_cooldown = 10  # Send command every 10 frames

            command_cooldown -= 1

            # Draw zone markers
            frame_height, frame_width = frame.shape[:2]
            center_start = int(frame_width * (0.5 - args.center_margin))
            center_end = int(frame_width * (0.5 + args.center_margin))
            
            # Draw vertical lines for zone boundaries
            cv2.line(frame, (center_start, 0), (center_start, frame_height), (255, 255, 0), 2)  # Yellow
            cv2.line(frame, (center_end, 0), (center_end, frame_height), (255, 255, 0), 2)     # Yellow
            
            # Add zone labels with percentage ranges
            cv2.putText(frame, "LEFT (33-100%)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, "CENTER (0-33%)", (center_start + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, "RIGHT (33-100%)", (center_end + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Add distance zone info
            cv2.putText(frame, f"Close: <{args.close_cm}cm", (10, frame_height - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, f"Idle: {args.close_cm}-{args.far_cm}cm", (10, frame_height - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Far: >{args.far_cm}cm", (10, frame_height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Add position percentage legend
            cv2.putText(frame, "Position: Slightly(0-33%) | Normal(33-60%) | Very(60-85%) | Full(85-100%)", (10, frame_height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Add motor command display
            cv2.putText(frame, f"Motor: {motor_command}", (frame_width - 150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # Add following status
            cv2.putText(frame, f"Following: {'ON' if person_detected else 'OFF'}", (10, frame_height - 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Add music status
            music_status = "🎵 Music: ON" if music_playing else "🔇 Music: OFF"
            cv2.putText(frame, music_status, (10, frame_height - 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Display the frame
            cv2.imshow("Human Detection (press 'q' to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        # Stop motors before exiting
        if ser:
            send_motor_command(ser, "S")
            ser.close()
        
        # Stop elevator music before exiting
        stop_elevator_music()
        
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
