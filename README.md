# 🤖 Following Robot

A real-time human detection and following robot system using YOLO and OpenCV.

## 🚀 Features

- **Real-time Human Detection**: Uses YOLO (YOLOv8/YOLOv10) for accurate person detection
- **Smart Following**: Robot follows person based on position and distance
- **Percentage-based Positioning**: Shows detailed left/right position with percentages
- **Distance-based Speed Control**: Adjusts speed based on distance to person
- **Arduino Motor Control**: Serial communication with L298N motor driver
- **Elevator Music**: Plays mild background music when person is idle (not moving)

## 📋 Requirements

- Python 3.8+ (tested with Python 3.13)
- Webcam
- Arduino with L298N motor driver
- YOLO model file (yolov8n.pt or yolov10n.pt)

## 🛠️ Installation

1. **Clone or download the project files**

2. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Download YOLO model** (if not already present):
   ```bash
   # The system will automatically download yolov8n.pt if not found
   ```

4. **Upload Arduino code**:
   - Open `arduino_motor_control.ino` in Arduino IDE
   - Upload to your Arduino board

## 🔧 Hardware Setup

### Arduino Pin Connections (L298N):
- **ENA** → Pin 3 (PWM)
- **IN1** → Pin 2
- **IN2** → Pin 4
- **IN3** → Pin 6
- **IN4** → Pin 7
- **ENB** → Pin 5 (PWM)

### Motor Connections:
- **Motor A**: Left motor
- **Motor B**: Right motor

## 🎮 Usage

### Basic Usage:
```bash
python main.py
```

### Advanced Usage:
```bash
python main.py --source 0 --model yolov8n.pt --conf 0.4 --serial_port COM3
```

### Command Line Options:
- `--source`: Camera index or video file (default: 0)
- `--model`: YOLO model path (default: yolov8n.pt)
- `--conf`: Confidence threshold (default: 0.4)
- `--close_cm`: Close distance threshold in cm (default: 340)
- `--far_cm`: Far distance threshold in cm (default: 455)
- `--serial_port`: Arduino serial port (default: COM3)
- `--baud_rate`: Serial baud rate (default: 9600)



## 🎵 Elevator Music

### Music Control:
- **Person Idle**: Plays mild elevator music from "Elevator Music - aeiouFU.mp3"
- **Person Moving**: Stops music automatically
- **No Person**: Stops music automatically
- **Volume**: Mild (30% volume) for background ambiance

### Music Features:
- **Real Elevator Music**: Uses actual elevator music file for authentic sound
- **Continuous Loop**: Music loops seamlessly while person is idle
- **Non-intrusive**: Moderate volume for pleasant background ambiance
- **Automatic Control**: Starts/stops based on person's movement state
- **Thread-safe**: Plays in background without blocking detection

## 🎯 Motor Control Logic

### Speed Control:
- **Idle Distance**: Speed 6
- **Far Distance**: Speed 8
- **Close Distance**: Stop (Safety)

### Movement Commands:
- **Center + Far**: Forward (F)
- **Center + Idle**: Forward (F)
- **Left positions**: Forward-Right (I)
- **Right positions**: Forward-Left (G)
- **Close distance**: Stop (S)

## 📊 Visual Feedback

The system displays:
- Person bounding box with position info
- Distance measurement in cm
- Movement state (Moving/Idle)
- Motor command being sent
- Following status (ON/OFF)
- Music status (ON/OFF when person is idle)

## 🧪 Testing

### Test Camera:
```bash
python test_camera.py
```

### Test Elevator Music:
```bash
python test_elevator_music.py
```

## 🔧 Troubleshooting

### Common Issues:

1. **Camera not working**:
   - Check camera permissions
   - Try different camera index: `--source 1`

2. **Serial connection failed**:
   - Check Arduino is connected
   - Verify correct COM port
   - Ensure Arduino code is uploaded

3. **Finger detection not accurate**:
   - Improve lighting conditions
   - Keep hand steady
   - Ensure hand is clearly visible

4. **Robot not following properly**:
   - Check motor connections
   - Verify Arduino code
   - Adjust distance thresholds

### Performance Tips:
- Use GPU if available: `--device cuda`
- Lower confidence for more detections: `--conf 0.3`
- Adjust distance thresholds for your environment

## 📁 File Structure

```
following robot/
├── main.py                    # Main application
├── arduino_motor_control.ino  # Arduino motor control code
├── requirements.txt           # Python dependencies
├── test_elevator_music.py     # Elevator music test
├── Elevator Music - aeiouFU.mp3  # Elevator music file
├── README.md                  # This file
├── yolov8n.pt                # YOLO model (auto-downloaded)
└── human detection dataset/   # Training dataset (if available)
```

## 🤝 Contributing

Feel free to submit issues and enhancement requests!

## 📄 License

This project is open source and available under the MIT License.

## 🙏 Acknowledgments

- YOLO by Ultralytics
- OpenCV community
- Arduino community 