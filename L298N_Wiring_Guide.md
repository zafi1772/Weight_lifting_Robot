# L298N Motor Driver Wiring Guide

## Components Required
- Arduino Uno/Nano/Mega
- L298N Motor Driver Module
- 2x DC Motors (for left and right wheels)
- 12V Power Supply (for motors)
- 5V Power Supply (for Arduino)
- Breadboard and jumper wires
- Optional: 100-220μF capacitor for power filtering

## Pin Connections

### Arduino to L298N Connections
| Arduino Pin | L298N Pin | Description |
|-------------|-----------|-------------|
| Pin 2       | IN1       | Left Motor Forward |
| Pin 3       | IN2       | Left Motor Backward |
| Pin 4       | IN3       | Right Motor Forward |
| Pin 5       | IN4       | Right Motor Backward |
| Pin 9       | ENA       | Left Motor Speed (PWM) |
| Pin 10      | ENB       | Right Motor Speed (PWM) |

### Power Connections
| Component | Connection | Description |
|-----------|------------|-------------|
| 12V Power Supply | L298N VCC | Motor power supply |
| 12V Power Supply | L298N GND | Motor ground |
| Arduino 5V | L298N +5V | Logic power (if not using USB) |
| Arduino GND | L298N GND | Common ground |

### Motor Connections
| Motor | L298N Pin | Description |
|-------|-----------|-------------|
| Left Motor + | OUT1 | Left motor positive |
| Left Motor - | OUT2 | Left motor negative |
| Right Motor + | OUT3 | Right motor positive |
| Right Motor - | OUT4 | Right motor negative |

## Wiring Diagram
```
Arduino          L298N Motor Driver
┌─────────┐      ┌─────────────────┐
│         │      │                 │
│ Pin 2   ├──────┤ IN1             │
│ Pin 3   ├──────┤ IN2             │
│ Pin 4   ├──────┤ IN3             │
│ Pin 5   ├──────┤ IN4             │
│ Pin 9   ├──────┤ ENA             │
│ Pin 10  ├──────┤ ENB             │
│         │      │                 │
│ 5V      ├──────┤ +5V             │
│ GND     ├──────┤ GND             │
└─────────┘      └─────────────────┘
                        │
                        │
                 ┌──────┴──────┐
                 │             │
                 │  12V Power  │
                 │   Supply    │
                 │             │
                 └──────┬──────┘
                        │
                 ┌──────┴──────┐
                 │             │
                 │   Motors    │
                 │             │
                 └─────────────┘
```

## Important Notes

### Power Supply Requirements
- **Motor Power**: 12V DC power supply (current rating depends on your motors)
- **Logic Power**: 5V (can be from Arduino USB or separate supply)
- **Current Rating**: Ensure your power supply can handle the combined current of both motors

### Safety Considerations
1. **Always connect grounds together** - Arduino GND and motor power supply GND must be connected
2. **Use appropriate wire gauge** - Thicker wires for motor power connections
3. **Add capacitors** - Place a 100-220μF capacitor between 12V and GND near the L298N for power filtering
4. **Heat sinking** - The L298N can get hot; consider adding a heat sink for prolonged use

### Motor Specifications
- **Voltage**: 12V DC motors recommended
- **Current**: Check motor specifications and ensure L298N can handle the current
- **Power**: Typical 12V motors draw 100-500mA each under load

## Testing the Setup

1. **Upload the Arduino code** (`motor_control.ino`) to your Arduino
2. **Open Serial Monitor** in Arduino IDE (set baud rate to 9600)
3. **Send test commands**:
   - `F` - Move forward
   - `B` - Move backward
   - `L` - Turn left
   - `R` - Turn right
   - `FL` - Forward left
   - `FR` - Forward right
   - `S` - Stop

## Troubleshooting

### Common Issues
1. **Motors not moving**: Check power supply connections and motor wiring
2. **Motors spinning wrong direction**: Swap motor wires or change IN1/IN2 or IN3/IN4 connections
3. **L298N getting hot**: Reduce motor speed or add heat sink
4. **Arduino resetting**: Separate motor power supply from Arduino power
5. **Noisy operation**: Add capacitors for power filtering

### Motor Direction Correction
If your motors are spinning in the wrong direction, you can either:
- Swap the motor wires (OUT1↔OUT2 or OUT3↔OUT4)
- Or modify the code to swap the HIGH/LOW values in the motor functions

## Integration with Python Code

To integrate this with your Python following robot code, you'll need to:
1. Add serial communication to send commands from Python to Arduino
2. Modify the `get_robot_command()` function in `main.py` to send appropriate motor commands
3. Use a library like `pyserial` to communicate with the Arduino

Example Python code for motor control:
```python
import serial
import time

# Initialize serial connection
arduino = serial.Serial('COM3', 9600, timeout=1)  # Change COM3 to your port
time.sleep(2)  # Wait for Arduino to reset

def send_motor_command(command):
    arduino.write((command + '\n').encode())
    time.sleep(0.1)  # Small delay for command processing
``` 