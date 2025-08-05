/*
 * Motor Control for Following Robot using L298N Motor Driver
 * Controls a 4-wheel car-like robot with differential steering
 */

// Motor driver pin definitions
// Left motor
const int LEFT_MOTOR_IN1 = 2;  // Left motor forward
const int LEFT_MOTOR_IN2 = 3;  // Left motor backward
const int LEFT_MOTOR_ENA = 9;  // Left motor speed control (PWM)

// Right motor
const int RIGHT_MOTOR_IN3 = 4; // Right motor forward
const int RIGHT_MOTOR_IN4 = 5; // Right motor backward
const int RIGHT_MOTOR_ENB = 10; // Right motor speed control (PWM)

// Motor speed settings
const int MOTOR_SPEED_FULL = 255;    // Full speed
const int MOTOR_SPEED_HIGH = 200;    // High speed
const int MOTOR_SPEED_MEDIUM = 150;  // Medium speed
const int MOTOR_SPEED_LOW = 100;     // Low speed
const int MOTOR_SPEED_TURN = 120;    // Speed for turning

// Serial communication
const int BAUD_RATE = 115200;
String incomingCommand = "";

// PWM speed control
int current_left_speed = 0;
int current_right_speed = 0;

void setup() {
  // Initialize motor control pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_ENA, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);
  pinMode(RIGHT_MOTOR_ENB, OUTPUT);
  
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  Serial.println("Motor Control System Ready");
  Serial.println("Available commands:");
  Serial.println("F - Forward");
  Serial.println("B - Backward");
  Serial.println("L - Left");
  Serial.println("R - Right");
  Serial.println("FL - Forward Left");
  Serial.println("FR - Forward Right");
  Serial.println("S - Stop");
  
  // Stop motors initially
  stopMotors();
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    
    if (incomingChar == '\n') {
      // Process complete command
      processCommand(incomingCommand);
      incomingCommand = ""; // Reset command string
    } else {
      // Add character to command string
      incomingCommand += incomingChar;
    }
  }
}

void processCommand(String command) {
  command.toUpperCase(); // Convert to uppercase for case-insensitive comparison
  
  Serial.print("Executing command: ");
  Serial.println(command);
  
  // Check for PWM speed commands first
  if (command.startsWith("PWM:")) {
    processPWMSpeedCommand(command);
    return;
  }
  
  if (command == "F" || command == "FORWARD") {
    moveForward();
  } else if (command == "B" || command == "BACKWARD") {
    moveBackward();
  } else if (command == "L" || command == "LEFT") {
    turnLeft();
  } else if (command == "R" || command == "RIGHT") {
    turnRight();
  } else if (command == "FL" || command == "FORWARD_LEFT") {
    moveForwardLeft();
  } else if (command == "FR" || command == "FORWARD_RIGHT") {
    moveForwardRight();
  } else if (command == "S" || command == "STOP") {
    stopMotors();
  } else {
    Serial.println("Unknown command: " + command);
  }
}

void processPWMSpeedCommand(String command) {
  // Format: PWM:LEFT_SPEED,RIGHT_SPEED,DIRECTION
  // Example: PWM:200,200,F (forward with 200 speed)
  // Example: PWM:150,150,B (backward with 150 speed)
  // Example: PWM:0,0,S (stop)
  
  command = command.substring(4); // Remove "PWM:" prefix
  int comma1 = command.indexOf(',');
  int comma2 = command.indexOf(',', comma1 + 1);
  
  if (comma1 == -1 || comma2 == -1) {
    Serial.println("Invalid PWM command format. Use: PWM:LEFT,RIGHT,DIRECTION");
    return;
  }
  
  int leftSpeed = command.substring(0, comma1).toInt();
  int rightSpeed = command.substring(comma1 + 1, comma2).toInt();
  String direction = command.substring(comma2 + 1);
  
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  current_left_speed = leftSpeed;
  current_right_speed = rightSpeed;
  
  Serial.print("PWM Speed - Left: ");
  Serial.print(leftSpeed);
  Serial.print(", Right: ");
  Serial.print(rightSpeed);
  Serial.print(", Direction: ");
  Serial.println(direction);
  
  // Apply direction
  if (direction == "F" || direction == "FORWARD") {
    setMotorDirection(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, HIGH, LOW);
    setMotorDirection(RIGHT_MOTOR_IN3, RIGHT_MOTOR_IN4, HIGH, LOW);
  } else if (direction == "B" || direction == "BACKWARD") {
    setMotorDirection(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LOW, HIGH);
    setMotorDirection(RIGHT_MOTOR_IN3, RIGHT_MOTOR_IN4, LOW, HIGH);
  } else if (direction == "L" || direction == "LEFT") {
    setMotorDirection(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LOW, HIGH);
    setMotorDirection(RIGHT_MOTOR_IN3, RIGHT_MOTOR_IN4, HIGH, LOW);
  } else if (direction == "R" || direction == "RIGHT") {
    setMotorDirection(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, HIGH, LOW);
    setMotorDirection(RIGHT_MOTOR_IN3, RIGHT_MOTOR_IN4, LOW, HIGH);
  } else if (direction == "S" || direction == "STOP") {
    setMotorDirection(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LOW, LOW);
    setMotorDirection(RIGHT_MOTOR_IN3, RIGHT_MOTOR_IN4, LOW, LOW);
  }
  
  // Apply speeds
  analogWrite(LEFT_MOTOR_ENA, leftSpeed);
  analogWrite(RIGHT_MOTOR_ENB, rightSpeed);
}

void setMotorDirection(int pin1, int pin2, int state1, int state2) {
  digitalWrite(pin1, state1);
  digitalWrite(pin2, state2);
}

// ===== MOTOR CONTROL FUNCTIONS =====

void moveForward() {
  Serial.println("Moving FORWARD");
  
  // Left motor forward
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_ENA, MOTOR_SPEED_HIGH);
  
  // Right motor forward
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  analogWrite(RIGHT_MOTOR_ENB, MOTOR_SPEED_HIGH);
}

void moveBackward() {
  Serial.println("Moving BACKWARD");
  
  // Left motor backward
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  analogWrite(LEFT_MOTOR_ENA, MOTOR_SPEED_HIGH);
  
  // Right motor backward
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);
  analogWrite(RIGHT_MOTOR_ENB, MOTOR_SPEED_HIGH);
}

void turnLeft() {
  Serial.println("Turning LEFT");
  
  // Left motor backward (or stop for tighter turn)
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  analogWrite(LEFT_MOTOR_ENA, MOTOR_SPEED_TURN);
  
  // Right motor forward
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  analogWrite(RIGHT_MOTOR_ENB, MOTOR_SPEED_TURN);
}

void turnRight() {
  Serial.println("Turning RIGHT");
  
  // Left motor forward
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_ENA, MOTOR_SPEED_TURN);
  
  // Right motor backward (or stop for tighter turn)
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);
  analogWrite(RIGHT_MOTOR_ENB, MOTOR_SPEED_TURN);
}

void moveForwardLeft() {
  Serial.println("Moving FORWARD LEFT");
  
  // Left motor forward at reduced speed
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_ENA, MOTOR_SPEED_LOW);
  
  // Right motor forward at full speed
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  analogWrite(RIGHT_MOTOR_ENB, MOTOR_SPEED_HIGH);
}

void moveForwardRight() {
  Serial.println("Moving FORWARD RIGHT");
  
  // Left motor forward at full speed
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_ENA, MOTOR_SPEED_HIGH);
  
  // Right motor forward at reduced speed
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  analogWrite(RIGHT_MOTOR_ENB, MOTOR_SPEED_LOW);
}

void stopMotors() {
  Serial.println("STOPPING motors");
  
  // Stop left motor
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_ENA, 0);
  
  // Stop right motor
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  analogWrite(RIGHT_MOTOR_ENB, 0);
}

// ===== UTILITY FUNCTIONS =====

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Ensure speed values are within valid range (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  analogWrite(LEFT_MOTOR_ENA, leftSpeed);
  analogWrite(RIGHT_MOTOR_ENB, rightSpeed);
}

void emergencyStop() {
  Serial.println("EMERGENCY STOP!");
  stopMotors();
}

// ===== TESTING FUNCTIONS =====

void testMotors() {
  Serial.println("Starting motor test sequence...");
  
  // Test forward
  moveForward();
  delay(2000);
  
  // Test backward
  moveBackward();
  delay(2000);
  
  // Test left turn
  turnLeft();
  delay(2000);
  
  // Test right turn
  turnRight();
  delay(2000);
  
  // Test forward left
  moveForwardLeft();
  delay(2000);
  
  // Test forward right
  moveForwardRight();
  delay(2000);
  
  // Stop
  stopMotors();
  Serial.println("Motor test complete!");
}

// ===== SERIAL COMMAND INTERFACE =====

void printStatus() {
  Serial.println("=== Motor Control Status ===");
  Serial.println("System: Ready");
  Serial.println("Available commands: F, B, L, R, FL, FR, S");
  Serial.println("============================");
} 