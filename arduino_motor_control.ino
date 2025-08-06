/*
 * L298N Motor Control for Following Robot
 * 
 * Pin Definitions:
 * #define ENA 3   // Enable motor A (PWM)
 * #define IN1 2   // Input 1 for motor A
 * #define IN2 4   // Input 2 for motor A
 * #define IN3 6   // Input 3 for motor B
 * #define IN4 7   // Input 4 for motor B
 * #define ENB 5   // Enable motor B (PWM)
 * 
 * Commands from Python:
 * F - Forward
 * B - Backward
 * L - Turn Left
 * R - Turn Right
 * S - Stop
 * G - Forward Left
 * I - Forward Right
 * H - Backward Left
 * J - Backward Right
 * 1-9 - Speed levels
 * q - Full speed
 */

#define ENA 3
#define IN1 2
#define IN2 4 
#define IN3 6 
#define IN4 7
#define ENB 5

int incomingByte = 0; // for incoming serial data
int speed_min = 125; //the minimum "speed" the motors will turn - take it lower and motors don't turn
int speed_max = 255; //

int speed_left = speed_max; // set both motors to maximum speed
int speed_right = speed_max;

void left();
void right();
void forward();
void backward();
void forward_left();
void forward_right();
void back_left();
void back_right();
void stopy();

void setup() {
  Serial.begin(9600);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Initialize motors to stop
  stopy();
}

void loop() {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
  }
  
  switch(incomingByte) {
    case 'S':
      stopy();
      incomingByte = '*';
      break;
      
    case 'B':
      backward();
      incomingByte = '*';
      break;
    
    case 'F':
      forward();
      incomingByte = '*';
      break;
     
    case 'R':
      right(); 
      incomingByte = '*';
      break;
      
    case 'L':
      left();     
      incomingByte = '*';
      break;
      
    case '1':
      speed_left = 20; 
      speed_right = 20;
      incomingByte = '*';
      break;
      
    case '2':
      speed_left = 40; 
      speed_right = 40;
      incomingByte = '*';
      break;
      
    case '3':
      speed_left = 60; 
      speed_right = 60;
      incomingByte = '*';
      break; 
      
    case '4':
      speed_left = 80; 
      speed_right = 80;
      incomingByte = '*';
      break; 
      
    case '5':
      speed_left = 100; 
      speed_right = 100;
      incomingByte = '*';
      break; 
      
    case '6':
      speed_left = 120; 
      speed_right = 120;
      incomingByte = '*';
      break; 
      
    case '7':
      speed_left = 140; 
      speed_right = 140;
      incomingByte = '*';
      break; 
      
    case '8':
      speed_left = 160; 
      speed_right = 160;
      incomingByte = '*';
      break; 
      
    case '9':
      speed_left = 200; 
      speed_right = 200;
      incomingByte = '*';
      break; 
      
    case 'q':
      speed_left = 255; 
      speed_right = 255;
      incomingByte = '*';
      break; 
      
    case 'J':
      back_right();
      incomingByte = '*';
      break;
      
    case 'H':
      back_left();
      incomingByte = '*';
      break;
      
    case 'I':
      forward_right();
      incomingByte = '*';
      break;
      
    case 'G':
      forward_left();
      incomingByte = '*';
      break;      
  }
}

void left() {
  // Turn left
  analogWrite(ENA, speed_left);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed_right);
}

void right() {
  // Turn right
  analogWrite(ENA, speed_left);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed_right);
}

void forward_left() {
  // Forward left
  analogWrite(ENA, speed_left);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed_right);
}

void forward_right() {
  // Forward right
  analogWrite(ENA, speed_left);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed_right);
}

void back_right() {
  // Backward right
  analogWrite(ENA, speed_left);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed_right);
}

void back_left() {
  // Backward left
  analogWrite(ENA, speed_left);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed_right);
}

void forward() {
  // Forward
  analogWrite(ENA, speed_left);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed_right);
}

void backward() {
  // Backward
  analogWrite(ENA, speed_left);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed_right);
}

void stopy() {
  // Stop
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
} 