#include <Servo.h>

#define TRIG_PIN 9
#define ECHO_PIN 8
#define SERVO_PIN 11

// PID constants (can be tuned via Serial)
float Kp = 6;
float Ki = 0.01;
float Kd = 9;

float setpoint = 15.0;  // Desired distance (in cm)
float error = 0, previous_error = 0;
float integral = 0, derivative = 0;
float output = 0;

int neutralAngle = 34;  // Servo center position
unsigned long stableStartTime = 0;  // Time when error first became small
bool isStable = false;

Servo myServo;
String inputString = "";
bool newInput = false;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  myServo.attach(SERVO_PIN);
  myServo.write(neutralAngle);  // Start in neutral
  inputString.reserve(50);

  Serial.println("=== PID Ball Balancer Initialized ===");
  Serial.println("Commands:");
  Serial.println("  Kp <val>   → Set proportional gain");
  Serial.println("  Ki <val>   → Set integral gain");
  Serial.println("  Kd <val>   → Set derivative gain");
  Serial.println("  neutral <val> → Set neutral angle");
  Serial.println("  servo <±val>  → Move servo relative to neutral");
}

float getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2.0;
  return distance;
}

void loop() {
  handleSerialInput();
  float distance = getDistanceCM();

  if (distance > 45.0) {
    myServo.write(neutralAngle);
    Serial.println("Ball out of range, resetting to neutral.");
    delay(100);
    return;
  }
  error = distance - setpoint;

  // Check for sustained stability
  if (abs(error) < 3) {
    if (!isStable) {
      stableStartTime = millis();
      isStable = true;
    } else if (millis() - stableStartTime >= 2500) {
      myServo.write(neutralAngle);
      Serial.println("Error stable <3 for 2.5s → Servo set to neutral.");
      delay(50);
      return;
    }
  } else {
    isStable = false;  // Reset if error increases
  }

  // PID calculations
  integral += error;
  derivative = error - previous_error;
  output = -(Kp * error + Ki * integral + Kd * derivative);

  // Limit output and amplify
  output = constrain(output, -60, 60);
  int servoAngle = constrain(neutralAngle + output * 1.2, 0, 180);
  myServo.write(servoAngle);

  // Debug info
  Serial.print("Distance: ");
  Serial.print(distance, 2);
  Serial.print(" cm | Error: ");
  Serial.print(error, 2);
  Serial.print(" | Servo Angle: ");
  Serial.print(servoAngle);
  Serial.print(" | Kp: ");
  Serial.print(Kp, 3);
  Serial.print(" | Ki: ");
  Serial.print(Ki, 5);
  Serial.print(" | Kd: ");
  Serial.println(Kd, 3);

  previous_error = error;
  delay(50);  // ~20Hz control rate
}

void handleSerialInput() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      newInput = true;
    } else if (inChar != '\r') {
      inputString += inChar;
    }
  }

  if (newInput) {
    parseCommand(inputString);
    inputString = "";
    newInput = false;
  }
}

void parseCommand(String cmd) {
  cmd.trim();
  int spaceIndex = cmd.indexOf(' ');
  if (spaceIndex == -1) return;

  String param = cmd.substring(0, spaceIndex);
  float value = cmd.substring(spaceIndex + 1).toFloat();

  if (param.equalsIgnoreCase("Kp")) {
    Kp = value;
    Serial.print("Updated Kp: ");
    Serial.println(Kp);
  } else if (param.equalsIgnoreCase("Ki")) {
    Ki = value;
    Serial.print("Updated Ki: ");
    Serial.println(Ki);
    
  } else if (param.equalsIgnoreCase("Kd")) {
    Kd = value;
    Serial.print("Updated Kd: ");
    Serial.println(Kd);
    
  } else if (param.equalsIgnoreCase("neutral")) {
    neutralAngle = constrain((int)value, 0, 180);
    Serial.print("Updated neutral angle: ");
    Serial.println(neutralAngle);
    
  } else if (param.equalsIgnoreCase("servo")) {
    int target = constrain(neutralAngle + (int)value, 0, 180);
    myServo.write(target);
    Serial.print("Moved servo to angle: ");
    Serial.println(target);
    
  } else {
    Serial.println("Unknown command. Use Kp, Ki, Kd, neutral, or servo.");
  }
}