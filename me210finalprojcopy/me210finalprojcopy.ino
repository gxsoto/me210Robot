//PINS ASSUMING AN ARDUINO MEGA: https://docs.arduino.cc/resources/pinouts/A000067-full-pinout.pdf 

// l298n motor driver "in#" to digital pins
uint8_t motor1in1 = 22; // left motor
uint8_t motor1in2 = 24;
uint8_t motor2in3 = 28; // right motor
uint8_t motor2in4 = 26;

// l298n motor driver "enable" to digital pwm pins
uint8_t pwmMotor1 = 7;
uint8_t pwmMotor2 = 8;

// both ultrasonic sensors
uint8_t backSensorTrig = 52;
uint8_t backSensorEcho = 50;

uint8_t rightSensorTrig = 53;
uint8_t rightSensorEcho = 51;

// all 4 ir sensors, one analog one digital per sensor
// facing the bot: ir1 = front right, ir2 = front center-right, ir3 = front center-left, ir4 = front left
uint8_t ir1digital = 48;
uint8_t ir1analog = A0;

uint8_t ir2digital = 56;
uint8_t ir2analog = A3;

uint8_t ir3digital = 58;
uint8_t ir3analog = A5;

uint8_t ir4digital = 60;
uint8_t ir4analog = A7;

// thresholds - tune these during testing
float rightUSThresh = 8.5;
float backUSThresh = 35.0;
uint8_t irLineThresh = 50; // if IR reading is above this, line is detected

uint8_t slowMotorSpeed = 120; // for ALIGN_TO_LINE, tune during testing

typedef enum {
  ORIENTING, MOVE_TO_LINE, ALIGN_TO_LINE, LINE_FOLLOWING,
  RELEASE_PUCK, BACKUP_TO_WALL, ORIENT_TO_ENTER_BOX, ENTER_BOX
} States_t;

States_t currState;

// ---- motor helpers ----

void motorForward(uint8_t pin1, uint8_t pin2, uint8_t enable) {
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  analogWrite(enable, 255);
}

void motorBackward(uint8_t pin1, uint8_t pin2, uint8_t enable) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  analogWrite(enable, 255);
}

void motorStop(uint8_t pin1, uint8_t pin2, uint8_t enable) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  analogWrite(enable, 0);
}

void moveForward() {
  motorForward(motor1in1, motor1in2, pwmMotor1);
  motorForward(motor2in3, motor2in4, pwmMotor2);
}

void moveBackward() {
  motorBackward(motor1in1, motor1in2, pwmMotor1);
  motorBackward(motor2in3, motor2in4, pwmMotor2);
}

void stopMotors() {
  motorStop(motor1in1, motor1in2, pwmMotor1);
  motorStop(motor2in3, motor2in4, pwmMotor2);
}

void turnLeft() {
  // left motor backward, right motor forward
  motorBackward(motor1in1, motor1in2, pwmMotor1);
  motorForward(motor2in3, motor2in4, pwmMotor2);
}

void turnRight() {
  // left motor forward, right motor backward
  motorForward(motor1in1, motor1in2, pwmMotor1);
  motorBackward(motor2in3, motor2in4, pwmMotor2);
}

// gentle left curve: right motor full, left motor slow
void slowTurnLeft() {
  analogWrite(pwmMotor1, slowMotorSpeed);
  digitalWrite(motor1in1, HIGH);
  digitalWrite(motor1in2, LOW);

  analogWrite(pwmMotor2, 255);
  digitalWrite(motor2in3, HIGH);
  digitalWrite(motor2in4, LOW);
}

// ---- sensor helpers ----

float readUltrasonicSensor(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 38000);
  return duration * 0.0343 / 2;
}

uint8_t readIRSensor(uint8_t analog, uint8_t digital) {
  digitalWrite(digital, HIGH);
  delayMicroseconds(500);
  int withLight = analogRead(analog);

  digitalWrite(digital, LOW);
  delayMicroseconds(500);
  int noLight = analogRead(analog);

  return max(0, withLight - noLight);
}

// reads all 4 IR sensors into an array: [ir1, ir2, ir3, ir4]
void readAllIR(uint8_t results[4]) {
  results[0] = readIRSensor(ir1analog, ir1digital); // front right
  results[1] = readIRSensor(ir2analog, ir2digital); // front center-right
  results[2] = readIRSensor(ir3analog, ir3digital); // front center-left
  results[3] = readIRSensor(ir4analog, ir4digital); // front left
}

// ---- state handlers ----

void handleOrienting() {
  float rightDist = readUltrasonicSensor(rightSensorTrig, rightSensorEcho);
  float backDist  = readUltrasonicSensor(backSensorTrig, backSensorEcho);

  Serial.print("ORIENTING | right: "); Serial.print(rightDist);
  Serial.print(" back: "); Serial.println(backDist);

  if (rightDist <= rightUSThresh && backDist <= backUSThresh) {
    stopMotors();
    Serial.println("-> MOVE_TO_LINE");
    currState = MOVE_TO_LINE;
  } else {
    turnLeft();
  }
}

void handleMoveToLine() {
  uint8_t ir[4];
  readAllIR(ir);

  Serial.print("MOVE_TO_LINE | ir: ");
  for (int i = 0; i < 4; i++) { Serial.print(ir[i]); Serial.print(" "); }
  Serial.println();

  // any IR sensor detects line -> go to ALIGN_TO_LINE
  if (ir[0] > irLineThresh || ir[1] > irLineThresh || ir[2] > irLineThresh || ir[3] > irLineThresh) {
    stopMotors();
    Serial.println("-> ALIGN_TO_LINE");
    currState = ALIGN_TO_LINE;
  } else {
    moveForward();
  }
}

void handleAlignToLine() {
  uint8_t ir[4];
  readAllIR(ir);

  Serial.print("ALIGN_TO_LINE | ir: ");
  for (int i = 0; i < 4; i++) { Serial.print(ir[i]); Serial.print(" "); }
  Serial.println();

  // aligned when all 4 sensors detect the line
  if (ir[0] > irLineThresh && ir[1] > irLineThresh && ir[2] > irLineThresh && ir[3] > irLineThresh) {
    stopMotors();
    Serial.println("-> LINE_FOLLOWING");
    currState = LINE_FOLLOWING;
  } else {
    slowTurnLeft();
  }
}

// ---- setup ----

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(motor1in1, OUTPUT);
  pinMode(motor1in2, OUTPUT);
  pinMode(motor2in3, OUTPUT);
  pinMode(motor2in4, OUTPUT);
  pinMode(pwmMotor1, OUTPUT);
  pinMode(pwmMotor2, OUTPUT);

  pinMode(backSensorTrig, OUTPUT);
  pinMode(backSensorEcho, INPUT);
  pinMode(rightSensorTrig, OUTPUT);
  pinMode(rightSensorEcho, INPUT);

  pinMode(ir1digital, OUTPUT);
  pinMode(ir2digital, OUTPUT);
  pinMode(ir3digital, OUTPUT);
  pinMode(ir4digital, OUTPUT);
  pinMode(ir1analog, INPUT);
  pinMode(ir2analog, INPUT);
  pinMode(ir3analog, INPUT);
  pinMode(ir4analog, INPUT);

  currState = ORIENTING;
}

// ---- loop ----

void loop() {
  if (currState == ORIENTING)      handleOrienting();
  else if (currState == MOVE_TO_LINE)   handleMoveToLine();
  else if (currState == ALIGN_TO_LINE)  handleAlignToLine();
  // remaining states to be implemented
}
