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
float right_orienting_thresh = 8.5;
float back_orienting_thresh = 35.0;
uint8_t irLineThresh = 50; // IR reading BELOW this means on the line (line absorbs IR)

uint8_t slowMotorSpeed = 120; // for ALIGN_TO_LINE, tune during testing

typedef enum {
  ORIENTING, MOVE_TO_LINE, ALIGN_TO_LINE, LINE_FOLLOWING,
  RELEASE_PUCK, BACKUP_TO_WALL, ORIENT_TO_ENTER_BOX, ENTER_BOX

// ORIENTING -> rotate left until right and back ultrasonic sensor values are within a certain threshold
// MOVE_TO_LINE -> move forward until you detect line on the front
// ALIGN_TO_LINE -> take a non-sharp turn (one motor slower than other) left until you detect line on the front
// LINE_FOLLOWING -> line following until outer left and right IR line sensors become 0 (detect black line)
// RELEASE_PUCK -> release the servo motor for the pucks with a 2.5 second timer
// BACKUP_TO_WALL -> line follow, but backwards, until your back ultrasonic sensor is within a certain threshold
// ORIENT_TO_ENTER_BOX -> take a sharp turn right until right ultrasonic sensor is within a certain threshold value
// ENTER_BOX -> move backwards until within a certain threshold for back ultrasonic sensor
// loop back to ORIENTING state

} States_t;

States_t currState;
States_t prev_state; // keep track of previous state to prevent motor
                     // from updating values at each loop iteration

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
  motorBackward(motor1in1, motor1in2, pwmMotor1);
  motorForward(motor2in3, motor2in4, pwmMotor2);
}

void turnRight() {
  motorForward(motor1in1, motor1in2, pwmMotor1);
  motorBackward(motor2in3, motor2in4, pwmMotor2);
}

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

void readAllIR(uint8_t results[4]) {
  results[0] = readIRSensor(ir1analog, ir1digital); // front right
  results[1] = readIRSensor(ir2analog, ir2digital); // front center-right
  results[2] = readIRSensor(ir3analog, ir3digital); // front center-left
  results[3] = readIRSensor(ir4analog, ir4digital); // front left
}

// ---- state handlers ----

void handleOrienting(void) {
  if (prev_state != ORIENTING) {
    turnLeft(); // set motors once on state entry
    prev_state = ORIENTING;
  }

  float rightDist = readUltrasonicSensor(rightSensorTrig, rightSensorEcho);
  float backDist  = readUltrasonicSensor(backSensorTrig, backSensorEcho);

  Serial.print("ORIENTING | right: "); 
  Serial.print(rightDist);
  Serial.print(" back: "); 
  Serial.println(backDist);

  if (rightDist <= right_orienting_thresh && backDist <= back_orienting_thresh) {
    stopMotors();
    Serial.println("-> MOVE_TO_LINE");
    currState = MOVE_TO_LINE;
  }
}

void handleMoveToLine(void) {
  if (prev_state != MOVE_TO_LINE) {
    moveForward(); // set motors once on state entry
    prev_state = MOVE_TO_LINE;
  }

  uint8_t ir[4];
  readAllIR(ir);

  Serial.print("MOVE_TO_LINE | ir: ");
  for (int i = 0; i < 4; i++) { 
    Serial.print(ir[i]); 
    Serial.print(" "); 
  }
  Serial.println();

  if (ir[1] == 0 && ir[2] == 0) { // both of the center ir line sensors detect
    // the black line, assuming both sensors are placed closed enough
    stopMotors();
    Serial.println("-> ALIGN_TO_LINE");
    currState = ALIGN_TO_LINE;
  }
}

void handleAlignToLine(void) {
  if (prev_state != ALIGN_TO_LINE) {
    slowTurnLeft(); // set motors once on state entry
    prev_state = ALIGN_TO_LINE;
  }

  uint8_t ir[4];
  readAllIR(ir);

  Serial.print("ALIGN_TO_LINE | ir: ");
  for (int i = 0; i < 4; i++) { Serial.print(ir[i]); Serial.print(" "); }
  Serial.println();

  if (ir[1] == 0 && ir[2] == 0) {
    stopMotors();
    Serial.println("-> LINE_FOLLOWING");
    currState = LINE_FOLLOWING;
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
  prev_state = ENTER_BOX; // deliberately different so first state entry always triggers motor setup
}

// ---- loop ----

void loop() {
  switch (currState) {
    case ORIENTING:      
      handleOrienting();    
      break;
    case MOVE_TO_LINE:   
      handleMoveToLine();   
      break;
    case ALIGN_TO_LINE:  
      handleAlignToLine();  
      break;
    case LINE_FOLLOWING:
    case RELEASE_PUCK:
    case BACKUP_TO_WALL:
    case ORIENT_TO_ENTER_BOX:
    case ENTER_BOX:
      // to be implemented
      break;
    default:
      Serial.println("Unknown state — should never get here");
      break;
  }
}
