//PINS ASSUMING AN ARDUINO MEGA: https://docs.arduino.cc/resources/pinouts/A000067-full-pinout.pdf 

#include <Servo.h> // library for the servo motor

// l298n motor driver "in#" to digital pins
uint8_t motor1in1 = 22; // left motor
uint8_t motor1in2 = 24;
uint8_t motor2in3 = 28; // right motor
uint8_t motor2in4 = 26;

// l298n motor driver "enable" to digital pwm pins
uint8_t pwmMotor1 = 7;
uint8_t pwmMotor2 = 8;

// servo motor: -
Servo puckServo;

uint8_t servoPin = 9;

uint8_t servoClosedAngle = 45; // starting position, tune during testing
uint8_t servoOpenAngle = 100; // 90 degrees anticlockwise from closed, tune during testing

// both ultrasonic sensors
uint8_t backSensorTrig = 52; // green
uint8_t backSensorEcho = 50; // purple

uint8_t rightSensorTrig = 53; // white
uint8_t rightSensorEcho = 51; // yellow

// all 4 ir sensors, one analog one digital per sensor
// direction when facing the bot: ir1 = front right, ir2 = front center-right, ir3 = front center-left, ir4 = front left
#define ir1analog A3 // purple wire
#define ir2analog A2 // blue wire
#define ir3analog A1 // green wire
#define ir4analog A0 // yellow wire

// thresholds - tune these during testing
float right_orienting_thresh = 13.0;
float back_orienting_thresh = 40.0;
float back_orienting_thresh_BackingToWall = 28.0; // to be tuned
float right_enter_box_thresh = 8.5; // to be tuned
float back_enter_box_thresh = 10.0; // to be tuned

float lineSensor_thresh = 500.0; // to be tuned

uint8_t slowMotorSpeed = 30; // for ALIGN_TO_LINE, tune during testing

unsigned long startTime;
unsigned long prevTime1; // TESTING
unsigned long prevTime2; // TESTING
unsigned long stateStartTime; 
const unsigned long competitionDuration = 120000; // 2 minutes in milliseconds

typedef enum {
  ORIENTING, MOVE_TO_LINE, ALIGN_TO_LINE, LINE_FOLLOWING,
  RELEASE_PUCK, BACKUP_TO_WALL, ORIENT_TO_ENTER_BOX, ENTER_BOX, WAIT_FOR_RELOAD

// ORIENTING -> rotate left until right and back ultrasonic sensor values are within a certain threshold
// MOVE_TO_LINE -> move forward until you detect line on the front
// ALIGN_TO_LINE -> take a non-sharp turn (one motor slower than other) left until you detect line on the front
// LINE_FOLLOWING -> line following until outer left and right IR line sensors become 0 (detect black line)
// RELEASE_PUCK -> release the servo motor for the pucks with a 2.5 second timer
// BACKUP_TO_WALL -> line follow, but backwards, until your back ultrasonic sensor is within a certain threshold
// ORIENT_TO_ENTER_BOX -> take a sharp turn right until right ultrasonic sensor is within a certain threshold value
// ENTER_BOX -> move backwards until within a certain threshold for back ultrasonic sensor
// WAIT_FOR_RELOAD -> wait 3 seconds in the same spot to allow us to reload the pucks before going to ORIENTING again
// loop back to ORIENTING state

} States_t;

States_t currState;
States_t prev_state; // keep track of previous state to prevent motor
                     // from updating values at each loop iteration

// ---- motor helpers ----

void motorForward(uint8_t pin1, uint8_t pin2, uint8_t enable) {
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  analogWrite(enable, 150);
}

void motorBackward(uint8_t pin1, uint8_t pin2, uint8_t enable) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  analogWrite(enable, 150);
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
  Serial.println("turning");
  motorBackward(motor1in1, motor1in2, pwmMotor1);
  motorForward(motor2in3, motor2in4, pwmMotor2);
}

void turnRight() {
  motorForward(motor1in1, motor1in2, pwmMotor1);
  motorBackward(motor2in3, motor2in4, pwmMotor2);
}

void slowTurnLeft() {
  analogWrite(pwmMotor1, 30);
  digitalWrite(motor1in1, HIGH);
  digitalWrite(motor1in2, LOW);

  analogWrite(pwmMotor2, 140);
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
  if (duration == 0) return -1; // timeout

  float dist = duration * 0.0343 / 2;

  if (dist < 2 || dist > 120) return -1;  // reject impossible values

  return dist;
}

void readAllIR(int results[4]) {
  results[0] = analogRead(ir1analog); // front right
  results[1] = analogRead(ir2analog); // front center-right
  results[2] = analogRead(ir3analog); // front center-left
  results[3] = analogRead(ir4analog); // front left
}

// ---- state handlers ----

void handleOrienting(void) {
  if (prev_state != ORIENTING) {
    turnRight(); // set motors once on state entry

    Serial.println("Inside condition");
    prev_state = ORIENTING;
  }

  float rightDist = readUltrasonicSensor(rightSensorTrig, rightSensorEcho);
  float backDist  = readUltrasonicSensor(backSensorTrig, backSensorEcho);

  Serial.print("rightDist: ");
  Serial.print(rightDist);
  Serial.print("backDist: ");
  Serial.println(backDist);

  // if ((rightDist <= right_orienting_thresh && rightDist >= 1.0) && backDist <= back_orienting_thresh )
  if (rightDist <= right_orienting_thresh && backDist <= back_orienting_thresh ) {
    stopMotors();

    Serial.println("Stopping now");
    currState = MOVE_TO_LINE;
  }
}

void handleMoveToLine(void) {
  // if (prev_state != MOVE_TO_LINE) {
  //   moveForward(); // set motors once on state entry
  //   prev_state = MOVE_TO_LINE;
  // }

  // int ir[4];
  // readAllIR(ir);

  // if (ir[1] < lineSensor_thresh || ir[2] < lineSensor_thresh) { // either of the center ir line sensors detect
  //   // the black line, assuming both sensors are placed closed enough
  //   stopMotors();
  //   Serial.println("handleMoveToLine");
  //   currState = ALIGN_TO_LINE;
  // }

  if (prev_state != MOVE_TO_LINE) {
    moveForward();
    prev_state = MOVE_TO_LINE;
  }

  float read_backUltra = readUltrasonicSensor(backSensorTrig, backSensorEcho); 

  if (read_backUltra >= 35.0) {
    Serial.println("handleMoveToLine passed");
    stopMotors();
    currState = ALIGN_TO_LINE;
  }
}

void handleAlignToLine(void) {
  if (prev_state != ALIGN_TO_LINE) {
    slowTurnLeft(); // set motors once on state entry
    delay(1000); 
    prev_state = ALIGN_TO_LINE;
  }

  int ir[4];
  readAllIR(ir);

  if (ir[1] > 100 || ir[2] > 100) {
    stopMotors();
    Serial.println("handleAlignToLine");
    currState = LINE_FOLLOWING;
  }
}

float Kp = 0.1; // tune during testing 
uint8_t baseSpeed = 110; // tune during testing 

void handleLineFollowing(void) {
  if (prev_state != LINE_FOLLOWING) {
    prev_state = LINE_FOLLOWING;
    // no motor set here since P control handles it dynamically every iteration
  }

  int ir[4];
  readAllIR(ir);

  // exit condition: both of the outer sensors detect black line
  if (ir[0] > 100 && ir[3] > 100) { 
    stopMotors();
    currState = RELEASE_PUCK;
    return;
  }

  // // error: negative = drifting right, positive = drifting left
  int error = ir[2] - ir[1];
  int correction = (int)(Kp * error);

  int leftSpeed  = constrain(baseSpeed + correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - correction, 0, 255);

  // apply speeds
  digitalWrite(motor1in1, HIGH);
  digitalWrite(motor1in2, LOW);
  analogWrite(pwmMotor1, leftSpeed); // adjust to leftSpeed

  digitalWrite(motor2in3, HIGH);
  digitalWrite(motor2in4, LOW);
  analogWrite(pwmMotor2, rightSpeed); // adjust to rightSpeed

  // printing every 1 second for debugging: -
  // if (millis() - prevTime1 >= 300) {
  //   Serial.print("ir[1]: ");
  //   Serial.print(ir[1]);
  //   Serial.print(" | ");

  //   Serial.print("ir[2]: ");
  //   Serial.println(ir[2]);

  //   Serial.print("Error: ");
  //   Serial.print(error);
  //   Serial.print(" | ");

  //   Serial.print("Correction: ");
  //   Serial.print(correction); 
  //   Serial.print(" | ");

  //   Serial.print("Left speed: ");
  //   Serial.println(leftSpeed);
  //   Serial.print(" | ");

  //   Serial.print("Right speed: ");
  //   Serial.print(rightSpeed);
  //   Serial.print(" | ");

  //   Serial.print("Error: ");
  //   Serial.print(error);
  //   Serial.print(" | ");
    
  //   Serial.print("Correction: ");
  //   Serial.print(correction);
  //   Serial.print(" | ");
    
  //   Serial.print("Ir2: ");
  //   Serial.print(ir[1]);
  //   Serial.print(" | ");
  //   Serial.print("Ir3: ");
  //   Serial.print(ir[2]);
  //   Serial.println(" | ");

  // prevTime1 = millis(); // update value
  // }
}

void handleReleasePuck(void) {
  // if (prev_state != RELEASE_PUCK) {
    prev_state = RELEASE_PUCK;
    puckServo.write(servoOpenAngle); // rotate 90 degrees anticlockwise

    Serial.println("Open");

    delay(3000);                     // hold for 3 seconds

    Serial.println("Closed");

    puckServo.write(servoClosedAngle); // return to closed position
    currState = BACKUP_TO_WALL;
  // }
}

void handleBackupToWall(void) {
  if (prev_state != BACKUP_TO_WALL) {
    prev_state = BACKUP_TO_WALL;
    // no motor set here since P control handles it dynamically every iteration
  }

  int ir[4];
  readAllIR(ir);

  float backDist = readUltrasonicSensor(backSensorTrig, backSensorEcho);

  // exit condition: back ultrasonic within threshold
  // if (backDist <= back_orienting_thresh_BackingToWall) {
  //   stopMotors();
  //   currState = ORIENT_TO_ENTER_BOX;
  //   return;
  // }

  // error flipped compared to LINE_FOLLOWING since we are moving backwards
   int error = ir[1] - ir[2];
  //int error = ir[2] - ir[1];
  int correction = (int)(Kp * error);

  int leftSpeed  = constrain(baseSpeed + correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - correction, 0, 255);

  // apply speeds backwards
  digitalWrite(motor1in1, LOW);
  digitalWrite(motor1in2, HIGH);
  analogWrite(pwmMotor1, leftSpeed);

  digitalWrite(motor2in3, LOW);
  digitalWrite(motor2in4, HIGH);
  analogWrite(pwmMotor2, rightSpeed);


  if (millis() - prevTime1 >= 300) {
    // Serial.print("ir[1]: ");
    // Serial.print(ir[1]);
    // Serial.print(" | ");

    // Serial.print("ir[2]: ");
    // Serial.println(ir[2]);

    // Serial.print("Error: ");
    // Serial.print(error);
    // Serial.print(" | ");

    // Serial.print("Correction: ");
    // Serial.print(correction); 
    // Serial.print(" | ");

    Serial.print("Left speed: ");
    Serial.println(leftSpeed);
    Serial.print(" | ");

    Serial.print("Right speed: ");
    Serial.print(rightSpeed);
    Serial.print(" | ");

    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" | ");
    
    Serial.print("Correction: ");
    Serial.print(correction);
    Serial.print(" | ");
    
    Serial.print("Ir2: ");
    Serial.print(ir[1]);
    Serial.print(" | ");
    Serial.print("Ir3: ");
    Serial.print(ir[2]);
    Serial.println(" | ");

    prevTime1 = millis(); // update value
  }

}

void handleOrientToEnterBox(void) {
  if (prev_state != ORIENT_TO_ENTER_BOX) {
    turnRight(); // set motors once on state entry
    prev_state = ORIENT_TO_ENTER_BOX;
  }

  float rightDist = readUltrasonicSensor(rightSensorTrig, rightSensorEcho);

  if (rightDist <= right_enter_box_thresh) {
    stopMotors();
    currState = ENTER_BOX;
  }
}

void handleEnterBox(void) {
  if (prev_state != ENTER_BOX) {
    moveBackward(); // set motors once on state entry
    prev_state = ENTER_BOX;
  }

  float backDist = readUltrasonicSensor(backSensorTrig, backSensorEcho);

  if (backDist <= back_enter_box_thresh) {
    stopMotors();
    currState = WAIT_FOR_RELOAD;
  }
}

void handleWaitForReload(void) {
  if (prev_state != WAIT_FOR_RELOAD) {
    prev_state = WAIT_FOR_RELOAD;
    delay(3000); // wait for puck reload
    currState = ORIENTING;
  }
}

// ---- setup ----

void setup() {
  Serial.begin(9600);

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

  pinMode(ir1analog, INPUT);
  pinMode(ir2analog, INPUT);
  pinMode(ir3analog, INPUT);
  pinMode(ir4analog, INPUT);

  // for servo motor: -
  puckServo.attach(servoPin);
  puckServo.write(servoClosedAngle); // closed angle


  // startTime = millis(); // to keep track of competition time

  // prevTime1 = millis(); // TESTING
  // prevTime2 = millis(); // TESTING

  stateStartTime = millis();

  currState = ORIENTING;
  prev_state = ENTER_BOX; // deliberately different so first state entry always triggers motor setup
}

// ---- loop ----

void loop() {
  
  // if (millis() - startTime >= competitionDuration) {
  //   stopMotors();
  //   while (true); // stop forever
  // }

  // if (millis() - prevTime2 >= 1000) { // printing every 1 second
  //   int ir_test[4];
  //   readAllIR(ir_test); 

    // int ir_test[4];
    // readAllIR(ir_test);

  //   prevTime2 = millis(); // update reading to print value every 1 second
  // }

  // float back_ultra = readUltrasonicSensor(backSensorTrig, backSensorEcho);
  // float right_ultra = readUltrasonicSensor(rightSensorTrig, rightSensorEcho); 

  // Serial.print("back: ");
  // Serial.print(back_ultra);
  // Serial.print("right: ");
  // Serial.println(right_ultra);

  // handleReleasePuck(); 

  moveForward();

  // switch (currState) {
  //   case ORIENTING:      
  //     handleOrienting();    
  //     break;
  //   case MOVE_TO_LINE:   
  //     handleMoveToLine();   
  //     break;
  //   case ALIGN_TO_LINE:  
  //     handleAlignToLine();  
  //     break;
  //   case LINE_FOLLOWING:
  //     handleLineFollowing();
  //     break;
  //   case RELEASE_PUCK:
  //     handleReleasePuck();
  //     break;
//     // case BACKUP_TO_WALL:
//     //   handleBackupToWall();
//     //   break;
//     // case ORIENT_TO_ENTER_BOX:
//     //   handleOrientToEnterBox();
//     //   break;
//     // case ENTER_BOX:
//     //   handleEnterBox();
//     //   break;
//     // case WAIT_FOR_RELOAD: 
//     //   handleWaitForReload();
//     //   break;
//     // default:
//     //   Serial.println("Unknown state — should never get here");
//     //   break;
    }
//}
