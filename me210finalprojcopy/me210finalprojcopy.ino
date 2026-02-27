//PINS ASSUMING AN ARDUINO MEGA: https://docs.arduino.cc/resources/pinouts/A000067-full-pinout.pdf 

// l289n motor driver "in#" to digital pins
uint8_t motor1in1 = 22;
uint8_t motor1in2 = 24;
uint8_t motor2in3 = 28;
uint8_t motor2in4 = 26;

// l289n motor driver "enable" to digital pwm pins
uint8_t pwmMotor1 = 7;
uint8_t pwmMotor2 = 8;

// both ultrasonic sesnors
uint8_t backSensorTrig = 52;
uint8_t backSensorEcho = 50;

uint8_t rightSensorTrig = 53;
uint8_t rightSensorEcho = 51;

// all 4 ir sensors, one analog one digital per sensor
uint8_t ir1digital = 48;
uint8_t ir1analog = A0; //digital 55

uint8_t ir2digital = 56;
uint8_t ir2analog = A3;

uint8_t ir3digital = 58;
uint8_t ir3analog = A5;

uint8_t ir4digital = 60;
uint8_t ir4analog = A7;

// pins needed for the servo that controls the puck gate

typedef enum { // defining states
ORIENTING, ESCAPE, FOLLOW_LINE, RELOAD, START_PUCK_RELEASE, DROP_PUCKS,
} States_t ; 

States_t currState;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);

  // setup motor 1 and 2 pins
  pinMode(motor1in1, OUTPUT);
  pinMode(motor1in2, OUTPUT);
  pinMode(motor2in3, OUTPUT);
  pinMode(motor2in4, OUTPUT);

  // setup pwm enable pins for motor driver
  pinMode(pwmMotor1, OUTPUT);
  pinMode(pwmMotor2, OUTPUT);

  //set up the back and right sesnors
  pinMode(backSensorTrig, OUTPUT);
  pinMode(backSensorEcho, INPUT);

  pinMode(rightSensorTrig, OUTPUT);
  pinMode(rightSensorEcho, INPUT);

  //setup the digital pins for the ir
  pinMode(ir1digital, OUTPUT);
  pinMode(ir2digital, OUTPUT);
  pinMode(ir3digital, OUTPUT);
  pinMode(ir4digital, OUTPUT);

  //setup analog pins for irs
  pinMode(ir1analog, INPUT);
  pinMode(ir2analog, INPUT);
  pinMode(ir3analog, INPUT);
  pinMode(ir4analog, INPUT);

  // once power switches on, we are immediately going to start orienting
  currState = ORIENTING; 
}

void motorFoward(uint8_t pin1, uint8_t pin2, uint8_t enable) { //each motor is driven by 2 pins + enable pin
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  analogWrite(enable, 255); //full speed
}

void moveBackward(uint8_t pin1, uint8_t pin2, uint8_t enable) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  analogWrite(enable, 255); //full speed
}

float readUltrasonicSensor(uint8_t trig, uint8_t echo){
  long duration;
  float distance;

  // Clear trigger
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  // Send pulse
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Read echo time
  duration = pulseIn(echo, HIGH, 38000);

  // Convert to distance
  distance = duration * 0.0343 / 2;

  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" cm");

  return distance;
}

uint8_t readIRSensor(uint8_t analog, uint8_t digital){
  digitalWrite(digital, HIGH);
  delayMicroseconds(500);
  int withLight = analogRead(analog);

  digitalWrite(digital, LOW);
  delayMicroseconds(500);
  int noLight = analogRead(analog);

  return max(0, withLight - noLight);
}

void turnLeft(uint8_t lPin1, uint8_t lPin2, uint8_t lEnable,
              uint8_t rPin1, uint8_t rPin2, uint8_t rEnable) {
  // Left wheel backward, right wheel forward
  digitalWrite(lPin1, LOW);
  digitalWrite(lPin2, HIGH);
  analogWrite(lEnable, 255);

  digitalWrite(rPin1, HIGH);
  digitalWrite(rPin2, LOW);
  analogWrite(rEnable, 255);
}

float rightUSThresh = 8.5;
float backUSThres = 35.0;


void loop() {
  // put your main code here, to run repeatedly:
  // readUltrasonicSensor();

  //8.5 cm as right threshold , more sensitive
  // less than 35 for back , more flexible 
  // ir sensor must be 0

  motorFoward(motor1in1, motor1in2, pwmMotor1);
  // if (currState == ORIENTING){
  //   float currRightUS = readUltrasonicSensor(rightSensorTrig, rightSensorEcho);
  //   float currBackUS = readUltrasonicSensor(backSensorTrig, backSensorEcho);
  //   while (currRightUS > rightUSThresh && currBackUS > currBackUS) {
  //     // if we are too far from the back and too far from the right, we must keep turning 
  //     turnLeft(motor1in1, motor1in2, pwmMotor1, motor2in3, motor2in4, pwmMotor2);
  //   }
  // }


  // float dis = readUltrasonicSensor(rightSensorTrig, rightSensorEcho);
  // Serial.print("Distance right: ");
  // Serial.print(dis);
  // Serial.println(" cm");

  // float disB = readUltrasonicSensor(backSensorTrig, backSensorEcho);
  // Serial.print("Distance back: ");
  // Serial.print(disB);
  // Serial.println(" cm");


  // uint8_t ir = readIRSensor(ir1analog, ir1digital);
  // Serial.println(ir);

}
