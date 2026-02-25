//PINS ASSUMING AN ARDUINO MEGA: https://docs.arduino.cc/resources/pinouts/A000067-full-pinout.pdf 

// l289n motor driver "in#" to digital pins
uint8_t motor1in1 = 9;
uint8_t motor1in2 = 8;
uint8_t motor2in3 = 10;
uint8_t motor2in4 = 11;

// l289n motor driver "enable" to digital pwm pins
uint8_t pwmMotor1 = 13;
uint8_t pwmMotor2 = 12;

// both ultrasonic sesnors
uint8_t backSensorTrig = 7;
uint8_t backSensorEcho = 6;

uint8_t rightSensorTrig = 5;
uint8_t rightSensorEcho = 4;

// all 4 ir sensors, one analog one digital per sensor
uint8_t ir1digital = 54;
uint8_t ir1analog = A1; //digital 55

unit8_t ir2digital = 56
uint8_t ir2analog = A3;

unit8_t ir3digital = 58
uint8_t ir3analog = A5;

uint8_t ir4digital = 60
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
  pinMode(backSesnorTrig, OUTPUT);
  pinMode(backSensorEcho, INPUT);

  pinMode(rightSesnortrig, OUTPUT);
  pinMode(rightSensorEcho, INPUT);


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
  duration = pulseIn(echo, HIGH);

  // Convert to distance
  distance = duration * 0.0343 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

void readIRSensor(){

}


void loop() {
  // put your main code here, to run repeatedly:

}
