// l289n motor driver "in#" to digital pins
uint8_t motor1in1 = 54;
uint8_t motor1in2 = 55;
uint8_t motor2in3 = 56;
uint8_t motor2in4 = 57;

// l289n motor driver "enable" to digital pwm pins
uint8_t pwmMotor1 = 13;
uint8_t pwmMotor2 = 12;


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



void loop() {
  // put your main code here, to run repeatedly:

}
