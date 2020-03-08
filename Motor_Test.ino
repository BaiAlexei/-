#define rightMotor1   8
#define rightMotor2   9
#define rightMotorPWM 11
#define leftMotor1    5
#define leftMotor2    7
#define leftMotorPWM  3


void setup() 
{


pinMode(rightMotor1, OUTPUT);
pinMode(rightMotor2, OUTPUT);
pinMode(leftMotor1, OUTPUT);
pinMode(leftMotor2, OUTPUT);
pinMode(rightMotorPWM, OUTPUT);
pinMode(leftMotorPWM, OUTPUT);
}

void loop() 
{





digitalWrite(rightMotor1, HIGH);
digitalWrite(rightMotor2, LOW);
digitalWrite(leftMotor1, HIGH);
digitalWrite(leftMotor2, LOW);


analogWrite(rightMotorPWM, 50);
analogWrite(leftMotorPWM, 50);

}
