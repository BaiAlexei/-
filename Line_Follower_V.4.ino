  #include <QTRSensors.h>

//PID
const double Kp = 0.070;
const double Ki = 0.0;
const double Kd = 0.0;
int lastError = 0;
int integral = 0;
const int MaxSpeed = 150; // max speed of the robot
const int BaseSpeed = 70; // this is the speed at which the motors should spin when the robot is perfectly on the line

//Motors
#define rightMotor1   8
#define rightMotor2   9
#define rightMotorPWM 11
#define leftMotor1    5
#define leftMotor2    7
#define leftMotorPWM  3
    
//Sensors
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2

QTRSensorsRC qtrrc((unsigned char[]) {A6, A7, A5, A4, A3, A2, A1, A0}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); // A7 - left sensors, A0 - right sensor
unsigned int sensorValues[NUM_SENSORS];

//Headlights
#define leftLight     2
#define rightLight    12

void leftmotor(int speed)
{
    Serial.println(speed);
    Serial.print('\t'); 

      if(speed >= 0)
      {
        digitalWrite(leftMotor1, LOW);
        digitalWrite(leftMotor2, HIGH);
        analogWrite(leftMotorPWM, speed);  
      }else
      {      
        digitalWrite(leftMotor1, HIGH);
        digitalWrite(leftMotor2, LOW);
        analogWrite(leftMotorPWM, -speed);
      }
}  

void rightmotor(int speed)
{
    Serial.println(speed);
    Serial.print('\t');  

      if(speed >= 0)
      {
        digitalWrite(rightMotor1, HIGH);
        digitalWrite(rightMotor2, LOW);
        analogWrite(rightMotorPWM, speed);  
      }else
      {      
        digitalWrite(rightMotor1, LOW);
        digitalWrite(rightMotor2, HIGH);
        analogWrite(rightMotorPWM, -speed);
      }
}  


void setup()
{
  //Headlights
  int Leftlight = 2;
  int Rightlight = 12;

  pinMode(Leftlight, OUTPUT);
  pinMode(Rightlight, OUTPUT);
  
  //Motors Set up
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  //Headlights Set up
  pinMode(leftLight, OUTPUT);
  pinMode(rightLight, OUTPUT);

  //Sensors Calibration
  delay(500);
  
  digitalWrite(leftLight, HIGH);    // turn on LEDs to indicate we are in calibration mode
  digitalWrite(rightLight, HIGH);
  for (int i = 0; i < 200; i++)  // adjust calibration lenght
  {
    qtrrc.calibrate();
  }      
  digitalWrite(leftLight, LOW);     // turn off LEDs to indicate we are through with calibration
  digitalWrite(rightLight, LOW);

  leftmotor(50);
  rightmotor(50);

  delay(500);
}

  
void loop()
{ 
  int position = qtrrc.readLine(sensorValues);
  int error = position - 3500;

  long int integral = integral + error; 
  if(integral < -50000 )
  {
   integral = -50000;
  }else if(integral > 50000 )
  {
   integral = 50000;
  }

  int motorSpeed = Kp * error + Kd * (error - lastError) + Ki * integral; 
  lastError = error;
    Serial.println(motorSpeed);
    Serial.print('\t');  

  int rightMotorSpeed = BaseSpeed - motorSpeed;
  int leftMotorSpeed = BaseSpeed + motorSpeed;
    Serial.println(leftMotorSpeed);
    Serial.print('\t');  
    Serial.println(rightMotorSpeed);
    Serial.print('\t');

  if(rightMotorSpeed > MaxSpeed)
  {
    rightMotorSpeed = MaxSpeed;
  }
  if(rightMotorSpeed < -MaxSpeed)
  {    
    rightMotorSpeed = -MaxSpeed;
  }
  if(leftMotorSpeed > MaxSpeed)
  {
    leftMotorSpeed = MaxSpeed;
  }
  if(leftMotorSpeed < -MaxSpeed)
  {
    leftMotorSpeed = -MaxSpeed;
  }

  leftmotor(leftMotorSpeed);
  rightmotor(rightMotorSpeed);
  
    Serial.println(leftMotorSpeed);
    Serial.print('\t');  
    Serial.println(rightMotorSpeed);
    Serial.print('\t');
    
  if(position >= 5000)
  {
    digitalWrite(rightLight, HIGH);
  }else if(position <= 2000)
  {
    digitalWrite(leftLight, HIGH);
  }else
  {
    digitalWrite(leftLight, HIGH);
    digitalWrite(rightLight, HIGH);
  }
  

  
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  // print motorSpeed
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(position); // comment this line out if you are using raw values

  digitalWrite(leftLight, LOW);
  digitalWrite(rightLight, LOW);
}
