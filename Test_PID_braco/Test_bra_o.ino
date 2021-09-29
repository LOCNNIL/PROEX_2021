#include "Servo.h"
#include <PID_v1.h>


#include <BMI160Gen.h>

const int select_pin = 10;
const int i2c_addr = 0x69;

double Setpoint, Input, Output=0;
double Kp=2.8, Ki=0.4, Kd=0.1; // Kp=2.3, Ki=0.4, Kd=0.05;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Servo motor;
int pwm_motor=0;

/*------------------*/

float acAngle_y;

void setup() {
  // put your setup code here, to run once:
  motor.attach(8);
  
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  // initialize device
  // BMI160.begin(BMI160GenClass::SPI_MODE, select_pin);
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);

  Input = 0;
  Setpoint = 90;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetOutputLimits(0,255);
  delay(5000);

}

void loop() {

  int ax, ay, az; // raw gyro values
 
  // read raw gyro measurements from device
  BMI160.readAccelerometer(ax, ay, az);
  acAngle_y = map(ay, -20000, 20000, 0 ,180);

  Input = acAngle_y;

//  Serial.print("Referencia:");
//  Serial.print(Setpoint);
//  Serial.print(" ");
//  Serial.print("angle:");
//  Serial.print(acAngle_y);
//  Serial.print(" ");
//  Serial.print("PID:");
//  Serial.print(Output);
//  Serial.println(); 
//  myPID.Compute(); 
//  motor.write(Output);
// Teste{ -----------------------------------------------
  if (Serial.available()>0){
    pwm_motor = Serial.parseInt();
    Serial.print("Angle:");
    Serial.print(acAngle_y);
    Serial.print(" ");
    Serial.print("PWM:");
    Serial.println(pwm_motor);
  }
  
  motor.write(pwm_motor);
  delay(2000);
 //} ----------------------------------------------- 
}
