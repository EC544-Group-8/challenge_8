// Program to control a R/c car in a straigh path down a hallway
// Sensors: LIDAR, Ultrasonic Range
// Control: PID
// Actuation: Servos

#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include "math.h"


//================================================
//                     Globals
//================================================
bool startup = true; // used to ensure startup only happens once
int startupDelay = 1000; // time to pause at each calibration step
double maxSpeedOffset = 45; // maximum speed magnitude, in servo 'degrees'
double maxWheelOffset = 85; // maximum wheel turn magnitude, in servo 'degrees'

// Max SOnar Sensor
const int sonarPin = 0; // used with the max sonar sensor
long anVolt, inches, cm;
int sum = 0; 
int avgRange = 60;

// Servo instances for controlling the vehicle
Servo wheels;
Servo esc;

// PID variables
double currentPos, steeringOut, setPos;
double sKp = 1, sKi = 0, sKd = 0;
double posError;
PID steeringPID(&currentPos, &steeringOut, &setPos,
                sKp, sKi,sKd,DIRECT);



//================================================
//                     Setup
//================================================
void setup()
{
  // Wheels and Motor
  wheels.attach(1);
  esc.attach(2);
  calibrateESC();

  // LIDAR

  // Ultrasonic Collision
}

//================================================
//                     Main
//================================================
void loop()
{

}

//================================================
//                  Functions
//================================================
void steeringPIDloop(void)
{
  posError = currentPos - setPos;
  steeringPID.SetTunings(sKp,sKi,sKd);
  steeringPID.Compute();
  wheels.write(1);
}

// Convert degree value to radians 
double degToRad(double degrees){
  return (degrees * 71) / 4068;
}

// Convert radian value to degrees 
double radToDeg(double radians){
  return (radians * 4068) / 71;
}

//Calibrate the ESC by sending a high signal, then a low, then middle
void calibrateESC(){
    esc.write(180); // full backwards
    delay(startupDelay);
    esc.write(0); // full forwards
    delay(startupDelay);
    esc.write(90); // neutral
    delay(startupDelay);
    esc.write(90); // reset the ESC to neutral (non-moving) value
}

void calcSonar(void)
{
  for(int i = 0; i < avgRange; i++)
  {
    anVolt = analogRead(sonarPin) / 2;
    sum += anVolt;
    delay(10);
  }
  inches = sum / avgRange;
  sum = 0;
}

//================================================
//                  Sytem Notes
//================================================
/*
    configuration:  Default 0.
      0: Default mode, balanced performance.
      1: Short range, high speed. Uses 0x1d maximum acquisition count.
      2: Default range, higher speed short range. Turns on quick termination
          detection for faster measurements at short range (with decreased
          accuracy)
      3: Maximum range. Uses 0xff maximum acquisition count.
      4: High sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for high sensitivity and noise.
      5: Low sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for low sensitivity and noise.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
*/
