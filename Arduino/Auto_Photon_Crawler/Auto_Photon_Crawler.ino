//Program to control a R/c car in a straigh path down a hallway
// Sensors: LIDAR, Ultrasonic Range
// Control: PID
// Actuation: Servos
#include <PID_v1.h>
#include <Servo.h>
#include "math.h"
#include "Wire.h"

//================================================
//                     Globals
//================================================ 
#define MIN_FRONT_IR_VALUE 50 // 5ft original
#define LIDAR_CALIBRATE_DIFFERENCE 0 //8
#define DEBUG 0 // 1 for debug mode, 0 for no, debug will disable motor and ultrasonic blocking
#define TRANSMIT_DELAY 20
#define MAX_WALL_DISTANCE 140
#define IRpin A1


bool startup = true; // used to ensure startup only happens once
int startupDelay = 1000; // time to pause at each calibration step
double maxSpeedOffset = 45; // maximum speed magnitude, in servo 'degrees'
double maxWheelOffset = 85; // maximum wheel turn magnitude, in servo 'degrees'
int left_led = 6;//----------------------------------------------------------------------------
int right_led = 7;//----------------------------------------------------------------------------------
int back_LDR_pin = 4;
int front_LDR_pin = 13;

// Lidar Lite V1
#define     LIDARLite_ADDRESS   0x62        // Default I2C Address of LIDAR-Lite.
#define     RegisterMeasure     0x00        // Register to write to initiate ranging.
#define     MeasureValue        0x04        // Value to initiate ranging.
#define     RegisterHighLowB    0x8f        // Register to get both High and Low bytes in 1 call.

int lidar_dist_front = 0;
int last_lidar_dist_front = 0;
int lidar_dist_back = 0;
int last_lidar_dist_back = 0;
double deltaD = 0;

// Motion ID for stop and start from node.js app
int new_motion(String new_id); // Need forward declaration for use in "setup" loop (note, must take a string, return an int to work)
String motion_id = "0";

// Max FrontIR Sensor
const int frontIRPin = A0; // used with the max frontIR sensor
long anVolt, inches, cm;
int sum = 0; 
int avgRange = 20;

// Servo instances for controlling the vehicle
Servo wheels;
Servo esc;
int wheels_write_value = 80;

// PID variables
double steeringOut = 0;
double setPos = 0;
// 2 0 0
//2 0 .04
double sKp = 2.2, sKi = 0.0, sKd = .05;
double posError;
PID steeringPID(&deltaD, &steeringOut, &setPos,
                sKp, sKi,sKd,DIRECT);
                
double distOfLeftWall;
double driftOut;
double driftSetPos = 100; // upstairs
// double driftSetPos = 70; // UAV LAB

double distOfRightWall;
int safeToTurn = 0;
int gapToLeft = 0;

// 0.7 0 0 
// 0.7 0 0.04
double dKp = 0.7,dKi= 0.0,dKd = 0.04;
PID driftPID(&distOfLeftWall, &driftOut, & driftSetPos,
              dKp,dKi,dKd,DIRECT);

PID driftRightPID(&distOfRightWall, &driftOut, & driftSetPos,
              dKp,dKi,dKd,DIRECT);


//================================================
//                     Setup
//================================================
void setup()
{
  // Motion change function needs to be declared so its accessible to node.js app (through Particle cloud) 

  // Enable Serial
  Serial.begin(9600);
  
  // Indicator LEDs
  pinMode(left_led, OUTPUT);
  pinMode(right_led, OUTPUT);
  
  // Wheels and Motor
  wheels.attach(3);//------------------------------
  esc.attach(2);//--------------------------
  if(!DEBUG)
    calibrateESC();

  // LIDAR
  Wire.begin();
  pinMode(back_LDR_pin,OUTPUT);
  pinMode(front_LDR_pin,OUTPUT);

  // Initialize PIDs
  // Keep Crawler going Straight ahead
  steeringPID.SetSampleTime(100);
  steeringPID.SetOutputLimits(-60,60);
  steeringPID.SetMode(AUTOMATIC);
  
  // Don't Let Crawler get too close to left wall
  driftPID.SetSampleTime(100);
  driftPID.SetOutputLimits(-60,60);
  driftPID.SetMode(AUTOMATIC); 

  // Don't Let Crawler get too close to right wall
  driftRightPID.SetSampleTime(100);
  driftRightPID.SetOutputLimits(-60,60);
  driftRightPID.SetMode(AUTOMATIC); 
}

//================================================
//                     Main
//================================================
void loop()
{
  calcFrontIR();
  Serial.println("Inches: " + String(inches));
  while((inches > MIN_FRONT_IR_VALUE || DEBUG)) // && motion_id == "1"
  {
      calcFrontIR();
      String dist = String(inches);
      Serial.println("FRONT_IR: " + dist);

      // Check the right wall
      calcRightIR();

      // Calculate the left wall
      calcLidar();
      if(gapToLeft == 1) {
        driftRightPIDloop();
      } else {
        driftPIDloop();
        steeringPIDloop();
      }
      wheels_write_value = 90+(steeringOut - driftOut)/2;

      // Too close to right wall, with room on the left
      Serial.println("- Right wall: " + String(distOfRightWall));
      Serial.println("- Left wall: " + String(distOfLeftWall));
      if(distOfRightWall < 20 && distOfLeftWall > 20) {
        Serial.println("WALLLLLLLLL!!! ");
        wheels_write_value = 65;
      }
      
      // Turning LEDs
      if(wheels_write_value < 90){
        digitalWrite(right_led, LOW);
        digitalWrite(left_led, HIGH);
      }
      else if (wheels_write_value > 90){
        digitalWrite(left_led, LOW);
        digitalWrite(right_led, HIGH);
      }
        
      wheels.write(wheels_write_value);
      //avg outputs and write them to the servo
      if(!DEBUG)
        esc.write(80); // originally 60 as the prime value
      Serial.println("Steeringout: " + String(steeringOut));
      Serial.println("Driftout: " + String(driftOut));

      if (DEBUG) {
        delay(1000);
      }
  }
  //delay(10);
  wheels.write(wheels_write_value);
  esc.write(90); 
}

//================================================
//                  Functions
//================================================
int new_motion(String new_id){
  motion_id = new_id;
  return 1;
}

void steeringPIDloop(void)
{
  steeringPID.SetTunings(sKp,sKi,sKd);
  steeringPID.Compute();
  //wheels.write(90+steeringOut);
}

void driftPIDloop(void)
{
  driftPID.SetTunings(dKp,dKi,dKd);
  driftPID.Compute();
  //wheels.write(90+steeringOut); 
}

void driftRightPIDloop(void)
{
  driftRightPID.SetTunings(dKp,dKi,dKd);
  driftRightPID.Compute();
  //wheels.write(90+steeringOut); 
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

void calcFrontIR(void)
{
  float volts = analogRead(frontIRPin)*0.0048828125; ;
  inches = 65*pow(volts, -1.10);  
}

void calcRightIR() {
  float volts = analogRead(IRpin)*0.0048828125; ;
  distOfRightWall = 65*pow(volts, -1.10);
  Serial.println("Right wall: " + String(distOfRightWall));
}


void calcLidar(void)
{
    // int lidar_dist_back_avg, lidar_dist_front_avg = 0;
    // int n_samples = 3;
    // for(int i = 0; i < n_samples; i++){
        // ---------  THIS IS FOR THE FRONT LIDAR  -------------
        digitalWrite(back_LDR_pin,LOW);
        digitalWrite(front_LDR_pin,HIGH);
        delay(5);
        Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
        Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
        Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
        Wire.endTransmission(); // stop transmitting
    
        delay(TRANSMIT_DELAY); // Wait 20ms for transmit
    
        Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
        Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
        Wire.endTransmission(); // stop transmitting
    
        delay(TRANSMIT_DELAY); // Wait 20ms for transmit
    
        Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite
    
        if(2 <= Wire.available()) // if two bytes were received
        {
            lidar_dist_front = Wire.read(); // receive high byte (overwrites previous reading)
            lidar_dist_front = lidar_dist_front << 8; // shift high byte to be high 8 bits
            lidar_dist_front |= Wire.read(); // receive low byte as lower 8 bits
            lidar_dist_front += LIDAR_CALIBRATE_DIFFERENCE;
            // Dead reckon if sensor values are very high and not at turning point (gap) or noise
            if( (lidar_dist_front > MAX_WALL_DISTANCE && safeToTurn == 0) || lidar_dist_front < 10 )
            {
              gapToLeft = 1;
              Serial.println("DEAD RECKON FRONT");
              lidar_dist_front = last_lidar_dist_front;
            } else {
              gapToLeft = 0;
            } 
            
            last_lidar_dist_front = lidar_dist_front;
            // lidar_dist_front_avg += lidar_dist_front;
            String debug1 = "FRONT LIDAR...";
            debug1.concat(String(lidar_dist_front));
            Serial.println(debug1);
            
        }
        // ---------  END FRONT LIDAR  -------------
        
        // ---------  THIS IS FOR THE BACK LIDAR  -------------
        digitalWrite(back_LDR_pin,HIGH);
        digitalWrite(front_LDR_pin,LOW);
        delay(5);
        Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
        Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
        Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
        Wire.endTransmission(); // stop transmitting
    
        delay(TRANSMIT_DELAY); // Wait 20ms for transmit
    
        Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
        Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
        Wire.endTransmission(); // stop transmitting
    
        delay(TRANSMIT_DELAY); // Wait 20ms for transmit
    
        Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite
    
        if(2 <= Wire.available()) // if two bytes were received
        {
            lidar_dist_back = Wire.read(); // receive high byte (overwrites previous reading)
            lidar_dist_back = lidar_dist_back << 8; // shift high byte to be high 8 bits
            lidar_dist_back |= Wire.read(); // receive low byte as lower 8 bits
            // Dead reckon if sensor values are very high and not at turning point (gap) or noise
            if( (lidar_dist_back > MAX_WALL_DISTANCE && safeToTurn == 0) || lidar_dist_back < 10 )
            {
              gapToLeft = 1;
              Serial.println("DEAD RECKON BACK");
              lidar_dist_back = last_lidar_dist_back;
            } else {
              gapToLeft = 0;
            } 

            
            last_lidar_dist_back = lidar_dist_back;
            // lidar_dist_back_avg += lidar_dist_back;
            String debug2 = "BACK LIDAR...";
            debug2.concat(String(lidar_dist_back));
            Serial.println(debug2);
            //delay(1000);
            // Particle.publish("DEBUG",String(lidar_dist_back));
        }
    // lidar_dist_back = lidar_dist_back_avg / n_samples;
    // lidar_dist_front = lidar_dist_front_avg / n_samples;
    // ---------  END BACK LIDAR  -------------
  
  // Calculate deltaD
  deltaD = lidar_dist_back - lidar_dist_front;
  if(deltaD > 0)
  {
      distOfLeftWall = lidar_dist_front;
  }
  else
  {
      distOfLeftWall = lidar_dist_back;
  }
//   deltaD -= LIDAR_CALIBRATE_DIFFERENCE;
  //distOfLeftWall = (lidar_dist_back + lidar_dist_front)/2;
   Serial.println("DELTA: " + String(deltaD));
  // Print serial deltaD here 
    // }
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
