// Program to control a R/c car in a straigh path down a hallway
// Sensors: LIDAR, Ultrasonic Range
// Control: PID
// Actuation: Servos
#include <PID_v1.h>
#include <Servo.h>
#include "math.h"
#include "Wire.h"
#include <SharpIR.h>
#include <ArduinoSTL.h>
#include <NewPing.h>

//================================================
//                     Globals
//================================================ 
#define MIN_FRONT_IR_VALUE 60 // 5ft original
#define LIDAR_CALIBRATE_DIFFERENCE 0 //8
#define DEBUG 0 // 1 for debug mode, 0 for no, debug will disable motor and ultrasonic blocking
#define DISPLAY_PIDS_MSGS 1
#define DISPLAY_FRONTIR_MSGS 0
#define DISPLAY_RIGHTIR_MSGS 1
#define TRANSMIT_DELAY 20
#define MAX_WALL_DISTANCE 180
#define IRpin A1
#define IR_Sensor_Loop 5
#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 200

#define safe_to_turn_pin 5
#define remote_start_stop_pin 6
#define remote_left_pin 7
#define remote_right_pin 8
#define led_pin 8

bool startup = true;        // used to ensure startup only happens once
int startupDelay = 1000;    // time to pause at each calibration step
double maxSpeedOffset = 45; // maximum speed magnitude, in servo 'degrees'
double maxWheelOffset = 85; // maximum wheel turn magnitude, in servo 'degrees'

// Pins
int left_led = 6;
int right_led = 7;
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
unsigned long lastTurnTime = 0;

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
// 2 0 0  (Saved values for different speeds)
// 2 0 .04
double sKp = 2.2, sKi = 0.0, sKd = .05;
double posError;
PID steeringPID(&deltaD, &steeringOut, &setPos,
                sKp, sKi,sKd,DIRECT);
                
double distOfLeftWall;
double driftOut;
double driftSetPos = 100;   // upstairs
// double driftSetPos = 70; // UAV LAB
double driftRightSetPos = 40;
double distOfRightWall;

// 0.7 0 0      (Saved Values for different speeds)
// 0.7 0 0.04
double dKp = 0.7,dKi= 0.0,dKd = 0.04;
PID driftPID(&distOfLeftWall, &driftOut, & driftSetPos,
              dKp,dKi,dKd,DIRECT);
double dKpR = 4.0;
PID driftRightPID(&distOfRightWall, &driftOut, & driftRightSetPos,
              dKpR,dKi,dKd,DIRECT);


// IR Sensors
SharpIR IR_Front(GP2Y0A02YK0F,A0);
SharpIR IR_Right_Side(GP2Y0A02YK0F,A1);
int IR_Data[5];

// HCSR-04
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// communicating with PI
int safeToTurn = 1;
int byWindows = 1;
int lookToRight = 0;
int gapToLeft = 0;
int inTheMiddleOfATurn = 0;
int first_delta = 0;
int start_or_stop = 0;
int left = 0;
int right = 0;

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
  pinMode(led_pin, OUTPUT);

  // Initialize PIDs
  // Keep Crawler going Straight ahead
  steeringPID.SetSampleTime(100);
  steeringPID.SetOutputLimits(-90,90);
  steeringPID.SetMode(AUTOMATIC);
  
  // Don't Let Crawler get too close to left wall
  driftPID.SetSampleTime(100);
  driftPID.SetOutputLimits(-90,90);
  driftPID.SetMode(AUTOMATIC); 

  // Don't Let Crawler get too close to right wall
  driftRightPID.SetSampleTime(100);
  driftRightPID.SetOutputLimits(-90,90);
  driftRightPID.SetMode(AUTOMATIC); 

  // raspberry pi communication pins
  pinMode(safe_to_turn_pin,INPUT);
  pinMode(remote_start_stop_pin,INPUT);
  pinMode(remote_left_pin,INPUT);
  pinMode(remote_right_pin,INPUT);

}

//================================================
//                     Main
//================================================
void loop()
{
  start_or_stop = digitalRead(remote_start_stop_pin);
  if(start_or_stop || 1) {
    calcFrontSonar();
    if(DISPLAY_FRONTIR_MSGS){
      Serial.println("Inches: " + String(inches));  
    }
    while((inches > MIN_FRONT_IR_VALUE || DEBUG)) //&& start_or_stop ) // && motion_id == "1"
       {
        start_or_stop = digitalRead(remote_start_stop_pin);
        calcFrontSonar();
        String dist = String(inches);
        if(DISPLAY_FRONTIR_MSGS){
          Serial.println("FRONT_IR: " + dist);
        }

        // Check the right wall
        calcRightIR();

        // Calculate the left wall
        calcLidar();
        
        canITurn(deltaD);
        
        if(byWindows == 1) {
          if(DISPLAY_RIGHTIR_MSGS){
            Serial.println("RIGHT LOOP ");
          }
          driftRightPIDloop();
          wheels_write_value = 90+driftOut; // The drift out value is opposite of left side
        } else {
          driftPIDloop();
          steeringPIDloop();
          wheels_write_value = 90+(steeringOut - driftOut)/2;

        }
        
        if(DISPLAY_RIGHTIR_MSGS){
            Serial.println("wheels_write_value: " + String(wheels_write_value));
        }

        // Too close to right wall, with room on the left
        if(DISPLAY_RIGHTIR_MSGS){
          Serial.println("- Right wall: " + String(distOfRightWall) + " - Left wall: " + String(distOfLeftWall));
        }
        if(distOfRightWall < 20 && distOfLeftWall > 20) {
          if(DISPLAY_RIGHTIR_MSGS){
            Serial.println("Running into right WALLLLLLLLL!!! ");
          }
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
        if(!DEBUG){
          esc.write(70); // originally 60 as the prime value
        }
        if(DISPLAY_PIDS_MSGS){
          Serial.println("Steeringout: " + String(steeringOut));
          Serial.println("Driftout: " + String(driftOut));
        }

        if (DEBUG) {
          delay(1000);
        }
    }
    
    delay(100);
    esc.write(90); 
    unsigned long stop_difference;
    if(inTheMiddleOfATurn == 1) {
      stop_difference = (millis() - lastTurnTime);
    }
    wheels.write(85);
    Serial.println("stopping motors");
    delay(500);
    Serial.println("backing up");
    esc.write(105);
    delay(2000);
    Serial.println("return to previous state...");

    wheels.write(135);
    esc.write(70);
    delay(1000);
    if(inTheMiddleOfATurn == 1) {
      lastTurnTime = (millis() - stop_difference);
    }
    
  }
}

//================================================
//                  Functions
//================================================
int new_motion(String new_id) {
  motion_id = new_id;
  return 1;
}

void steeringPIDloop(void) {
  steeringPID.SetTunings(sKp,sKi,sKd);
  steeringPID.Compute();
  //wheels.write(90+steeringOut);
}

void driftPIDloop(void) {
  driftPID.SetTunings(dKp,dKi,dKd);
  driftPID.Compute();
  //wheels.write(90+steeringOut); 
}

void driftRightPIDloop(void) {
  driftRightPID.SetTunings(dKp,dKi,dKd);
  driftRightPID.Compute();
  //wheels.write(90+steeringOut); 
}

// Convert degree value to radians 
double degToRad(double degrees) {
  return (degrees * 71) / 4068;
 }

// Convert radian value to degrees 
double radToDeg(double radians) {
  return (radians * 4068) / 71;
}

void canITurn(double delta){
  // If the difference between the lidars is > 100, then 
  Serial.println("CAN I TURN");
  Serial.println(delta);
  Serial.println(safeToTurn);
  // If not in the middle of a turn
  if(inTheMiddleOfATurn == 0) {
    // If we see a big gap, and it is safe to rutn
    if((abs(delta) > 100 && safeToTurn == 1) || (first_delta > 0 && (lidar_dist_front > 150) && safeToTurn == 1) ) {
      if(first_delta < 2) {
        Serial.println("1st DELTA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        first_delta += 1; // this is our first big delta
      } else  {
        Serial.println("1st DELTA AND GAP LEFT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! STARTING TURN");
        lastTurnTime = millis();
        // start the turn
        inTheMiddleOfATurn = 1;
        first_delta = 0; // reset
      } 
    } else {
      first_delta = 0; // reset
    }
  }

  // if in the middle of a turn
  if(inTheMiddleOfATurn == 1) {
    Serial.print("time since last turn: ");
    Serial.println((millis() - lastTurnTime));
    // If it has been more than 5 seconds since our last turn, don't make turns for the next 15 seconds
    if( (millis() - lastTurnTime > 2500) && safeToTurn == 1) {
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! SHUTTING DOWN SAFE TO TURN FOR 15 SEC");
      safeToTurn = 0;
  
    } else if(millis() - lastTurnTime > 9000) {
      // If it has been more than 20 seconds since our last turn, we are safe to turn again
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! SAFE TO TURN AGAIN");
      safeToTurn = 1;
      inTheMiddleOfATurn = 0;
    }
  }
}


//Calibrate the ESC by sending a high signal, then a low, then middle
void calibrateESC() {
    esc.write(180); // full backwards
    delay(startupDelay);
    esc.write(0); // full forwards
    delay(startupDelay);
    esc.write(90); // neutral
    delay(startupDelay);
    esc.write(90); // reset the ESC to neutral (non-moving) value
}

void calcFrontSonar(void) {
  inches = sonar.ping_cm();
  if (inches <=10) {
    inches = 200;
  }
  if(DISPLAY_FRONTIR_MSGS){
    Serial.println("FRONT IR DIST: " + String(inches));
  }
  delay(40);
}

void calcRightIR() {
  for (int i = 0; i < IR_Sensor_Loop; i++) {
    IR_Data[i] = IR_Right_Side.getDistance();
  }
  distOfRightWall = IR_Data[IR_Sensor_Loop/2];
}

void calcLidar(void) {
    // int lidar_dist_back_avg, lidar_dist_front_avg = 0;
    // int n_samples = 3;
    // for(int i = 0; i < n_samples; i++){
        // ---------  THIS IS FOR THE FRONT LIDAR  -------------
        byWindows = digitalRead(safe_to_turn_pin); // check in with the PI to see if we are in a turning bin
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
    
        if(2 <= Wire.available()) { // if two bytes were received
            lidar_dist_front = Wire.read(); // receive high byte (overwrites previous reading)
            lidar_dist_front = lidar_dist_front << 8; // shift high byte to be high 8 bits
            lidar_dist_front |= Wire.read(); // receive low byte as lower 8 bits
            lidar_dist_front += LIDAR_CALIBRATE_DIFFERENCE;
            
            // handle noise in lidar
            if(lidar_dist_front < 10 || lidar_dist_front > 1500) {
              lidar_dist_front = last_lidar_dist_front;
            }
            
            if(lidar_dist_front > MAX_WALL_DISTANCE) {
              gapToLeft = 1;
              if(DISPLAY_RIGHTIR_MSGS){
                Serial.println("GAP TO LEFT - FRONT");
              }
            } else {
              gapToLeft = 0;
            }
            
            // Look to right sensor if there is a gap to left and we are not at turning point (gap)
            if(!byWindows && !safeToTurn) {
              if(DISPLAY_RIGHTIR_MSGS){
                Serial.println("LOOK TO RIGHT - FRONT");
              }
              lookToRight = 1;
              Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! LOOKING TO THE RIGHT");
              Serial.print("RIGHT IR VALUE: ");
              Serial.println(distOfRightWall);
//              lidar_dist_front = last_lidar_dist_front;
            } else {
              lookToRight = 0;
            } 
            
            last_lidar_dist_front = lidar_dist_front;
            // lidar_dist_front_avg += lidar_dist_front;
            String debug1 = "FRONT LIDAR...";
            debug1.concat(String(lidar_dist_front));
            if(DISPLAY_PIDS_MSGS){
              Serial.println(debug1);
            }
            
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
    
        if(2 <= Wire.available()) {// if two bytes were received
            lidar_dist_back = Wire.read(); // receive high byte (overwrites previous reading)
            lidar_dist_back = lidar_dist_back << 8; // shift high byte to be high 8 bits
            lidar_dist_back |= Wire.read(); // receive low byte as lower 8 bits

            // handle noise in lidar
            if(lidar_dist_back < 10 || lidar_dist_back > 1500) {
              lidar_dist_back = last_lidar_dist_back;
            }

            if(abs(lidar_dist_front - lidar_dist_back) < 75 && (lidar_dist_front < 150 && lidar_dist_back < 150)){
              Serial.println("SAFE TO USE LEFT AFTER ALL~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
              lookToRight = 0;
            }
            
            last_lidar_dist_back = lidar_dist_back;
            // lidar_dist_back_avg += lidar_dist_back;
            String debug2 = "BACK LIDAR...";
            debug2.concat(String(lidar_dist_back));
            if(DISPLAY_PIDS_MSGS){
              Serial.println(debug2);
            }
            //delay(1000);
            // Particle.publish("DEBUG",String(lidar_dist_back));
        }
    // lidar_dist_back = lidar_dist_back_avg / n_samples;
    // lidar_dist_front = lidar_dist_front_avg / n_samples;
    // ---------  END BACK LIDAR  -------------
  
  // Calculate deltaD
  deltaD = lidar_dist_back - lidar_dist_front;
  if(deltaD > 0) {
      distOfLeftWall = lidar_dist_front;
  } else {
      distOfLeftWall = lidar_dist_back;
  }
//   deltaD -= LIDAR_CALIBRATE_DIFFERENCE;
  //distOfLeftWall = (lidar_dist_back + lidar_dist_front)/2;
    if(DISPLAY_PIDS_MSGS){
      Serial.println("DELTA: " + String(deltaD));
    }

  // LED
  if(lookToRight){
    digitalWrite(led_pin, HIGH);
  } else {
    digitalWrite(led_pin, LOW);
  }
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
