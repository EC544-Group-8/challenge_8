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
//                     Globals/Constants
//================================================ 
// Constants
#define MIN_FRONT_IR_VALUE 70 // 5ft original
#define LIDAR_CALIBRATE_DIFFERENCE 0 //8
#define DEBUG 0 // 1 for debug mode, 0 for no, debug will disable motor and ultrasonic blocking
#define DISPLAY_PIDS_MSGS 1
#define DISPLAY_FRONTIR_MSGS 1
#define DISPLAY_RIGHTIR_MSGS 0
#define TRANSMIT_DELAY 20
#define MAX_WALL_DISTANCE 180
#define IRpin A1
#define IR_Sensor_Loop 5
#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 200
#define startup true        // used to ensure startup only happens once
#define startupDelay 1000   // time to pause at each calibration step
#define maxSpeedOffset 45   // maximum speed magnitude, in servo 'degrees'
#define maxWheelOffset 85   // maximum wheel turn magnitude, in servo 'degrees'
#define FULL_SPEED 70
#define SLOW_SPEED 70
#define ESC_DELAY 100
#define STRAIGHT 90

// Pins
#define safe_to_turn_pin 5
#define remote_start_stop_pin 6
#define remote_look_right_pin 7
#define led_pin 9
#define left_led 6
#define right_led 7
#define back_LDR_pin 4
#define front_LDR_pin 13

// Lidar Lite V1
#define     LIDARLite_ADDRESS   0x62        // Default I2C Address of LIDAR-Lite.
#define     RegisterMeasure     0x00        // Register to write to initiate ranging.
#define     MeasureValue        0x04        // Value to initiate ranging.
#define     RegisterHighLowB    0x8f        // Register to get both High and Low bytes in 1 call.

// Global Variables
int lidar_dist_front = 0;
int last_lidar_dist_front = 0;
int lidar_dist_back = 0;
int last_lidar_dist_back = 0;
double deltaD = 0;
unsigned long lastTurnTime = 0;

// Max FrontIR Sensor
const int frontIRPin = A0; // used with the max frontIR sensor
long anVolt, inches, cm;
int sum = 0; 
int avgRange = 20;

// Servo instances for controlling the vehicle
Servo wheels;
Servo esc;
int wheels_write_value = 80;
int esc_write_value = FULL_SPEED;

// PID variables
double steeringOut = 0;
double setPos = 0;
double sKp = 2.2, sKi = 0.0, sKd = .05;
double posError;
PID steeringPID(&deltaD, &steeringOut, &setPos,
                sKp, sKi,sKd,DIRECT);
                
double distOfLeftWall;
double driftOut;
double driftSetPos = 120;   // 4th floor hallwy
// double driftSetPos = 70; // UAV LAB hallway
double driftRightSetPos = 40;
double distOfRightWall;

double dKp = 0.7,dKi= 0.0,dKd = 0.04;
PID driftPID(&distOfLeftWall, &driftOut, & driftSetPos,
              dKp,dKi,dKd,DIRECT);
              
double dKpR = 3.0;
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
int lookRightSignal = 0;
int left = 0;
int right = 0;

//================================================
//                     Setup
//================================================
void setup()
{
  // Enable Serial
  Serial.begin(9600);
  
  // Indicator LEDs
  pinMode(left_led, OUTPUT);
  pinMode(right_led, OUTPUT);
  
  // Wheels and Motor
  wheels.attach(3);
  esc.attach(2);
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
  pinMode(remote_look_right_pin,INPUT);
}

void changeLED(bool ledSignal) {
  // LEDCHANGE (debugging LED)
  if(ledSignal){
    digitalWrite(led_pin, HIGH);
  } else {
    digitalWrite(led_pin, LOW);
  }
}

//================================================
//                     Main
//================================================
void loop()
{
  // Read signals from Pi
  start_or_stop = digitalRead(remote_start_stop_pin);
  Serial.print("READING START_STOP AS: ");
  Serial.println(start_or_stop);
  lookRightSignal = digitalRead(remote_look_right_pin);
  changeLED(lookRightSignal || lookToRight);
  Serial.print("READING LOOK RIGHT AS: ");
  Serial.println(lookRightSignal);

  if(start_or_stop) {
    Serial.println("START START !!! START START");
  } else {
    Serial.println("STOP STOP !!! STOP STOP");
  } // end if
  
  if(start_or_stop) {
    calcFrontSonar();
    if(DISPLAY_FRONTIR_MSGS){
      Serial.print("FRONT READING: ");  
      Serial.println(inches);
  } // end if
    
  while((inches > MIN_FRONT_IR_VALUE || DEBUG) && start_or_stop ) {
        start_or_stop = digitalRead(remote_start_stop_pin);
        calcFrontSonar();
        String dist = String(inches);
        if(DISPLAY_FRONTIR_MSGS){
          Serial.println("FRONT_IR: " + dist);
        } // end if

        // Check the right wall
        calcRightIR();

        // Calculate the left wall
        calcLidar();
        
        canITurn(deltaD);
        
        lookRightSignal = digitalRead(remote_look_right_pin);
        changeLED(lookRightSignal);

        if(lookToRight == 1 || lookRightSignal == 1) {
          if(DISPLAY_RIGHTIR_MSGS){
            Serial.println("LOOKING RIGHT ");
          } // end if
          driftRightPIDloop();
          wheels_write_value = STRAIGHT + driftOut; // The drift out value is opposite of left side
        } else {
          driftPIDloop();
          steeringPIDloop();
          wheels_write_value = STRAIGHT + (int)((0.4 * (double)steeringOut - 0.6 * (double)driftOut));
        } // end else
        
        if(DISPLAY_RIGHTIR_MSGS){
            Serial.println("wheels_write_value: " + String(wheels_write_value));
        } // end if

        // Too close to right wall, with room on the left
        if(DISPLAY_RIGHTIR_MSGS){
          Serial.println("- Right wall: " + String(distOfRightWall) + " - Left wall: " + String(distOfLeftWall));
        } // end if
        
        if(distOfRightWall < 20 && distOfLeftWall > 20) {
          if(DISPLAY_RIGHTIR_MSGS){
            Serial.println("Running into right WALLLLLLLLL!!! ");
          } // end if
        } // end if
        
        // Turn Signal LEDs
        if(wheels_write_value < STRAIGHT){
          digitalWrite(right_led, LOW);
          digitalWrite(left_led, HIGH);
        } // end if
        else if (wheels_write_value > STRAIGHT){
          digitalWrite(left_led, LOW);
          digitalWrite(right_led, HIGH);
        } else {
          digitalWrite(left_led, LOW);
          digitalWrite(right_led, LOW);
        } // end else
     
          
        wheels.write(wheels_write_value);
        
        //avg outputs and write them to the servo
        if(!DEBUG && start_or_stop){
          esc.write(esc_write_value); // originally 60 as the prime value
        } // end if
        
        if(DISPLAY_PIDS_MSGS){
          Serial.println("Steeringout: " + String(steeringOut));
          Serial.println("Driftout: " + String(driftOut));
        } // end if

        if (DEBUG) {
          delay(1000);
        } // end if
    } // end while

    // Delay for switching ESC
    esc.write(90); 
    delay(ESC_DELAY);
    
    unsigned long stop_difference;
    if(inTheMiddleOfATurn == 1) {
      stop_difference = (millis() - lastTurnTime);
    } // end if
    
    if(start_or_stop) {
      wheels.write(85);
      Serial.println("stopping motors");
      delay(500);
      Serial.println("backing up");
      
      if(start_or_stop) {
        esc.write(105);
        delay(2000);
      } // end if
      
      Serial.println("return to previous state...");
      if(start_or_stop) {
        if(!lookRightSignal)
          wheels.write(135);
        else
          wheels.write(45);
          
        esc.write(esc_write_value);
        delay(1000);
      } // end if
      
      if(inTheMiddleOfATurn == 1 && start_or_stop) {
        lastTurnTime = (millis() - stop_difference);
      } // end if
      
    } else {
      // Do nothing
    } // end else
  } // end if(start_or_stop)
} // end while

//================================================
//                  Functions
//================================================
void steeringPIDloop(void) {
  steeringPID.SetTunings(sKp,sKi,sKd);
  steeringPID.Compute();
}

void driftPIDloop(void) {
  driftPID.SetTunings(dKp,dKi,dKd);
  driftPID.Compute();
}

void driftRightPIDloop(void) {
  driftRightPID.SetTunings(dKp,dKi,dKd);
  driftRightPID.Compute();
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
  Serial.print("DELTA IS: ");
  Serial.println(delta);
  Serial.print("SAFE TO TURN IS: ");
  Serial.println(safeToTurn);

  bool hasDelayed = false;
  // If not in the middle of a turn
  if(inTheMiddleOfATurn == 0) {
    // If we see a big gap, and it is safe to rutn
    if((abs(delta) > 100 && safeToTurn == 1) || (first_delta > 0 && (lidar_dist_front > 150) && safeToTurn == 1) ) {
      if(first_delta < 2) {
        Serial.println("1st DELTA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        first_delta += 1; // this is our first big delta
        // Slow down to check for more deltas
        esc_write_value = SLOW_SPEED;
        esc.write(SLOW_SPEED);
        if( 0 &&  !hasDelayed) { // hardcoded to 0 right now
          hasDelayed = true;
          delay(1000);
        }
        //delay(ESC_DELAY);
      } else  {
        Serial.println("1st DELTA AND GAP LEFT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! STARTING TURN");
        // Speed back up for turn
        esc_write_value = FULL_SPEED;
        esc.write(FULL_SPEED);

        //delay(ESC_DELAY);

        lastTurnTime = millis();
        // start the turn
        inTheMiddleOfATurn = 1;
        first_delta = 0; // reset
        hasDelayed = false;
      } 
    } else {
         first_delta = 0; // reset
         esc_write_value = FULL_SPEED;
         esc.write(FULL_SPEED);
         hasDelayed = false;
        //delay(ESC_DELAY);
    }
  }

  // if in the middle of a turn
  if(inTheMiddleOfATurn == 1) {
    Serial.print("time since last turn: ");
    Serial.println((millis() - lastTurnTime));
    if( (millis() - lastTurnTime > 2500) && safeToTurn == 1) {
      // If it has been more than 5 seconds since our last turn, don't make turns for the next 15 seconds
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
        Wire.beginTransmission((int)LIDARLite_ADDRESS);     // transmit to LIDAR-Lite
        Wire.write((int)RegisterMeasure);                   // sets register pointer to  (0x00)  
        Wire.write((int)MeasureValue);                      // sets register pointer to  (0x00)  
        Wire.endTransmission();                             // stop transmitting
    
        delay(TRANSMIT_DELAY);                              // Wait 20ms for transmit
    
        Wire.beginTransmission((int)LIDARLite_ADDRESS);     // transmit to LIDAR-Lite
        Wire.write((int)RegisterHighLowB);                  // sets register pointer to (0x8f)
        Wire.endTransmission();                             // stop transmitting
    
        delay(TRANSMIT_DELAY);                              // Wait 20ms for transmit
    
        Wire.requestFrom((int)LIDARLite_ADDRESS, 2);        // request 2 bytes from LIDAR-Lite
    
        if(2 <= Wire.available()) {                         // if two bytes were received
            lidar_dist_front = Wire.read();                 // receive high byte (overwrites previous reading)
            lidar_dist_front = lidar_dist_front << 8;       // shift high byte to be high 8 bits
            lidar_dist_front |= Wire.read();                // receive low byte as lower 8 bits
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
            
            if(!byWindows && !safeToTurn) {
              // Look to right sensor if there is a gap to left and we are not at turning point (gap)

              if(DISPLAY_RIGHTIR_MSGS){
                Serial.println("LOOK TO RIGHT - FRONT");
              }
              lookToRight = 1;
              Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! LOOKING TO THE RIGHT");
              Serial.print("RIGHT IR VALUE: ");
              Serial.println(distOfRightWall);
            } else {
              lookToRight = 0;
            } 
            
            last_lidar_dist_front = lidar_dist_front;
            if(DISPLAY_PIDS_MSGS){
              Serial.print("FRONT LIDAR... ");
              Serial.println(lidar_dist_front);
            }
            
        }
        // ---------  END FRONT LIDAR  -------------
        
        // ---------  THIS IS FOR THE BACK LIDAR  -------------
        digitalWrite(back_LDR_pin,HIGH);
        digitalWrite(front_LDR_pin,LOW);
        delay(5);
        Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
        Wire.write((int)RegisterMeasure);               // sets register pointer to  (0x00)  
        Wire.write((int)MeasureValue);                  // sets register pointer to  (0x00)  
        Wire.endTransmission();                         // stop transmitting
    
        delay(TRANSMIT_DELAY);                          // Wait 20ms for transmit
    
        Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
        Wire.write((int)RegisterHighLowB);              // sets register pointer to (0x8f)
        Wire.endTransmission();                         // stop transmitting
    
        delay(TRANSMIT_DELAY);                          // Wait 20ms for transmit
      
        Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite
    
        if(2 <= Wire.available()) {                 // if two bytes were received
            lidar_dist_back = Wire.read();          // receive high byte (overwrites previous reading)
            lidar_dist_back = lidar_dist_back << 8; // shift high byte to be high 8 bits
            lidar_dist_back |= Wire.read();         // receive low byte as lower 8 bits

            // handle noise in lidar
            if(lidar_dist_back < 10 || lidar_dist_back > 1500) {
              lidar_dist_back = last_lidar_dist_back;
            } // end if

            if(lookToRight == 1 && abs(lidar_dist_front - lidar_dist_back) < 75 && (lidar_dist_front < 150 && lidar_dist_back < 150)){
              // If left wall is in range and both sensors agree, use it
              Serial.println("SAFE TO USE LEFT WALL AFTER ALL~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
              lookToRight = 0;
            } // end if
            
            last_lidar_dist_back = lidar_dist_back;
            if(DISPLAY_PIDS_MSGS){
              Serial.print("BACK LIDAR... ");
              Serial.println(lidar_dist_back);
            } // end if
        } // end if
    // ---------  END BACK LIDAR  -------------
  
  // Calculate deltaD: (deltaD==0 means going straight)
  deltaD = lidar_dist_back - lidar_dist_front;
  
  if(deltaD > 0) {
    // Use the closer lidar to determine dist from wall
      distOfLeftWall = lidar_dist_front;
  } else {
      distOfLeftWall = lidar_dist_back;
  }
  
    if(DISPLAY_PIDS_MSGS){
      Serial.println("DELTA: " + String(deltaD));
    }
}
