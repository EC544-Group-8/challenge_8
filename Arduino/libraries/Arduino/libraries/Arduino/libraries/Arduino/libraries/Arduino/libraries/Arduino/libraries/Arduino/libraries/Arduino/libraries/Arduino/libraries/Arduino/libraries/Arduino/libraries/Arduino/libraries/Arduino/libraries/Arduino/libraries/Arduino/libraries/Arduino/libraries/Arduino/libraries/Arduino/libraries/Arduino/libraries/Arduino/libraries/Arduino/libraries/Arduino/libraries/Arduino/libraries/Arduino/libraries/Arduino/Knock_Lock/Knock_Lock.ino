#include <Servo.h>
Servo myServo;

const int piezo = A0;
const int switchPin = 2;
const int YellowLed = 3;
const int greenLed = 4
const int redLed = 5;

int knockVal;
int switchVal;

const int quietKnock = 10;
const int loudKnock = 100;
boolean locked = false;
int numberOfKnocks = 0;

void setup()
 {
  myServo.attach(9);
  pinMode(yellowLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(switchPin, INPUT);
  Serial.begin(9600);
  digitalWrite(greenLed, HIGH);
  myservo.write(0);
  Serial.println("The box is unlocked!");
 }
 
 void loop()
 {
   if(locked == false)
     {
       switchVal = digitalRead(switchPin);
       if(switchVal == HIGH)
         {
           digitalWrite(greenLed, LOW);
           digitalWrite(redLed, HIGH);
           myServo.write(90);
           Serial.println ("The box is locked!");
           delay(1000);
         }
     }
     if(locked == true)
       {
         knockVal = analogRead(piezo);
         
           
  
