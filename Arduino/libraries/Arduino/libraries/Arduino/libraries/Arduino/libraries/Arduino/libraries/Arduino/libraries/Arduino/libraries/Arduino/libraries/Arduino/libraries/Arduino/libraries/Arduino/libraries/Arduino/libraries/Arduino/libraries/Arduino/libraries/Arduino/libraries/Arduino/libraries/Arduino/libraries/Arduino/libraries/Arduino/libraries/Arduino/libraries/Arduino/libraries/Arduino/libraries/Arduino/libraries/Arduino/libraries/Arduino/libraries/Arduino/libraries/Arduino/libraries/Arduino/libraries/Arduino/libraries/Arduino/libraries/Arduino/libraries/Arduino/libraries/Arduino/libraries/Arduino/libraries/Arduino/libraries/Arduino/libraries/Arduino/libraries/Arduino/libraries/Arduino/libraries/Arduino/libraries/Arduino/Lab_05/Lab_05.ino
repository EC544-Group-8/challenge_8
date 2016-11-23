

#include <Servo.h> // To  include the servo library 

Servo myServo; // in order to refer to the servo, create a named instance of the servo library in a variable: this is called an object
              // it is essentially a name that has all of the functions of the servo library
              
int const potPin = A0; //named constant for the pin the potentiometer is attached to
int potVal; // variable to hold the analog input value
int angle; // variable to hold the angle that you want the servo to move

void setup()
{
  myServo.attach(9); // tells arduino what pin the servo is attached to
  Serial.begin(9600); // begins the serial communication in the setup function 
}

void loop()
{
  potVal = analogRead(potPin); // reads the analog input and prints it out
  Serial.print("potVal:");
  Serial.print(potVal);
  
  angle = map(potVal, 0, 1023, 0, 179); // scales numbers...changes values from 0-1023 to 0-179 with use of 5 arguments 
  Serial.print(", angle: ");
  Serial.println(angle);
  
  myServo.write(angle); // moves the servo the desired angle
  delay(15); // needed to allow the servo to move to its new position 
}
  

