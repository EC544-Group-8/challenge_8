
// defineing the global constants
// these are the digital output pins to the color changing LED

const int greenLedPin = 9;
const int redLedPin = 11;
const int blueLedPin = 10;

// these are the analog sensor pins that detect the light from the filmed photoresistors 

const int redSensorPin = A0;
const int blueSensorPin = A2;
const int greenSensorPin = A1;

// defining the variables
// these correspond to the digital output values to the color changing LED (common cathode)
// intensity corresponds to magnitude

int redValue = 0;
int greenValue = 0;
int blueValue = 0;

// these variables will change depending on the input value detected at the analog ligh sensor pins
// intensity corresponds with a larger magnitude

int redSensorValue = 0;
int greenSensorValue = 0;
int blueSensorValue = 0;

// the first function in this sketch

void setup() 
{
  Serial.begin (9600); // opens up a communication with the arduino
  
  pinMode(greenLedPin, OUTPUT); // sets the digital pins assigned above as outputs
  pinMode(redLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
}

void loop() 
{
  redSensorValue = analogRead(redSensorPin);
  delay (5);
  greenSensorValue = analogRead(greenSensorPin);
  delay(5);
  blueSensorValue = analogRead(blueSensorPin);
  
  Serial.print("Raw Sensor Value \t Red: ");
  Serial.print(redSensorValue);
  Serial.print(" \t Green: ");
  Serial.print(greenSensorValue);
  Serial.print(" \t Blue: ");
  Serial.print(blueSensorValue);
  
  redValue = redSensorValue/4;
  greenValue = greenSensorValue/4;
  blueValue = blueSensorValue/4;
  
  Serial.print("Mapped Sensor Values \t Red: ");
  Serial.print(redValue);
  Serial.print(" \t Green: ");
  Serial.print(greenValue);
  Serial.print("Ms \t Blue: ");
  Serial.print(blueValue);
  
  analogWrite(redLedPin, redValue);
  analogWrite(greenLedPin, greenValue);
  analogWrite(blueLedPin, blueValue);
}
  
