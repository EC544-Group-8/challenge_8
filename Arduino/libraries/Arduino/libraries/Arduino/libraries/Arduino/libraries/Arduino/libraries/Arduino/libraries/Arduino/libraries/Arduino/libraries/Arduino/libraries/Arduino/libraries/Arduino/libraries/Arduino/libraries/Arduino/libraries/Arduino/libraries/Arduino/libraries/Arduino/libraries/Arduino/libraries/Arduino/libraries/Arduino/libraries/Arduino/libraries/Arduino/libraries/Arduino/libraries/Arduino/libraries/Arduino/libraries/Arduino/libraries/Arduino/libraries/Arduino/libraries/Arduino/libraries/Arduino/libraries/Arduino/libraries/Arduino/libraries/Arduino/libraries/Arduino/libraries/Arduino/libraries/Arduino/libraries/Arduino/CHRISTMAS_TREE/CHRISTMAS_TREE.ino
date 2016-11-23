const int redTree = 8;
const int greenTree = 2;
const int redLedPin  = 10;
const int blueLedPin = 12;
const int greenLedPin = 11;
const int treeBase = 3;

void setup()
{
  pinMode(redTree, OUTPUT);
  pinMode(greenTree, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(treeBase, OUTPUT);
  
  digitalWrite(greenTree, HIGH);
  digitalWrite(treeBase, HIGH);
  digitalWrite(redTree, HIGH);
}

void loop()
{
  digitalWrite(redTree, HIGH);
  delay(250);
  digitalWrite(redTree, LOW);  
  digitalWrite(redLedPin, HIGH);
  delay(250);
  digitalWrite(redLedPin, LOW);
  delay(250);
  digitalWrite(blueLedPin, HIGH);
  delay(250);
  digitalWrite(blueLedPin, LOW);
  delay(250);
  digitalWrite(greenLedPin, HIGH);
  delay(250);
  digitalWrite(greenLedPin, LOW);
}


  
  


  
