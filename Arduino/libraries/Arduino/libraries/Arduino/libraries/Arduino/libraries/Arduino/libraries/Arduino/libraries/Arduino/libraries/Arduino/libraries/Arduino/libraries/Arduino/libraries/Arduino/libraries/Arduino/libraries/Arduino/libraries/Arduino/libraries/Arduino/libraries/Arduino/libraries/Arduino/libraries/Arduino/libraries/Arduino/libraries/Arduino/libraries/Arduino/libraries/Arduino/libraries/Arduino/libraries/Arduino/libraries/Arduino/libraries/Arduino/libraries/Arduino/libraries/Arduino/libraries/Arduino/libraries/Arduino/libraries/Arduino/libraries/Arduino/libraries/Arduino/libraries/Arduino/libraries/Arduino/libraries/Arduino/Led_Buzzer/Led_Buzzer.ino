const int piezo = A0;
const int firstLed = 5;
const int secLed = 6;
const int thrdLed = 7;
int pVal;


void setup()
{
  pinMode(piezo, INPUT);
  pinMode(firstLed, OUTPUT);
  pinMode(secLed, OUTPUT);
  pinMode(thrdLed, OUTPUT);
  Serial.begin(9600);
  
}

void loop()
{
  pVal = analogRead(piezo);
  Serial.println(pVal);
  if(pVal > 5)
  {
    digitalWrite(firstLed, HIGH);
  }
  
  if (pVal > 10)
  {
    digitalWrite(firstLed, HIGH);
    digitalWrite(secLed, HIGH);
  }
  
  if(pVal > 15)
  {
    digitalWrite(firstLed, HIGH);
    digitalWrite(secLed, HIGH);
    digitalWrite(thrdLed, HIGH);
  }
 digitalWrite(firstLed, LOW);
  digitalWrite(secLed, LOW);
  digitalWrite(thrdLed, LOW);
}
    
  
