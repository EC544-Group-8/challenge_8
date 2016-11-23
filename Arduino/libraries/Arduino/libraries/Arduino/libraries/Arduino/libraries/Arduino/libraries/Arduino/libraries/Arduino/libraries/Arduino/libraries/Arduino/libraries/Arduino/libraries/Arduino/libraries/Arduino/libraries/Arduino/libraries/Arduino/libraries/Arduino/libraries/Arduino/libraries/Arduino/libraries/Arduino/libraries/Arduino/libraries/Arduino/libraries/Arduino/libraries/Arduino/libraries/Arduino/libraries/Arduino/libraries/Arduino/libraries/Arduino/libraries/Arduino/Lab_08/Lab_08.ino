
const int switchPin = 8; // swith pin is attached to the arduino at point 8 
unsigned long previousTime = 0;// holds the time an LED was last changed
int switchState = 0;
int prevSwitchState = 0;
// these states will be compared within the loop

int led = 2; // the first lED in the sequence and is used to light the next
long interval = 1000; // corresponds to 10 seconds

// should totally make a pseudo code format for all of what I've done so far so i have the perfect template of what im doing with this all

void setup()
{ 
  // for loo pused to declare alll six LED pins as outputs
  
  for(int x = 2; x < 8; x++)
  {pinMode(x, OUTPUT);
  }
  pinMode(switchPin, INPUT);
}

void loop()
{
  unsigned long currentTime = millis(); // the amount of time the arduino has been running...thus it is a local variable 
  if(currentTime - previousTime > interval) // reffering to long interval of 10 min
  {previousTime = currentTime;
  digitalWrite(led, HIGH);
  led++;
  switchState = digitalRead(switchPin);
  
  if(led == 8)
  {
    delay(1000);
    int count = 0;
    for(int x = 2; x < 8; x++)
    {
      
      delay(100);
      digitalWrite(x, LOW);
      delay(100);
      digitalWrite(x, HIGH);
      delay(100);
      count++;
      if(count == 6)
      {
        for(int x = 2; x < 8; x++)
        {
          digitalWrite(x, LOW);
          led = 2;
          previousTime = currentTime;
        }
        prevSwitchState = switchState;
      }
    }
  }
  
  if(switchState != prevSwitchState)
  {
    for( int x = 2; x < 8; x++)
    {
      digitalWrite(x, LOW);
    }
    led = 2;
    previousTime = currentTime;
  }
  prevSwitchState = switchState;
}
}

  




