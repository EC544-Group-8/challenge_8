#include <LiquidCrystal.h>
LiquidCrystal lcd(12,11,5,4,3,2); // initialize the library telling it what pins it will be using ot communicate 

// set variables and constant since we just set up the library 

const int switchPin = 6;
int switchState = 0;
int previousSwitchState = 0;
int reply;

void setup()
{
  lcd.begin(16,2); // starts it and tells it the size of the screen
  pinMode(switchPin, INPUT);
  lcd.print("Ask the");
  lcd.setCursor(0,1); // move cursor and then print
  lcd.print("Crystal Ball!");
}

void loop()
{
  switchState = digitalRead(switchPin); // Define local variable 
  
 if (switchState != previousSwitchState)
 {
   if (switchState == LOW)
   {
     reply = random(8); // Sets the conditions to choose a random reply
     lcd.clear();
     lcd.setCursor(0,0);
     lcd.print("The ball says");
     lcd.setCursor(0,1);
     
     switch(reply)
     {
       case 0:
       lcd.print("Yes");
       break;
       case 1:
       lcd.print("Most likely");
       break;
       case 2:
       lcd.print("Certainly");
       break;
       case 3:
       lcd.print("Outlook good");
       break;
       case 4:
       lcd.print("Unsure");
       break;
       case 5:
       lcd.print("Ask again");
       break;
       case 6:
       lcd.print("Doubtful");
       break;
       case 7:
       lcd.print("No");
       break;
     }
   }
 }
 
 previousSwitchState = switchState; // CHanges the global variable in the loop so that the previous state is the current state so the program can detect if something has changed
}
       
     
  
  
