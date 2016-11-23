#include <avr/pgmspace.h> // allows use of PROGMEM to store patterns in flash

#define CUBESIZE 4
#define PLANESIZE CUBESIZE*CUBESIZE
#define PLANETIME 100 // time each plane is displayed in us -> 100 Hz refresh
#define TIMECONST 20 // multiplies DisplayTime to get ms - why not =100?

// LED Pattern Table in PROGMEM - last column is display time in 100ms units
// TODO this could be a lot more compact but not with binary pattern representation
prog_uchar PROGMEM PatternTable[] = {

//1   //2   //3   //4   //5   //6   //7   //8   //9   //10  //11  //12  //13  //14  //15  //16     //row numbers, very useful
B0000,B0000,B0000,B0000,B0000,B0000,B0000,B0000,B0000,B0000,B0000,B0000,B0000,B0000,B0000,B0000,10,//empty set 
  
  
//Code goes here, i started you off above





// this is a dummy element for end of table (duration=0) aka !!!DO NOT TOUCH!!!
B0000, B0000, B0000, B0000, B0000, B0000, B0000, B0000, B0000, B0000, B0000, B0000, B0000, B0000, B0000, B0000, 0
};

/*
** Defining pins in array makes it easier to rearrange how cube is wired
** Adjust numbers here until LEDs flash in order - L to R, T to B
** Note that analog inputs 0-5 are also digital outputs 14-19!
** Pin DigitalOut0 (serial RX) and AnalogIn5 are left open for future apps
*/

int LEDPin[] = {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 14, 15};
int PlanePin[] = {16, 17, 18, 19};

// initialization
void setup()
{
int pin; // loop counter
// set up LED pins as output (active HIGH)
for (pin=0; pin<PLANESIZE; pin++) {
pinMode( LEDPin[pin], OUTPUT );
}
// set up plane pins as outputs (active LOW)
for (pin=0; pin<CUBESIZE; pin++) {
pinMode( PlanePin[pin], OUTPUT );
}
}

// display pattern in table until DisplayTime is zero (then repeat)
void loop()
{
// declare variables
byte PatternBuf[PLANESIZE]; // saves current pattern from PatternTable
int PatternIdx;
byte DisplayTime; // time*100ms to display pattern
unsigned long EndTime;
int plane; // loop counter for cube refresh
int patbufidx; // indexes which byte from pattern buffer
int ledrow; // counts LEDs in refresh loop
int ledcol; // counts LEDs in refresh loop
int ledpin; // counts LEDs in refresh loop

// Initialize PatternIdx to beginning of pattern table
PatternIdx = 0;
// loop over entries in pattern table - while DisplayTime>0
do {
// read pattern from PROGMEM and save in array
memcpy_P( PatternBuf, PatternTable+PatternIdx, PLANESIZE );
PatternIdx += PLANESIZE;
// read DisplayTime from PROGMEM and increment index
DisplayTime = pgm_read_byte_near( PatternTable + PatternIdx++ );
// compute EndTime from current time (ms) and DisplayTime
EndTime = millis() + ((unsigned long) DisplayTime) * TIMECONST;

// loop while DisplayTime>0 and current time < EndTime
while ( millis() < EndTime ) {
patbufidx = 0; // reset index counter to beginning of buffer
// loop over planes
for (plane=0; plane<CUBESIZE; plane++) {
// turn previous plane off
if (plane==0) {
digitalWrite( PlanePin[CUBESIZE-1], HIGH );
} else {
digitalWrite( PlanePin[plane-1], HIGH );
}

// load current plane pattern data into ports
ledpin = 0;
for (ledrow=0; ledrow<CUBESIZE; ledrow++) {
for (ledcol=0; ledcol<CUBESIZE; ledcol++) {
digitalWrite( LEDPin[ledpin++], PatternBuf[patbufidx] & (1 << ledcol) );
}
patbufidx++;
}

// turn current plane on
digitalWrite( PlanePin[plane], LOW );
// delay PLANETIME us
delayMicroseconds( PLANETIME );
} // for plane
} // while <EndTime
} while (DisplayTime > 0); // read patterns until time=0 which signals end
}

