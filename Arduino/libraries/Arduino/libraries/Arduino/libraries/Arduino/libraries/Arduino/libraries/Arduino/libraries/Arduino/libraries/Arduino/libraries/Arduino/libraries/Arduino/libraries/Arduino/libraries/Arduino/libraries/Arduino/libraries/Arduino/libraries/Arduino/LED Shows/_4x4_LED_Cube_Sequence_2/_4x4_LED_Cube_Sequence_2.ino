//4x4 LED Cube Sequence 2
//By: Tyler Manders
//7-12-2012
//lights go up and down each column one at a time
int C1 = 13;
int C2 = 12;
int C3 = 11;
int C4 = 10;
int C5 = 9;
int C6 = 8;
int C7 = 7;
int C8 = 6;
int C9 = 5;
int C10 = 4;
int C11 = 3;
int C12 = 2;
int C13 = 1;
int C14 = 0;
int C15 = 14;
int C16 = 15;
int L1 = 16;
int L2 = 17;
int L3 = 18;
int L4 = 19;

void setup() {
  pinMode(C1, OUTPUT);
  pinMode(C2, OUTPUT);
  pinMode(C3, OUTPUT);
  pinMode(C4, OUTPUT);
  pinMode(C5, OUTPUT);
  pinMode(C6, OUTPUT);
  pinMode(C7, OUTPUT);
  pinMode(C8, OUTPUT);
  pinMode(C9, OUTPUT);
  pinMode(C10, OUTPUT);
  pinMode(C11, OUTPUT);
  pinMode(C12, OUTPUT);
  pinMode(C13, OUTPUT);
  pinMode(C14, OUTPUT);
  pinMode(C15, OUTPUT);
  pinMode(C16, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  pinMode(L4, OUTPUT);
}

void loop() {
  digitalWrite(C1, HIGH); //C1 begins
  digitalWrite(L1, HIGH);
  delay(250);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L4, HIGH);
  delay(250);
  digitalWrite(C1, LOW);
  digitalWrite(C2, HIGH); //C2 begins
  delay(250);
  digitalWrite(L4, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L1, HIGH);
  delay(250);
  digitalWrite(C2, LOW);
  digitalWrite(C3, HIGH); //C3 begins
  delay(250);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L4, HIGH);
  delay(250);
  digitalWrite(C3, LOW);
  digitalWrite(C4, HIGH); //C4 Begins
  delay(250);
  digitalWrite(L4, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L1, HIGH);
  delay(250);
  digitalWrite(C4, LOW);
  digitalWrite(C8, HIGH); //C8 begins
  delay(250);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L4, HIGH);
  delay(250);
  digitalWrite(C8, LOW);
  digitalWrite(C7, HIGH); //C7 Begins
  delay(250);
  digitalWrite(L4, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L1, HIGH);
  delay(250);
  digitalWrite(C7, LOW);
  digitalWrite(C6, HIGH); //C6 begins
  delay(250);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L4, HIGH);
  delay(250);
  digitalWrite(C6, LOW);
  digitalWrite(C5, HIGH); //C5 Begins
  delay(250);
  digitalWrite(L4, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L1, HIGH);
  delay(250);
  digitalWrite(C5, LOW);
  digitalWrite(C9, HIGH); //C9 begins
  delay(250);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L4, HIGH);
  delay(250);
  digitalWrite(C9, LOW);
  digitalWrite(C10, HIGH); //C10 Begins
  delay(250);
  digitalWrite(L4, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L1, HIGH);
  delay(250);
  digitalWrite(C10, LOW);
  digitalWrite(C11, HIGH); //C11 begins
  delay(250);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L4, HIGH);
  delay(250);
  digitalWrite(C11, LOW);
  digitalWrite(C12, HIGH); //C12 Begins
  delay(250);
  digitalWrite(L4, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L1, HIGH);
  delay(250);
  digitalWrite(C12, LOW);
  digitalWrite(C16, HIGH); //C16 begins
  delay(250);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L4, HIGH);
  delay(250);
  digitalWrite(C16, LOW);
  digitalWrite(C15, HIGH); //C15 Begins
  delay(250);
  digitalWrite(L4, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L1, HIGH);
  delay(250);
  digitalWrite(C15, LOW);
  digitalWrite(C14, HIGH); //C14 begins
  delay(250);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L4, HIGH);
  delay(250);
  digitalWrite(C14, LOW);
  digitalWrite(C13, HIGH); //C13 Begins
  delay(250);
  digitalWrite(L4, LOW);
  digitalWrite(L3, HIGH);
  delay(250);
  digitalWrite(L3, LOW);
  digitalWrite(L2, HIGH);
  delay(250);
  digitalWrite(L2, LOW);
  digitalWrite(L1, HIGH);
  delay(250);
  digitalWrite(L1, LOW);
  digitalWrite(C13, LOW); //end 
  delay(250);
}

