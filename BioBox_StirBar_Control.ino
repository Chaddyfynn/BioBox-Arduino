#define OUT1 8
#define OUT2 9
#define OUT3 10
#define OUT4 11
#define FANPIN 12
#define IN1 A0

// Declaration of Globals
  bool gRunMotor = false;
  bool gAntiClockwise = false;
  bool gRunFan = false;
  unsigned long gRevPerMin = 500;
  float thresholdVal = 256;
  const unsigned int MAX_MESSAGE_LENGTH = 12;

void setup() {
  // Serial Port Setup
  Serial.begin(9600);

  // Pin Initialisation
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(OUT3, OUTPUT);
  pinMode(OUT4, OUTPUT);
  pinMode(FANPIN, OUTPUT);
  pinMode(IN1, INPUT);
}

void loop() {
  while (Serial.available() > 0){checkSerial();}
  if (gRunMotor){revolve(1, gRevPerMin, gAntiClockwise);}
  else{pinOut(false, false, false, false);}
  if (gRunFan){digitalWrite(FANPIN, HIGH);}
  else{digitalWrite(FANPIN, LOW);}
}

void checkSerial(){
 //Create a place to hold the incoming message
   static char message[MAX_MESSAGE_LENGTH];
   static unsigned int message_pos = 0;

   //Read the next available byte in the serial receive buffer
   char inByte = Serial.read();

   //Message coming in (check not terminating character) and guard for over message size
   if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
   {
     //Add the incoming byte to our message
     message[message_pos] = inByte;
     message_pos++;
   }
   //Full message received...
   else
   {
     //Add null character to string
     message[message_pos] = '\0';

     //Print the message (or do other things)
     // processSerial(message);

    if (message[0] == 'c')
     {
      if (message[1] == '0'){
        gRunMotor = false;
      }
      else if (message[1] == '1'){
        gRunMotor = true;
      }
    }
    else if (message[0] == 'S'){
      char RPM[12];
      int pos = 0;
      while (message[pos + 1] != '\0' && ((pos + 1) < 12)){
        RPM[pos] = message[pos + 1];
        pos++;
      }
      RPM[pos] = '\0';
      gRevPerMin = atoi(RPM);
    }
  else if (message[0] == 'r'){
    if (message[1] == '0'){
      gAntiClockwise = true;
    }
    else{
      gAntiClockwise = false;
    }
  }
  else if (message[0] == 'T'){
    if (message[1] == '0'){
      gRunFan = false;
    }
    else{
      gRunFan = true;
    }
  }

     //Reset for the next message
     message_pos = 0;
  }
}

void revolve(int numRevs, double timeIn, bool antiClockwise){
  // Calculate spacing between signal changes
  unsigned long waitTime;
  timeIn /= 60;
  timeIn = 1/timeIn;
  timeIn /= 8;
  timeIn *= 1000;
  waitTime = floor(timeIn);

  // Perform the correct number of revolutions
  if (antiClockwise){
    for (int i = 0; i < numRevs; i++){
      pinOut(true, false, false, false);
      delay(waitTime);
      pinOut(true, false, true, false);
      delay(waitTime);
      pinOut(false, false, true, false);
      delay(waitTime);
      pinOut(false, true, true, false);
      delay(waitTime);
      pinOut(false,true, false, false);
      delay(waitTime);
      pinOut(false, true, false, true);
      delay(waitTime);
      pinOut(false, false, false, true);
      delay(waitTime);
      pinOut(true, false, false, true);
      delay(waitTime);
    }
  }
  else{
    for (int i = 0; i < numRevs; i++){
      pinOut(true, false, false, true);
      delay(waitTime);
      pinOut(false, false, false, true);
      delay(waitTime);
      pinOut(false, true, false, true);
      delay(waitTime);
      pinOut(false, true, false, false);
      delay(waitTime);
      pinOut(false,true, true, false);
      delay(waitTime);
      pinOut(false, false, true, false);
      delay(waitTime);
      pinOut(true, false, true, false);
      delay(waitTime);
      pinOut(true, false, false, false);
      delay(waitTime);
    }
  }
}

void pinOut(bool a, bool b, bool c, bool d){
  // Easy function for writing to  pins
  if (a){
    digitalWrite(OUT1, HIGH);
  }
  else{
    digitalWrite(OUT1, LOW);
  }
  if (b){
    digitalWrite(OUT2, HIGH);
  }
  else{
    digitalWrite(OUT2, LOW);
  }
  if (c){
    digitalWrite(OUT3, HIGH);
  }
  else{
    digitalWrite(OUT3, LOW);
  }
  if (d){
    digitalWrite(OUT4, HIGH);
  }
  else{
    digitalWrite(OUT4, LOW);
  }
}
