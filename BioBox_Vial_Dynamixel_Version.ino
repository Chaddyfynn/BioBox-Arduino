#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> // libraries for the servo driver

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(); // instance of servo object from the driver

#define servoMin 450 // set these as the min and max angles during calibration
#define servoMax 3600 // should correspond to the 0 degree and 270 degree vials
#define servoChannel 0 // channel on the servo driver that the servo is connected to 

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(12, 13); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif


const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

using namespace ControlTableItem;

int frequency = 330; // frequency of PWM signals
int dutyCycle = servoMin; // start at vial number 1
float offset = 0;
int inpVal = 0; // input value to change the position of the servo as calibration
bool callibrated = false;
bool newPosition = false;
bool useVial = true;
int numCallibrated = 0;
int totalVials = 1;
const unsigned int MAX_MESSAGE_LENGTH = 12;
double setAngle = 0;

int vialDutyCycles[98]; 



void setup(){
  // servo.begin();
  vialDutyCycles[0] = servoMin;

  // servo.setPWMFreq(frequency);
  // servo.setPWM(servoChannel, 0, servoMin);

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(9600);
  while(!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 10);
}

void loop(){
  while (DEBUG_SERIAL.available() > 0){checkSerial();} // When msg is in buffer, run checkSerial()
   if (useVial){dynVial(inpVal);}
   else{dynAngle(setAngle);}
   delay(1);
}

// void goToVial(int vialNum){
//   // vialNum 0 is the first vial and therefore the index of the first vial too
//   dutyCycle = vialDutyCycles[vialNum];
//   goToDuty(dutyCycle);
// }

// void goToDuty(int dutyVal){
//   if (newPosition && (dutyVal <= vialDutyCycles[totalVials] + 150)){
//       servo.setPWM(servoChannel, 0, dutyVal + 150);
//       delay(500);
//       newPosition = false;
//     }
//     servo.setPWM(servoChannel, 0, dutyVal);
// }

void dynVial(int vialNum){
  float angle = 360 * vialNum / totalVials + offset;
  //DEBUG_SERIAL.print("a");
  //DEBUG_SERIAL.println(angle);
  //DEBUG_SERIAL.print("\n");
  dxl.setGoalPosition(DXL_ID, angle, UNIT_DEGREE);
}

void dynAngle(double angle){
  angle += offset;
  //DEBUG_SERIAL.print("a");
  //DEBUG_SERIAL.println(angle);
  //DEBUG_SERIAL.print("\n");
  dxl.setGoalPosition(DXL_ID, angle, UNIT_DEGREE);
}

void checkSerial(){
 //Create a place to hold the incoming message
   static char message[MAX_MESSAGE_LENGTH];
   static unsigned int message_pos = 0;

   //Read the next available byte in the serial receive buffer
   char inByte = DEBUG_SERIAL.read();

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

    if (message[0] == 'C')
     {
      int vialNum;
      char vialNumString[3];
      for (int i = 1; i < MAX_MESSAGE_LENGTH; i++){
        if (message[i] == 'D'){vialNumString[i - 1] = '\0'; break;}
        else{vialNumString[i - 1] = message[i];}
      }
      vialNum = atoi(vialNumString);
      int dutyIndex;
      char dutyCycleString[5];
      int dutyCycleInt;
      for (int i = 0; i < MAX_MESSAGE_LENGTH; i++){
        if (message[i] == 'D'){
          dutyIndex = i + 1;
          break;
        }
      }
      for (int i = dutyIndex; i < MAX_MESSAGE_LENGTH; i++){
        if (message[i] == '\0'){dutyCycleString[i - dutyIndex] = '\0'; break;}
        else{dutyCycleString[i - dutyIndex] = message[i];}
      }
      dutyCycleInt = atoi(dutyCycleString);
      vialDutyCycles[vialNum] = dutyCycleInt;
      numCallibrated ++;
      if (numCallibrated >= totalVials){
        callibrated = true;
      }
    }
    else if (message[0] == 'V'){
      if (message[1] == '0'){
        if (inpVal >= totalVials - 1){
          inpVal = 0;
          newPosition = true;
        }
        else{
        inpVal++;
        newPosition = true;
        }
      }
      else{
        if (inpVal == 0){
          inpVal = totalVials - 1;
          newPosition = true;
        }
        else{
        inpVal--;
        newPosition = true;
        }
      }
      useVial = true;
    }
  else if (message[0] == 'P'){
    char vialNumString[3];
      for (int i = 1; i < MAX_MESSAGE_LENGTH; i++){
        if (message[i] == '\0'){vialNumString[i - 1] = '\0'; break;}
        else{vialNumString[i - 1] = message[i];}
      }
      inpVal = atoi(vialNumString);
      newPosition = true;
      useVial = true;
    }
  else if (message[0] == 'T'){
    char totalVialString[3];
      for (int i = 1; i < MAX_MESSAGE_LENGTH; i++){
        if (message[i] == '\0'){totalVialString[i - 1] = '\0'; break;}
        else{totalVialString[i - 1] = message[i];}
      }
      totalVials = atoi(totalVialString) - 1;
  }
  else if (message[0] == 'D'){
    char inpMessage[6];
      for (int i = 1; i < MAX_MESSAGE_LENGTH; i++){
        if (message[i] == '\0'){inpMessage[i - 1] = '\0'; break;}
        else{inpMessage[i - 1] = message[i];}
      }
      offset = atoi(inpMessage);
  }
  else if (message[0] == 'A'){
    char inpMessage[7];
      for (int i = 1; i < MAX_MESSAGE_LENGTH; i++){
        if (message[i] == '\0'){inpMessage[i - 1] = '\0'; break;}
        else{inpMessage[i - 1] = message[i];}
      }
      setAngle = atof(inpMessage);
      useVial = false;
  }
  else if (message[0] == 'Q'){
    char inpMessage[7];
      for (int i = 1; i < MAX_MESSAGE_LENGTH; i++){
        if (message[i] == '\0'){inpMessage[i - 1] = '\0'; break;}
        else{inpMessage[i - 1] = message[i];}
      }
      int velocity = atoi(inpMessage);
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, velocity);
  }

     //Reset for the next message
     message_pos = 0;
  }
}