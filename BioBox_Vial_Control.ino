#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> // libraries for the servo driver
#include <Dynamixel2Arduino.h>

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(); // instance of servo object from the driver

#define servoMin 450 // set these as the min and max angles during calibration
#define servoMax 3600 // should correspond to the 0 degree and 270 degree vials
#define servoChannel 0 // channel on the servo driver that the servo is connected to 

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

int frequency = 330; // frequency of PWM signals
int dutyCycle = servoMin; // start at vial number 1
float offset = 0;
int inpVal = 0; // input value to change the position of the servo as calibration
bool callibrated = false;
bool newPosition = false;
bool useVial = false;
int numCallibrated = 0;
int totalVials = 1;
const unsigned int MAX_MESSAGE_LENGTH = 12;

int vialDutyCycles[98]; 



void setup(){
  Serial.begin(9600);
  // servo.begin();
  vialDutyCycles[0] = servoMin;

  // servo.setPWMFreq(frequency);
  // servo.setPWM(servoChannel, 0, servoMin);

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
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
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 30);
}

void loop(){
  while (Serial.available() > 0){checkSerial();} // When msg is in buffer, run checkSerial()
  dynVial(inpVal);
  // if (callibrated && useVial){goToVial(inpVal);}
  // else if (!useVial){goToDuty(dutyCycle);}
}

void goToVial(int vialNum){
  // vialNum 0 is the first vial and therefore the index of the first vial too
  dutyCycle = vialDutyCycles[vialNum];
  goToDuty(dutyCycle);
}

void goToDuty(int dutyVal){
  if (newPosition && (dutyVal <= vialDutyCycles[totalVials] + 150)){
      servo.setPWM(servoChannel, 0, dutyVal + 150);
      delay(500);
      newPosition = false;
    }
    servo.setPWM(servoChannel, 0, dutyVal);
}

void dynVial(int vialNum){
  float angle = 360 / totalVials;
  dxl.setGoalPosition(DXL_ID, angle * vialNum + offset, UNIT_DEGREE);
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
      inpVal = atoi(vialNumString) - 1;
      newPosition = true;
      useVial = true;
    }
  else if (message[0] == 'T'){
    char totalVialString[3];
      for (int i = 1; i < MAX_MESSAGE_LENGTH; i++){
        if (message[i] == '\0'){totalVialString[i - 1] = '\0'; break;}
        else{totalVialString[i - 1] = message[i];}
      }
      totalVials = atoi(totalVialString);
  }
  else if (message[0] == 'D'){
    char inpMessage[6];
      for (int i = 1; i < MAX_MESSAGE_LENGTH; i++){
        if (message[i] == '\0'){inpMessage[i - 1] = '\0'; break;}
        else{inpMessage[i - 1] = message[i];}
      }
      offset = atoi(inpMessage);
      // useVial = false;
  }

     //Reset for the next message
     message_pos = 0;
  }
}