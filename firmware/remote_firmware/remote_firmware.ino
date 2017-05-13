#include <serLCD.h>
#include <radio.h>
#include <quad_remote.h>

const unsigned int FLOAT_SIZE = sizeof(float);
const unsigned int UINT_SIZE = sizeof(unsigned int);

serLCD * lcd;
const float THROTTLE_MIN = 121.0;
const float THROTTLE_MAX = 816.0;
const float THROTTLE_COEFFICIENT = 1024.0/(THROTTLE_MAX - THROTTLE_MIN);
const float THROTTLE_CONSTANT = 1024.0 - THROTTLE_MAX*THROTTLE_COEFFICIENT;

const float YAW_MIN = 816.0;
const float YAW_MID = 494.0;
const float YAW_MAX = 124.0;
const float YAW_COEFFICIENT = 612.0/(YAW_MID - YAW_MIN);
const float YAW_CONSTANT = 612.0 - YAW_MID*YAW_COEFFICIENT;
//yaw goes from 0 -> 1315


const float ROLL_MIN = 138.0;
const float ROLL_MID = 503.0;
const float ROLL_MAX = 816.0;
const float ROLL_COEFFICIENT = 612.0/(ROLL_MID - ROLL_MIN);
const float ROLL_CONSTANT = 612.0 - ROLL_MID*ROLL_COEFFICIENT;
//roll from 0 -> 1136

const float PITCH_MIN = 816.0;
const float PITCH_MID = 514.0;
const float PITCH_MAX = 130.0;
const float PITCH_COEFFICIENT = 612.0/(PITCH_MID - PITCH_MIN);
const float PITCH_CONSTANT = 612.0 - PITCH_MID*PITCH_COEFFICIENT;
//pitch from 0 -> 1390 (more if pushed really hard)

struct flightControlInfo
{
  const float HEADER = 0xDEADBEEF;
  float pitch = 0.0;
  float roll = 0.0;
  float throttle = 0.0;
  float yaw = 0.0;
  unsigned int pot1 = 0;
  unsigned int pot2 = 0;
  unsigned int button1 = 1;
  unsigned int button2 = 1;
  float footer = 0x0;
} info;
  
const unsigned int INFO_SIZE = sizeof(flightControlInfo);



void setup()
{
  Serial.begin(9600);
  rfBegin(12);
  lcd = new serLCD();
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
}

uint8_t calculateChecksum(uint8_t * infoPointer)
{
  uint8_t checksum = 0;
  
  for (int i = 0; i < INFO_SIZE - FLOAT_SIZE; i++)
  {
    checksum ^= *(infoPointer++);
  }

  return checksum;  
}

void loop()
{
  /*Serial.println(digitalRead(PIN_BTN2)); 
  Serial.println(analogRead(PIN_POT2)); //118-816 ////116 - 815
  lcd->clear();
  lcd->print(digitalRead(PIN_BTN2));*/

  info.button1 = digitalRead(PIN_BTN1);
  info.button2 = digitalRead(PIN_BTN2);
  info.pot1 = analogRead(PIN_POT1);
  info.pot2 = analogRead(PIN_POT2);

  
  info.pitch = analogRead(PIN_PITCH)*PITCH_COEFFICIENT + PITCH_CONSTANT;
  info.roll = analogRead(PIN_ROLL)*ROLL_COEFFICIENT + ROLL_CONSTANT;
  info.throttle = analogRead(PIN_THROTTLE)*THROTTLE_COEFFICIENT + THROTTLE_CONSTANT;
  info.yaw = analogRead(PIN_YAW)*YAW_COEFFICIENT + YAW_CONSTANT;
  //lcd->print(info.throttle);
  uint8_t * infoPointer = (uint8_t*)&info;
  rfWrite(infoPointer, INFO_SIZE);
  info.footer = calculateChecksum(infoPointer);
  
  delay(25); //TODO: Delay time needs to be adjusted
  if (info.button1 == 0 || info.button2 == 0) delay(1000);
}
