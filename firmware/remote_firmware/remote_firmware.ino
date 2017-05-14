#include <serLCD.h>
#include <radio.h>
#include <quad_remote.h>
#include <flight_control.h>

const unsigned int DELAY_TIME = 30;
const unsigned int BUTTON_DELAY_TIME = 1000;

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

flightControlInfo info;



void setup()
{
  Serial.begin(9600);
  rfBegin(12);
  lcd = new serLCD();
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  initializeInfo();
}

void initializeInfo()
{
  info.header = HEADER;
  info.pitch = 0.0;
  info.roll = 0.0;
  info.throttle = 0.0;
  info.yaw = 0.0;
  info.pot1 = 0;
  info.pot2 = 0;
  info.button1 = 1;
  info.button2 = 1;
  info.footer = 0;
}


void loop()
{
  /*Serial.println(digitalRead(PIN_BTN2)); 
  Serial.println(analogRead(PIN_POT2)); //118-816 ////116 - 815
  lcd->clear();
  lcd->print(digitalRead(PIN_BTN2));*/
  unsigned char * infoPointer = (unsigned char*)&info;
  info.button1 = digitalRead(PIN_BTN1);
  info.button2 = digitalRead(PIN_BTN2);
  info.pot1 = analogRead(PIN_POT1);
  info.pot2 = analogRead(PIN_POT2);
  info.pitch = analogRead(PIN_PITCH)*PITCH_COEFFICIENT + PITCH_CONSTANT;
  info.roll = analogRead(PIN_ROLL)*ROLL_COEFFICIENT + ROLL_CONSTANT;
  info.throttle = analogRead(PIN_THROTTLE)*THROTTLE_COEFFICIENT + THROTTLE_CONSTANT;
  info.yaw = analogRead(PIN_YAW)*YAW_COEFFICIENT + YAW_CONSTANT;
  info.footer = calculateChecksum(infoPointer);
  
  rfWrite(infoPointer, INFO_SIZE);
  //lcd->clear();
  //lcd->print(analogRead(PIN_POT1));
  Serial.println(info.pot1);
 
  
  delay(DELAY_TIME); //TODO: Delay time needs to be adjusted
  if (info.button1 == 0 || info.button2 == 0) delay(BUTTON_DELAY_TIME);
}
