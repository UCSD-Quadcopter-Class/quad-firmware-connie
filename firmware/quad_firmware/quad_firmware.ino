#include <radio.h>

const int INT_SIZE = sizeof(int);
const int FLOAT_SIZE = sizeof(float);

const int HEADER_SIZE = FLOAT_SIZE;
const int FOOTER_SIZE = FLOAT_SIZE;

const int INFO_BUFFER_SIZE = HEADER_SIZE + 4*FLOAT_SIZE + FOOTER_SIZE; //4 floats, header, and the footer 
uint8_t infoBuffer [INFO_BUFFER_SIZE]; 
//uint8_t headerBuffer [HEADER_SIZE];

const int MOTOR_1 = 3;
const int MOTOR_2 = 4;
const int MOTOR_3 = 5;
const int MOTOR_4 = 8;

const float HEADER = 0xDEADBEEF;

const float THROTTLE_MAX = 1024;
const float THROTTLE_COEFFICIENT = 255;

const float YAW_MAX = 1315;
const float ROLL_MAX = 1136;
const float PITCH_MAX = 1390;

void setup()
{
  Serial.begin(9600);
  rfBegin(12);

  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(MOTOR_3, OUTPUT);
  pinMode(MOTOR_4, OUTPUT);
}

uint8_t calculateChecksum(uint8_t * infoPointer)
{
  uint8_t checksum = 0;
  
  for (int i = 0; i < INFO_BUFFER_SIZE - FLOAT_SIZE; i++)
  {
    checksum ^= *(infoPointer++);
  }

  return checksum;
}

void loop()
{
  while (rfAvailable())
  {
    int numBytesAvailable = rfAvailable(); //behaves weirdly if you use rfAvailable return value directly
    
    if (numBytesAvailable >= INFO_BUFFER_SIZE)
    {
      rfRead(infoBuffer, INFO_BUFFER_SIZE);
      float * infoPointer = (float*) infoBuffer;
      float header = *(infoPointer++);
      
      if(HEADER != header)
      {
        //Serial.println("Bad Header!");
        continue;
      }
      
      float pitch = *(infoPointer++);
      float roll = *(infoPointer++);
      float throttle = (*(infoPointer++));
      if(throttle > 1024.0 || throttle < 0) throttle = 0.0;

      throttle = throttle/THROTTLE_MAX * THROTTLE_COEFFICIENT;
      
      float yaw = *(infoPointer++);
      float footer = *(infoPointer);

      uint8_t checksum = calculateChecksum(infoBuffer);

      if(checksum != (uint8_t) footer)
      {
        //Serial.println("Bad packet!");
        continue;
      }

      analogWrite(MOTOR_1, throttle);
      analogWrite(MOTOR_2, throttle);
      analogWrite(MOTOR_3, throttle);
      analogWrite(MOTOR_4, throttle);
    }
  }
}
