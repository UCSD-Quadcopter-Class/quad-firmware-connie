#include <radio.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_Sensor_Set.h>

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

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

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag(), &lsm.getGyro());

void setup()
{
  Serial.begin(115200);
  rfBegin(12);

  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(MOTOR_3, OUTPUT);
  pinMode(MOTOR_4, OUTPUT);

  setupSensor();
}

void setupSensor()
{
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  
    // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
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
    sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.gyro_y);
    Serial.println(F(""));
  }
  
  delay(100);

  
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
