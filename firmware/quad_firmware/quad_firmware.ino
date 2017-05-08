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

float pitchMax = 45; //approx
float pitchMin = -45; //approx
const int INT_SIZE = sizeof(int);
const int FLOAT_SIZE = sizeof(float);

const int HEADER_SIZE = FLOAT_SIZE;
const int FOOTER_SIZE = FLOAT_SIZE;

const int INFO_BUFFER_SIZE = HEADER_SIZE + 4*FLOAT_SIZE + FOOTER_SIZE; //4 floats, header, and the footer 
uint8_t infoBuffer [INFO_BUFFER_SIZE]; 
//uint8_t headerBuffer [HEADER_SIZE];

const int MOTOR_FR = 3; //green short
const int MOTOR_BL = 4; //green long
const int MOTOR_BR = 5; //yellow short
const int MOTOR_FL = 8; //yellow long

const float HEADER = 0xDEADBEEF;

const float THROTTLE_MAX = 1024;
const float THROTTLE_COEFFICIENT = 255;

const float YAW_MAX = 1315;
const float ROLL_MAX = 1136;
const float PITCH_MAX = 1390;

const float KP = 0.8;
const float KI = 5;
const float KD = 0.01;
float error = 0;
float errorSum = 0;
float lastError =0;


unsigned long lastTime = 0;

float throttleFR = 0;
float throttleBL = 0;
float throttleFL = 0;
float throttleBR = 0;
float pitchPID = 0;
float rollPID = 0;

float pitchReading = 0;
float rollReading = 0;
float gyroReading = 0;

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag(), &lsm.getGyro());

void setup()
{
  Serial.begin(115200);
  rfBegin(12);

  pinMode(MOTOR_FR, OUTPUT);
  pinMode(MOTOR_BL, OUTPUT);
  pinMode(MOTOR_BR, OUTPUT);
  pinMode(MOTOR_FL, OUTPUT);

  setupSensor();
  lastTime = millis();
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

float normalizePitch(float pitchValue)
{
  return pitchValue / (PITCH_MAX/(pitchMax - pitchMin)) - (pitchMax - pitchMin)/2.0;
}

void calculate_PID(float pitch, float roll, float throttle, float yaw)
{
  unsigned long now = millis();
  float timeChange = (float)(now - lastTime);
  
  error = normalizePitch(pitch) - pitchReading;
  errorSum += error * timeChange;
  float dError = (error -  lastError) / timeChange;
  
  pitchPID = KP * error + KI * errorSum + KD * dError;
  
  lastError = error;
  lastTime = now;

  //TODO: CHECK BACKWARDS
  throttleBL = limitThrottle(throttle + pitchPID + rollPID);
  throttleBR = limitThrottle(throttle + pitchPID - rollPID);
  throttleFL = limitThrottle(throttle - pitchPID + rollPID);
  throttleFR = limitThrottle(throttle - pitchPID - rollPID);
  if (pitchReading < pitchMin) pitchMin = pitchReading; //debugging
  if (pitchReading > pitchMax) pitchMax = pitchReading; //debugging
  Serial.print(pitchPID); Serial.print(" "); Serial.print(normalizePitch(pitch)); Serial.print(" "); Serial.println(pitchReading); 
  Serial.print(pitchMin); Serial.print(" "); Serial.println(pitchMax);
}

float limitThrottle(float throttleValue)
{
  if (throttleValue < 0.0) return 0;
  else if (throttleValue > 255.0) return 255;
  else return throttleValue;
}

void loop()
{
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    pitchReading = orientation.pitch;
    rollReading = orientation.roll;
    gyroReading = orientation.gyro_y;
    /*Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.gyro_y);
    Serial.println(F(""));*/
  }
  
  //delay(100);

  
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

      calculate_PID(pitch, roll, throttle, yaw);

      analogWrite(MOTOR_FR, throttleFR);
      analogWrite(MOTOR_BL, throttleBL);
      analogWrite(MOTOR_BR, throttleBR);
      analogWrite(MOTOR_FL, throttleFL);
    }
  }
}
