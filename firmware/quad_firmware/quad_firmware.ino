#include <radio.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_Sensor_Set.h>
#include <flight_control.h>

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

bool reset = true;
bool motorsOn = true;
bool pidCalculationStarted = false;

float pitchMax = 45; //approx
float pitchMin = -45; //approx

const int HEADER_SIZE = FLOAT_SIZE;
const int FOOTER_SIZE = FLOAT_SIZE;

const int INFO_BUFFER_SIZE = HEADER_SIZE + 4*FLOAT_SIZE + 4*UINT_SIZE + FOOTER_SIZE; //4 floats, header, and the footer 
uint8_t infoBuffer [INFO_BUFFER_SIZE]; 
//uint8_t headerBuffer [HEADER_SIZE];

const int MOTOR_FR = 3; //green short
const int MOTOR_BL = 4; //green long
const int MOTOR_BR = 5; //yellow short
const int MOTOR_FL = 8; //yellow long


const float THROTTLE_MAX = 1024;
const float THROTTLE_COEFFICIENT = 255;

const float YAW_MAX = 1315;
const float ROLL_MAX = 1136;
const float PITCH_MAX = 1224; //1390 actual max
const float PITCH_MID = 612;
const float PITCH_MIN = 0;
const float PITCH_OFFSET = (pitchMax - pitchMin) / 2.0 - PITCH_MID / ((PITCH_MAX - PITCH_MIN) / (pitchMax - pitchMin));

const float KP = 1.8;
const float KI = 0;//0.0001;//0.0001;//0.01;
const float KD = 0.001;//0.0000001;//0.001;//0.001;
float error = 0;
float errorSum = 0;
float lastError =0;


unsigned long lastTime = 0;

const float OFFSET_FR = -0.5;//18;
const float OFFSET_BL = -0.5;//18;
const float OFFSET_BR = 0;//28;
const float OFFSET_FL = -2.5  ;//12;

float throttleFR = 0;
float throttleBL = 0;
float throttleFL = 0;
float throttleBR = 0;
float pitchPID = 0;
float rollPID = 0;

float filteredPitch = 0;
float pitchReadingAverage = 0;
//float lastPitchReading = 0;
float rollReadingAverage = 0;
float gyroReadingAverage = 0;

const int WINDOW_SIZE = 1;
int windowCounter = 0;
float pitchReadings [WINDOW_SIZE];
float rollReadings [WINDOW_SIZE];
float gyroReadings [WINDOW_SIZE];


float pitchOffset = 0;
float rollOffset = 0;
float gyroOffset = 0;

float pot1Value = 0;
float pot2Value=0;
unsigned int button1Value = 1; //reset
unsigned int button2Value = 1; //on/off

const float POT1_MIN = 115;
const float POT1_MAX = 816;
const float POT1_LIMIT_MIN = 0.0;
const float POT1_LIMIT_MAX = 4.0;
const float POT1_COEFFICIENT = (POT1_LIMIT_MAX - POT1_LIMIT_MIN)/(POT1_MAX - POT1_MIN);
const float POT1_CONSTANT = POT1_LIMIT_MAX - POT1_MAX * POT1_COEFFICIENT;


const float POT2_MIN = 0;
const float POT2_MAX = 1003;
const float POT2_LIMIT_MIN = 0.0;
const float POT2_LIMIT_MAX = 1.0;
const float POT2_COEFFICIENT = (POT2_LIMIT_MAX - POT2_LIMIT_MIN)/(POT2_MAX - POT2_MIN);
const float POT2_CONSTANT = POT2_LIMIT_MAX - POT2_MAX * POT2_COEFFICIENT;

float compCoefficient = 0;
const float COMP_COEFFICIENT = 0.8;


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

  setupWindow();
  setupSensor();
  //calibrateSensor();
  
  lastTime = millis();
}

void setupWindow()
{
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    pitchReadings[i] = 0;
    rollReadings[i] = 0;
    gyroReadings[i] = 0;
  }
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

void calibrateSensor()
{
  sensors_vec_t   orientation;
  float pitchValue = 0;
  float rollValue = 0;
  float gyroValue = 0;

  float pitchSum = 0;
  float rollSum = 0;
  float gyroSum = 0;

  for(int i = 0; i < 5; i++)
  {
    for(int j = 0; j < 10; j++)
    {
      // Use the simple AHRS function to get the current orientation.
      if (ahrs.getOrientation(&orientation))
      {
        pitchSum += orientation.pitch;
        rollSum += orientation.roll;
        gyroSum += orientation.gyro_y;
      }
    }

    pitchValue += pitchSum / 10.0;
    rollValue += rollSum / 10.0;
    gyroValue += gyroSum / 10.0;
  }

  pitchOffset = pitchValue / 5.0;
  rollOffset = rollValue / 5.0;
  gyroOffset = gyroValue / 5.0;

  //Serial.println(pitchOffset);
  //Serial.println(rollOffset);
  //Serial.println(gyroOffset);
}

float normalizePitch(float pitchValue)
{
  if (pitchValue < PITCH_MIN) pitchValue = PITCH_MIN;
  else if (pitchValue > PITCH_MAX) pitchValue = PITCH_MAX;
  return pitchValue / (PITCH_MAX/(pitchMax - pitchMin)) - (pitchMax - pitchMin)/2.0 + PITCH_OFFSET;
}

void calculatePID(float pitch, float roll, float throttle, float yaw)
{
  if (lastError < -200 || lastError > 200) lastError=0;
  unsigned long now = millis();
  float timeChange = (float)(now - lastTime);
  //compCoefficient = pot1Value; Serial.println(compCoefficient);

  if (pidCalculationStarted)
  {
    filteredPitch = COMP_COEFFICIENT * (filteredPitch + gyroReadingAverage * timeChange/1000.0) + (1.0 - COMP_COEFFICIENT) * pitchReadingAverage;
  }
  else
  {
    filteredPitch = pitchReadingAverage;
    pidCalculationStarted = true;
  }

  //Serial.print(pitchReadingAverage); Serial.print(" "); Serial.println(filteredPitch); //Serial.print(" "); Serial.println(gyroReadingAverage);
  //Serial.println(pitchReadingAverage); Serial.print(" ");
  //Serial.println(pot1Value);
  error = normalizePitch(pitch) - filteredPitch;
  errorSum += error * timeChange;
  float dError = (error -  lastError) / timeChange;
  
  pitchPID = pot1Value * error + KI * errorSum + pot2Value * dError;
  //if (pitchPID < -255 || pitchPID >255) Serial.println(pitchPID);  
  //if(lastError < -1000 || lastError > 1000) Serial.println(lastError);
  //if(dError < -100 || dError > 100.0) Serial.print(dError); Serial.print(" "); Serial.print(error); Serial.print(" "); Serial.print(lastError); Serial.print(" "); Serial.println(timeChange);
  lastError = error;
  lastTime = now;

  //TODO: CHECK BACKWARDS
  throttleBL = limitThrottle(throttle + pitchPID + rollPID + OFFSET_BL);
  throttleBR = limitThrottle(throttle + pitchPID - rollPID + OFFSET_BR);
  throttleFL = limitThrottle(throttle - pitchPID + rollPID + OFFSET_FL);
  throttleFR = limitThrottle(throttle - pitchPID - rollPID + OFFSET_FR);
  //if (pitchReadingAverage < pitchMin) pitchMin = pitchReadingAverage; //debugging
  //if (pitchReadingAverage > pitchMax) pitchMax = pitchReadingAverage; //debugging
  //Serial.print(pitchPID); Serial.print(" "); Serial.print(normalizePitch(pitch)); Serial.print(" "); Serial.println(pitchReadingAverage); 
  //Serial.print(pitchPID); Serial.print(" "); Serial.println(error);
  //Serial.print(pitchMin); Serial.print(" "); Serial.println(pitchMax);
  //Serial.println(gyroReadingAverage);
}

float limitThrottle(float throttleValue)
{
  if (throttleValue < 0.0) return 0;
  else if (throttleValue > 255.0) return 255;
  else return throttleValue;
}

void calculateReadingAverages()
{
  sensors_vec_t   orientation;
  
  if(reset)
  {
    windowCounter = 0;
    
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
      if (ahrs.getOrientation(&orientation))
      {
        pitchReadings[windowCounter] = orientation.pitch;
        rollReadings[windowCounter] = orientation.roll;
        gyroReadings[windowCounter] = orientation.gyro_y;  
        if (windowCounter == 10) windowCounter = 0;
        else windowCounter++;  
      }
    }
  }
  

  if (ahrs.getOrientation(&orientation))
  {
    pitchReadings[windowCounter] = orientation.pitch;
    rollReadings[windowCounter] = orientation.roll;
    gyroReadings[windowCounter] = orientation.gyro_y;
    pitchReadingAverage = 0;
    rollReadingAverage = 0;
    gyroReadingAverage = 0;
    

    for(int i = 0; i < WINDOW_SIZE; i++)
    {
      pitchReadingAverage += pitchReadings[i];
      rollReadingAverage += rollReadings[i];
      gyroReadingAverage += gyroReadings[i];
    }

    pitchReadingAverage /= float(WINDOW_SIZE);
    rollReadingAverage /= float(WINDOW_SIZE);
    gyroReadingAverage /= float(WINDOW_SIZE);

    if (windowCounter == WINDOW_SIZE) windowCounter = 0;
    else windowCounter++;  
    
    //Serial.println(pitchReadingAverage);
    /*Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.gyro_y);
    Serial.println(F(""));*/
  }

  
  pitchReadingAverage -= pitchOffset;
  rollReadingAverage -= rollOffset;
}

void loop()
{  
  calculateReadingAverages();
  if (reset)
  {
    pidCalculationStarted = false;
    reset = false;  
  }

  //Serial.println(pitchReadingAverage);
    
  if (rfAvailable())
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
        return;
      } //else Serial.println("Good");
      
      float pitch = *(infoPointer++);
      float roll = *(infoPointer++);
      float throttle = (*(infoPointer++));
      if(throttle > 1024.0 || throttle < 0) throttle = 0.0;

      throttle = throttle/THROTTLE_MAX * THROTTLE_COEFFICIENT;
      
      float yaw = *(infoPointer++);
      unsigned int * infoUIntPointer = (unsigned int *)infoPointer;
      pot1Value = float(*(infoUIntPointer++)) * POT1_COEFFICIENT + POT1_CONSTANT;
      if (pot1Value < POT1_LIMIT_MIN) pot1Value = POT1_LIMIT_MIN;
      else if(pot1Value > POT1_LIMIT_MAX) pot1Value = POT1_LIMIT_MAX;
     
      
      
      pot2Value = float(*(infoUIntPointer++)) * POT2_COEFFICIENT + POT2_CONSTANT;
      if (pot2Value < POT2_LIMIT_MIN) pot2Value = POT2_LIMIT_MIN;
      else if(pot2Value > POT2_LIMIT_MAX) pot2Value = POT2_LIMIT_MAX;
      //Serial.print(pot1Value); Serial.print(" "); Serial.println(pot2Value);
      unsigned int button1 = *(infoUIntPointer++);
      unsigned int button2 = *(infoUIntPointer++);
      if(button1 == 0) reset = true;
      if(button2 == 0)
      {
        motorsOn = !motorsOn;
      }
      

      infoPointer = (float *) infoUIntPointer;
      float footer = *(infoPointer);

      uint8_t checksum = calculateChecksum(infoBuffer);

      if(checksum != (uint8_t) footer)
      {
        //Serial.println("Bad packet!");
        return;
      } //else Serial.println("Good");

      
      //Serial.println(pot1Value);
      calculatePID(pitch, roll, throttle, yaw);

      if (motorsOn)
      {
        analogWrite(MOTOR_FR, throttleFR); //red
        analogWrite(MOTOR_BL, throttleBL); //white
        analogWrite(MOTOR_BR, throttleBR); //black
        analogWrite(MOTOR_FL, throttleFL); //green
      }
      else
      {
        analogWrite(MOTOR_FR, 0); //red
        analogWrite(MOTOR_BL, 0); //white
        analogWrite(MOTOR_BR, 0); //black
        analogWrite(MOTOR_FL, 0); //green
      }
    }
  }
}
