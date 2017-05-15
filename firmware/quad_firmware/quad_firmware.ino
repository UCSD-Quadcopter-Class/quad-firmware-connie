#include <radio.h>
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


unsigned char infoBuffer [INFO_SIZE]; 

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

const float KP = 0.5;
const float KI = 0.3;
const float KD = 0.18;
float error = 0;
float errorSum = 0;
float lastError = 0;


unsigned long lastTime = 0;

const float OFFSET_FR = 0;//-28.0;//18;
const float OFFSET_BL = 0;//3;//18;
const float OFFSET_BR = 0;//0;//28;
const float OFFSET_FL = 0;//-20  ;//12;

const float PROPORTIONAL_FR = 1;
const float PROPORTIONAL_BL = 1;
const float PROPORTIONAL_BR = 1;
const float PROPORTIONAL_FL = 1;

unsigned int throttleFR = 0;
unsigned int throttleBL = 0;
unsigned int throttleFL = 0;
unsigned int throttleBR = 0;
float pitchPID = 0;
float rollPID = 0;

float filteredPitch = 0;
float pitchReadingAverage = 0;
float rollReadingAverage = 0;
float gyroReadingAverage = 0;

const int WINDOW_SIZE = 2;
int windowCounter = 0;
float pitchReadings [WINDOW_SIZE];
float rollReadings [WINDOW_SIZE];
float gyroReadings [WINDOW_SIZE];


float pitchOffset = 0;
float rollOffset = 0;

float pot1Value = 0;
float pot2Value=0;
unsigned int button1Value = 1; //reset
unsigned int button2Value = 1; //on/off

const float POT1_MIN = 115;
const float POT1_MAX = 816;
const float POT1_LIMIT_MIN = 0.2;
const float POT1_LIMIT_MAX = 1.2;
const float POT1_COEFFICIENT = (POT1_LIMIT_MAX - POT1_LIMIT_MIN)/(POT1_MAX - POT1_MIN);
const float POT1_CONSTANT = POT1_LIMIT_MAX - POT1_MAX * POT1_COEFFICIENT;


const float POT2_MIN = 0;
const float POT2_MAX = 1003;
const float POT2_LIMIT_MIN = -0.5;
const float POT2_LIMIT_MAX = 3;
const float POT2_COEFFICIENT = (POT2_LIMIT_MAX - POT2_LIMIT_MIN)/(POT2_MAX - POT2_MIN);
const float POT2_CONSTANT = POT2_LIMIT_MAX - POT2_MAX * POT2_COEFFICIENT;

float compCoefficient = 0;
const float COMP_COEFFICIENT = 0.97;


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
  calibrateSensor();
  
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
  float pitchSum = 0;
  float rollSum = 0;

  for(int i = 0; i < 10; i++)
  {
    pitchSum = 0;
    rollSum = 0;
    
    for(int j = 0; j < 10; j++)
    {
      if (ahrs.getOrientation(&orientation))
      {
        pitchSum += orientation.pitch;
        rollSum += orientation.roll;
      }
    }

    pitchValue += pitchSum / 10.0;
    rollValue += rollSum / 10.0;
  }

  pitchOffset = pitchValue / 10.0;
  rollOffset = rollValue / 10.0;
  Serial.println("Calibration complete.");
}

float normalizePitch(float pitchValue)
{
  if (pitchValue < PITCH_MIN) pitchValue = PITCH_MIN;
  else if (pitchValue > PITCH_MAX) pitchValue = PITCH_MAX;
  return pitchValue / (PITCH_MAX/(pitchMax - pitchMin)) - (pitchMax - pitchMin)/2.0 + PITCH_OFFSET;
}

void calculatePID(float pitch, float roll, float throttle, float yaw)
{
  if (lastError < -200 || lastError > 200) lastError = 0;
  float timeChange = ((float)abs((millis() - lastTime)))/1000.0;

  if (pidCalculationStarted)
  {
    filteredPitch = constrain(COMP_COEFFICIENT * (filteredPitch - gyroReadingAverage * timeChange) + (1.0 - COMP_COEFFICIENT) * pitchReadingAverage, -180.0, 180.0); 
  }
  else
  {
    filteredPitch = pitchReadingAverage;
    pidCalculationStarted = true;
  }

  error = normalizePitch(pitch) - filteredPitch;
  errorSum *= 0.9999; //decay errorSum
  errorSum += error * timeChange;
  errorSum = constrain(errorSum, -200.0, 200.0); //bound errorSum
  
  float dError = (error -  lastError) / timeChange;
  
  //Serial.print(error); Serial.print(" "); Serial.print(errorSum); Serial.print(" "); Serial.println(pot2Value);
  pitchPID = KP * error + KI * errorSum + KD * dError;
  lastError = error;
  lastTime = millis();

  throttleBL = limitThrottle((throttle + pitchPID + rollPID) * PROPORTIONAL_BL + OFFSET_BL);
  throttleBR = limitThrottle((throttle + pitchPID - rollPID) * PROPORTIONAL_BR + OFFSET_BR);
  throttleFL = limitThrottle((throttle - pitchPID + rollPID) * PROPORTIONAL_FL + OFFSET_FL);
  throttleFR = limitThrottle((throttle - pitchPID - rollPID) * PROPORTIONAL_FR + OFFSET_FR);
}

int limitThrottle(float throttleValue)
{
  return constrain((int)throttleValue, 0, 255);
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
  }
  
  pitchReadingAverage -= pitchOffset;
  rollReadingAverage -= rollOffset;
}

void resetAndRecalibrate()
{
  errorSum = 0;
  lastError = 0;
  calibrateSensor();
  filteredPitch = pitchReadingAverage;
  pidCalculationStarted = false;
  reset = false;  
}

void loop()
{  
  calculateReadingAverages();
  
  if (reset)
  {
    resetAndRecalibrate();
  }


  if (rfAvailable())
  {
    int numBytesAvailable = rfAvailable(); //behaves weirdly if you use rfAvailable return value directly
    
    if (numBytesAvailable >= INFO_SIZE)
    {
      rfRead(infoBuffer, INFO_SIZE);
      flightControlInfo info = *((flightControlInfo *) infoBuffer);
     
      if(info.header != HEADER)
      {
        Serial.println("Bad Header!");
        return;
      }

      if(calculateChecksum(infoBuffer) != info.footer)
      {
        Serial.println("Bad checksum!");
        return;
      }
      
      float pitch = info.pitch;
      float roll = info.roll;
      float throttle = constrain(info.throttle, 0.0, 1024.0);

      throttle = throttle/THROTTLE_MAX * THROTTLE_COEFFICIENT;
      
      float yaw = info.yaw;
      pot1Value = float(info.pot1) * POT1_COEFFICIENT + POT1_CONSTANT;
      if (pot1Value < POT1_LIMIT_MIN) pot1Value = POT1_LIMIT_MIN;
      else if(pot1Value > POT1_LIMIT_MAX) pot1Value = POT1_LIMIT_MAX;
     
      pot2Value = float(info.pot2) * POT2_COEFFICIENT + POT2_CONSTANT;
      if (pot2Value < POT2_LIMIT_MIN) pot2Value = POT2_LIMIT_MIN;
      else if(pot2Value > POT2_LIMIT_MAX) pot2Value = POT2_LIMIT_MAX;

      if(info.button1 == 0) reset = true;
      if(info.button2 == 0)
      {
        motorsOn = !motorsOn;
      }
      
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
