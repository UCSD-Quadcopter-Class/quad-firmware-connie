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
bool motorsOn = false;
bool pidCalculationStarted = false;

float pitchMax = 90; //approx
float pitchMin = -90; //approx

float rollMax = 90; //approx
float rollMin = -90; //approx

float yawMax = 360;
float yawMin = 0;
float yawMid = yawMax/2.0;




unsigned char infoBuffer [INFO_SIZE]; 

const int MOTOR_FR = 3; //green short
const int MOTOR_BL = 4; //green long
const int MOTOR_BR = 5; //yellow short
const int MOTOR_FL = 8; //yellow long


const float THROTTLE_MAX = 1024;
const float THROTTLE_COEFFICIENT = 255;

const float YAW_MAX = 1315;
const float YAW_MIN = 0;
const float YAW_MID = 612; //(YAW_MAX - YAW_MIN) / 2.0;
const float ROLL_MAX = 1136;
const float ROLL_MIN = 10;
const float ROLL_MID = 627.09; //(ROLL_MAX - ROLL_MIN) / 2.0;

const float PITCH_MAX = 1224; //1390 actual max
const float PITCH_MIN = 0;
const float PITCH_MID = (PITCH_MAX - PITCH_MIN) / 2.0;

const float PITCH_OFFSET = (pitchMax - pitchMin) / 2.0 - PITCH_MID / ((PITCH_MAX - PITCH_MIN) / (pitchMax - pitchMin)) - 0.15;
const float ROLL_OFFSET = (rollMax - rollMin) / 2.0 - ROLL_MID / ((ROLL_MAX - ROLL_MIN) / (rollMax - rollMin)) + 0.44;
const float YAW_OFFSET = (yawMax - yawMin) / 2.0 - YAW_MID / ((YAW_MAX - YAW_MIN) / (yawMax - yawMin));

const float PITCH_KP = 6.5; //0.5;
const float PITCH_KI = 3.22;
const float PITCH_KD = 1.15;// 0.18;

const float ROLL_KP = 6.5;
const float ROLL_KI = 3.22;
const float ROLL_KD = 1.15;

const float YAW_KP = 0;
const float YAW_KI = 0;
const float YAW_KD = 0;


float pitchError = 0;
float pitchErrorSum = 0;
float lastPitchError = 0;
float rollError = 0;
float lastRollError = 0;
float rollErrorSum = 0;
float yawError = 0;
float yawErrorSum = 0;
float lastYawError = 0;

const unsigned long TIMEOUT = 5 * 1000;
unsigned long lastTime = 0;
unsigned long lastValidPacketTime = 0;

const float OFFSET_FR = -5;//-28.0;//18;
const float OFFSET_BL = 0;//3;//18;
const float OFFSET_BR = 0;//0;//28;
const float OFFSET_FL = 0;//-20  ;//12;

const float PROPORTIONAL_FR = 0.9;
const float PROPORTIONAL_BL = 1;
const float PROPORTIONAL_BR = 1;
const float PROPORTIONAL_FL = 1;

unsigned int throttleFR = 0;
unsigned int throttleBL = 0;
unsigned int throttleFL = 0;
unsigned int throttleBR = 0;
float pitchPID = 0;
float rollPID = 0;
float yawPID = 0;

float filteredPitch = 0;
float filteredRoll = 0;
float filteredYaw = 0;
float pitchReadingAverage = 0;
float rollReadingAverage = 0;
float gyroYReadingAverage = 0;
float gyroXReadingAverage = 0;
float gyroZReadingAverage = 0;
float yawReadingAverage = 0;

const int WINDOW_SIZE = 1;
int windowCounter = 0;
float pitchReadings [WINDOW_SIZE];
float rollReadings [WINDOW_SIZE];
float gyroYReadings [WINDOW_SIZE];
float gyroXReadings [WINDOW_SIZE];
float gyroZReadings [WINDOW_SIZE];
float yawReadings [WINDOW_SIZE];

float pitch = 0;
float throttle = 0;
float roll = 0;
float yaw = 0;

float pitchOffset = 0;
float rollOffset = 0;
float yawOffset = 0;

float pot1Value = 0;
float pot2Value=0;
unsigned int button1Value = 1; //reset
unsigned int button2Value = 1; //on/off

const float POT1_MIN = 115;
const float POT1_MAX = 816;
const float POT1_LIMIT_MIN = 4;
const float POT1_LIMIT_MAX = 8;
const float POT1_COEFFICIENT = (POT1_LIMIT_MAX - POT1_LIMIT_MIN)/(POT1_MAX - POT1_MIN);
const float POT1_CONSTANT = POT1_LIMIT_MAX - POT1_MAX * POT1_COEFFICIENT;


const float POT2_MIN = 0;
const float POT2_MAX = 1003;
const float POT2_LIMIT_MIN = 0.2;
const float POT2_LIMIT_MAX = 2.5;
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
    gyroYReadings[i] = 0;
    gyroXReadings[i] = 0;
    gyroZReadings[i] = 0;
    yawReadings[i] = 0;
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

float normalizeRoll(float rollValue)
{
  if (rollValue < ROLL_MIN) rollValue = ROLL_MIN;
  else if (rollValue > ROLL_MAX) rollValue = ROLL_MAX;
  return rollValue / (ROLL_MAX/(rollMax - rollMin)) - (rollMax - rollMin)/2.0 + ROLL_OFFSET;
}

float normalizeYaw(float yawValue)
{
  if(yawValue < YAW_MIN) yawValue = YAW_MIN;
  else if (yawValue > YAW_MAX) yawValue = YAW_MAX;
  return yawValue / (YAW_MAX/(yawMax - yawMin)) - (yawMax - yawMin)/2.0 + YAW_OFFSET;
}

void calculatePID(float pitch, float roll, float throttle, float yaw)
{
  if (lastPitchError < -200 || lastPitchError > 200) lastPitchError = 0;
  if (lastRollError < -200 || lastRollError > 200) lastRollError = 0;
  if(lastYawError < -200 || lastYawError > 200) lastYawError = 0;
  float timeChange = ((float)abs((millis() - lastTime)))/1000.0;
  //Serial.print(normalizePitch(pitch)); Serial.print("\t"); Serial.print(normalizeRoll(roll)); Serial.print("\t"); Serial.println(normalizeYaw(yaw));

  if (pidCalculationStarted)
  {
    //Serial.print(gyroYReadingAverage); Serial.print("\t");
    //Serial.println(yawReadingAverage); //Serial.print("\t");
    //Serial.println(gyroZReadingAverage);
    filteredPitch = constrain(COMP_COEFFICIENT * (filteredPitch - gyroYReadingAverage * timeChange) + (1.0 - COMP_COEFFICIENT) * pitchReadingAverage, -180.0, 180.0); 
    filteredRoll = constrain(COMP_COEFFICIENT * (filteredRoll - gyroXReadingAverage * timeChange) + (1.0 - COMP_COEFFICIENT) * rollReadingAverage, -180.0, 180.0); //TODO
    filteredYaw = constrain(COMP_COEFFICIENT * (filteredYaw - gyroZReadingAverage * timeChange) + (1.0 - COMP_COEFFICIENT) * yawReadingAverage, 0, 360.0);
  }
  else
  {
    filteredPitch = pitchReadingAverage;
    filteredRoll = rollReadingAverage;
    filteredYaw = yawReadingAverage;
    pidCalculationStarted = true;
  }

  pitchError = normalizePitch(pitch) - filteredPitch;
  rollError = normalizeRoll(roll) - filteredRoll;
  yawError = normalizeYaw(yaw) - filteredYaw;

  float pitchDError = (pitchError -  lastPitchError) / timeChange;
  float rollDError = (rollError - lastRollError) / timeChange;
  float yawDError = (yawError - lastYawError) / timeChange;
  if ((abs(pitchErrorSum) > 20)&& (abs(pitchErrorSum + pitchDError) < abs(pitchDError)) && (abs(pitchDError) > 450.0)) {pitchErrorSum *= 0.999998;}
  else if (abs(pitchErrorSum + pitchError) < abs(pitchErrorSum)) pitchErrorSum *= 0.9999999;
  
  pitchErrorSum *= 0.99999999; //decay errorSum
  pitchErrorSum += pitchError * timeChange;
  pitchErrorSum = constrain(pitchErrorSum, -10000.0, 10000.0); //bound errorSum

  if ((abs(rollErrorSum) > 20)&& (abs(rollErrorSum + rollDError) < abs(rollDError)) && (abs(rollDError) > 450.0)) {rollErrorSum *= 0.999998;}
  else if (abs(rollErrorSum + rollError) < abs(rollErrorSum)) rollErrorSum *= 0.9999999;
  else rollErrorSum *= 0.99999999; //decay errorSum
  rollErrorSum += rollError * timeChange;
  rollErrorSum = constrain(rollErrorSum, -10000.0, 10000.0); //bound errorSum

  if ((abs(yawErrorSum) > 10)&& (abs(yawErrorSum + yawDError) < abs(yawDError)) && (abs(yawDError) > 150.0)) {yawErrorSum *= 0.2;}
  else if (abs(yawErrorSum + yawError) < abs(yawErrorSum)) yawErrorSum *= 0.996;
  else yawErrorSum *= 0.999; //decay errorSum
  yawErrorSum += yawError * timeChange;
  yawErrorSum = constrain(yawErrorSum, -150.0, 150.0); //bound errorSum
  
  
 //float tempPITCH_KP = pot1Value;//PITCH_KP;
 //float tempPITCH_KD = pot2Value;
 //float tempPITCH_KI = pot1Value;

  //float tempROLL_KP = pot1Value;//PITCH_KP;
  //float tempROLL_KD = pot2Value;
  //float tempROLL_KI = pot1Value;

  pitchPID = PITCH_KP * pitchError + PITCH_KI * pitchErrorSum + PITCH_KD * pitchDError;
  rollPID = ROLL_KP * rollError + ROLL_KI * rollErrorSum + ROLL_KD * rollDError;
  yawPID = YAW_KP * yawError + YAW_KI * yawErrorSum + YAW_KD * yawDError;
   //Serial.print(error); Serial.print("\t"); Serial.print(PITCH_KP*error); Serial.print("\t"); Serial.print(PITCH_KD*dError); Serial.print("\t"); Serial.println(pitchPID);
  lastPitchError = pitchError;
  lastRollError = rollError;
  lastYawError = yawError;
  lastTime = millis();

  throttleBL = limitThrottle((throttle - pitchPID /*+ rollPID*/) * PROPORTIONAL_BL + OFFSET_BL);   //BACK
  throttleBR = limitThrottle((throttle /*- pitchPID*/ - rollPID) * PROPORTIONAL_BR + OFFSET_BR);
  throttleFL = limitThrottle((throttle /*+ pitchPID*/ + rollPID) * PROPORTIONAL_FL + OFFSET_FL);
 
  throttleFR = limitThrottle((throttle + pitchPID /*- rollPID*/) * PROPORTIONAL_FR + OFFSET_FR);  //FRONT
   //Serial.print(limitThrottle((throttle + pitchPID /*- rollPID*/))); Serial.print("\t");
  //Serial.println(throttleFR);
}

int limitThrottle(float throttleValue)
{ 
  if(throttleValue < 0.0) throttleValue = 0;
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
        gyroYReadings[windowCounter] = orientation.gyro_y;  
        gyroXReadings[windowCounter] = orientation.gyro_x; 
        gyroZReadings[windowCounter] = orientation.gyro_z; 
        yawReadings[windowCounter] = orientation.heading;
        if (windowCounter == 10) windowCounter = 0;
        else windowCounter++;  
      }
    }
  }

  if (ahrs.getOrientation(&orientation))
  {
    pitchReadings[windowCounter] = orientation.pitch;
    rollReadings[windowCounter] = orientation.roll;
    gyroYReadings[windowCounter] = orientation.gyro_y;
    gyroXReadings[windowCounter] = orientation.gyro_x;
    gyroZReadings[windowCounter] = orientation.gyro_z;
    yawReadings[windowCounter] = orientation.heading;
    pitchReadingAverage = 0;
    rollReadingAverage = 0;
    gyroYReadingAverage = 0;
    gyroXReadingAverage = 0;
    gyroZReadingAverage = 0;
    yawReadingAverage = 0;

    for(int i = 0; i < WINDOW_SIZE; i++)
    {
      pitchReadingAverage += pitchReadings[i];
      rollReadingAverage += rollReadings[i];
      gyroYReadingAverage += gyroYReadings[i];
      gyroXReadingAverage += gyroXReadings[i];
      gyroZReadingAverage += gyroZReadings[i];
      yawReadingAverage += yawReadings[i];
    }

    pitchReadingAverage /= float(WINDOW_SIZE);
    rollReadingAverage /= float(WINDOW_SIZE);
    gyroYReadingAverage /= float(WINDOW_SIZE);
    gyroXReadingAverage /= float(WINDOW_SIZE);
    gyroZReadingAverage /= float(WINDOW_SIZE);
    yawReadingAverage /= float(WINDOW_SIZE);

    if (windowCounter == WINDOW_SIZE) windowCounter = 0;
    else windowCounter++;  
  }
  
  pitchReadingAverage -= pitchOffset;
  rollReadingAverage -= rollOffset;
  yawReadingAverage -= yawOffset;
}

void resetAndRecalibrate()
{
  pitchErrorSum = 0;
  rollErrorSum = 0;
  yawErrorSum = 0;
  lastPitchError = 0;
  lastRollError = 0;
  lastYawError = 0;
  calibrateSensor();
  filteredPitch = pitchReadingAverage;
  filteredRoll = rollReadingAverage;
  filteredYaw = yawReadingAverage;
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
      bool validPacket = true;
      rfRead(infoBuffer, INFO_SIZE);
      flightControlInfo info = *((flightControlInfo *) infoBuffer);
     
      if(info.header != HEADER)
      {
        Serial.println("Bad Header!");
        validPacket = false;
      }

      if(validPacket && (calculateChecksum(infoBuffer) != info.footer))
      {
        Serial.println("Bad checksum!");
        validPacket = false;
      }

      if(validPacket)
      {      
        pitch = info.pitch;
        roll = info.roll;
        throttle = constrain(info.throttle, 0.0, 1024.0);
  
        throttle = throttle/THROTTLE_MAX * THROTTLE_COEFFICIENT;
        
        yaw = info.yaw;
        //Serial.print(info.pitch); Serial.print("\t"); Serial.print(info.roll); Serial.print("\t"); Serial.println(info.yaw);
        pot1Value = float(info.pot1) * POT1_COEFFICIENT + POT1_CONSTANT;
        //Serial.print(pot1Value); Serial.print(" "); Serial.println(pot2Value);
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
        
        lastValidPacketTime = millis();
      }
    }
  }

  long timeSinceLastValidPacket = millis() - lastValidPacketTime;
  if(timeSinceLastValidPacket > TIMEOUT)
  {
    motorsOn = false;
  }
  
  calculatePID(pitch, roll, throttle, yaw);

  if (motorsOn)
  {
    analogWrite(MOTOR_FR, throttleFR); //red //FRONT
    analogWrite(MOTOR_BL, throttleBL); //white //BACK
    analogWrite(MOTOR_BR, throttleBR); //black //RIGHT
    analogWrite(MOTOR_FL, throttleFL); //green  //LEFT
  }
  else
  {
    analogWrite(MOTOR_FR, 0); //red
    analogWrite(MOTOR_BL, 0); //white
    analogWrite(MOTOR_BR, 0); //black
    analogWrite(MOTOR_FL, 0); //green
  }
}
