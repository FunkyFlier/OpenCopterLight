/*openCopter
 An open source flight controller based on the openIMU
 The sensors used in this example are the ADXL345, the L3G3200, the LSM303DLHC, and the BMP085
 If the device is moved during startup it will not function properly. 
 
 Copyright (C) 2013  Michael Baker
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 
 Gyro must be in RAD/s
 Sensors must be in the North East Down convention
 To use on a different arduino change the slave select defines or use digitalWrite 
 */

//*****************************************************************************************************************//
//CALIBRATION OF THE ELECTRONIC SPEED CONTROLLERS AND THE ACCELEROMETER MUST BE COMPLETED BEFORE ATTEMPTING TO FLY!//
//*****************************************************************************************************************//
#include <EEPROM.h>
#include <I2C.h>
/*
http://dsscircuits.com/articles/arduino-i2c-master-library.html
 */
#include "openIMUL.h"
#include "MPIDL.h"//the L is for local in case there is already a library by that name
#include "openCopterLight.h"
#include <Streaming.h>

//sensor data 
typedef union{
  struct{
    int16_t x;
    int16_t y;
    int16_t z;
  }
  v;
  uint8_t buffer[6];
}
Sensor_t;
Sensor_t gyro;
Sensor_t acc;

typedef union{
  struct{
    float ACC_OFFSET_X;
    float ACC_OFFSET_Y;
    float ACC_OFFSET_Z;
    float ACC_SCALE_X;
    float ACC_SCALE_Y;
    float ACC_SCALE_Z;
  }
  v;
  uint8_t buffer[24];
}
Calibration_t;

Calibration_t accCal;

//RC signal variables
uint8_t rcType,readState,inByte,byteCount,channelNumber;
uint32_t frameTime;
boolean detected = false;
boolean newRC = false;
int bufferIndex=0;
uint8_t spekBuffer[14];


typedef union{
  struct{
    uint16_t aileron;
    uint16_t elevator;
    uint16_t throttle;
    uint16_t rudder;
    uint16_t gear;
    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
  }
  values;
  byte buffer[16];
  uint16_t standardRCBuffer[8];
}
RadioControl_t;
RadioControl_t rcCommands;

uint8_t currentPinState = 0;
uint8_t previousPinState = 0;
uint8_t changeMask = 0;
uint8_t lastPinState = 0;
uint16_t currentTime = 0;
uint16_t timeDifference = 0;
uint16_t changeTime[8];
int16_t offset = 0;
uint8_t sBusData[25];

//for the calibration of the gyro
int32_t gyroSumX,gyroSumY,gyroSumZ;
int16_t offsetX,offsetY,offsetZ;

//IMU related vars
float radianGyroX,radianGyroY,radianGyroZ;
float degreeGyroX,degreeGyroY,degreeGyroZ;
float smoothAccX,smoothAccY,smoothAccZ;
float accToFilterX,accToFilterY,accToFilterZ;
//float dt;

//control related vars
boolean integrate = false;
uint16_t motorCommand1,motorCommand2,motorCommand3,motorCommand4;

float pitchSetPoint;
float rollSetPoint;
float pitchAngle;
float rollAngle;
float rateSetPointX;    
float rateSetPointY;
float rateSetPointZ;
float adjustmentX;
float adjustmentY;
float adjustmentZ; 

float kp_r_p = 0.4125;
float ki_r_p = 2.9049296;
float kd_r_p = 0.03905;
float nPitch = 50.0;

float kp_r_r = 0.4125;
float ki_r_r = 2.9049296;
float kd_r_r = 0.03905;
float nRoll = 50.0;

float kp_r_y = 2.25;
float ki_r_y = 0.25;
float kd_r_y = 0.01;
float nYaw = 50.0;

float kp_a_p = 5.35;
float ki_a_p = 0;
float kd_a_p = 0.075;
float nPitchA = 75;

float kp_a_r = 5.35;
float ki_a_r = 0;
float kd_a_r = 0.075;
float nRollA = 75;


//general purpose index variables
uint16_t i;
uint16_t j;
uint8_t k;
uint32_t timer;

float dt;

//set up the attitude estimator 
openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&dt);
//set up the PID loops
MPID PitchAngle(&pitchSetPoint,&imu.pitch,&rateSetPointY,&integrate,&kp_a_p,&ki_a_p,&kd_a_p,&nPitchA,&dt,500,500);
MPID RollAngle(&rollSetPoint,&imu.roll,&rateSetPointX,&integrate,&kp_a_r,&ki_a_r,&kd_a_r,&nRollA,&dt,500,500);
MPID PitchRate(&rateSetPointY,&degreeGyroY,&adjustmentY,&integrate,&kp_r_p,&ki_r_p,&kd_r_p,&nPitch,&dt,500,500);
MPID RollRate(&rateSetPointX,&degreeGyroX,&adjustmentX,&integrate,&kp_r_r,&ki_r_r,&kd_r_r,&nRoll,&dt,500,500);
MPID YawRate(&rateSetPointZ,&degreeGyroZ,&adjustmentZ,&integrate,&kp_r_y,&ki_r_y,&kd_r_y,&nYaw,&dt,500,500);

boolean watchDogStartCount = false;
uint8_t RCFailSafeCounter = 0, watchDogFailSafeCounter = 0;


//saftey related variables
boolean failSafe = false;
boolean hold = true;
boolean toggle;

long printTimer;

void setup(){
  Serial.begin(115200);
  pinMode(RED,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  pinMode(GREEN,OUTPUT);
  digitalWrite(YELLOW,HIGH);
  digitalWrite(RED,HIGH);
  GetCalibrationValues();
  MotorInit();//start the motor signals
  DetectRC();
  
  Arm();//move the rudder to the right to begin calibration
  I2c.begin();
  I2c.setSpeed(1);
  GyroInit();
  AccInit();
  Reset();
  SafetyCheck();
  digitalWrite(YELLOW,LOW);
  digitalWrite(RED,HIGH);
  digitalWrite(GREEN,HIGH);

  ISRConfig();
  timer = micros();
  watchDogStartCount = true;
  printTimer = millis();
}

void loop(){

  if (micros() - timer > 5262){//~190 hz  
    dt = ((micros() - timer) / 1000000.0);
    timer = micros();
    GetGyro();
    GetAcc();
    imu.IMUupdate();
    if (rcCommands.values.gear < 1500){
      //the gear channel toggles stunt mode
      //stunt mode is much more difficult to fly in than normal mode
      imu.GetEuler();
      Angle();
      digitalWrite(YELLOW,LOW);
    }
    else{
      digitalWrite(YELLOW,HIGH);
    }
    Rate();
    MotorHandler();
  }
  if (rcType != RC){
    FeedLine();
  }
  if (newRC == true){
    newRC = false;
    ProcessChannels();
    RCFailSafeCounter = 0;

  }  
  /*if ((millis() - printTimer >= 50) ){
    printTimer = millis();
    Serial<<rollSetPoint<<","<<pitchSetPoint<<","<<rateSetPointZ<<"\r\n";
  }*/

  if (RCFailSafeCounter >= 190 || failSafe == true ){
    TIMSK4 = (0<<OCIE4A);
    Motor1WriteMicros(1000);
    Motor2WriteMicros(1000);
    Motor3WriteMicros(1000);
    Motor4WriteMicros(1000);
    digitalWrite(GREEN,LOW);
    while(1){
      FeedLine();
      digitalWrite(YELLOW,HIGH);
      delay(500);
      digitalWrite(YELLOW,LOW);
      delay(500);
    }
  }
  watchDogFailSafeCounter = 0;
}

//simple low pass filter
void Smoothing(int16_t *raw, float *smooth){
  *smooth = (*raw * (0.15)) + (*smooth * 0.85);
}
//mapping function
void MapVar (uint16_t *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}









































