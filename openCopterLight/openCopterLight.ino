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
 This example code will only work on the MEGA
 To use on a different arduino change the slave select defines or use digitalWrite 
 */
 
//*****************************************************************************************************************//
//CALIBRATION OF THE ELECTRONIC SPEED CONTROLLERS AND THE ACCELEROMETER MUST BE COMPLETED BEFORE ATTEMPTING TO FLY!//
//*****************************************************************************************************************//
#include <I2C.h>
/*
http://dsscircuits.com/articles/arduino-i2c-master-library.html
 */
#include "openIMUL.h"
#include "MPIDL.h"//the L is for local in case there is already a library by that name
#include "openCopterLight.h"


//accelerometer calibration values
//this is where the values from the accelerometer calibration sketch belong
#define ACC_OFFSET_X  2.3413698f
#define ACC_OFFSET_Y -0.4740458f
#define ACC_OFFSET_Z 26.5365638f
#define ACC_SCALE_X 0.0370868f
#define ACC_SCALE_Y 0.0373058f
#define ACC_SCALE_Z 0.0391581f

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

//RC signal variables
uint8_t rcType,readState,inByte;
boolean detected = false;
boolean newRC = false;
int bufferIndex=0;
//this is used to get the DSM2/DSMx channels in the right order
uint8_t syncArray1[14] = {
  1,0,11,10,3,2,9,8,7,6,13,12,5,4};
uint8_t syncArray2[14] = {
  1,0,11,10,9,8,3,2,13,12,5,4,7,6};

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
float dt;

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
//gains for the PID loops
float kp_r_p = 0.65;
float ki_r_p = 0.05;
float kd_r_p = 0.01423;
float nPitch = 19.5924;

float kp_r_r = 0.65;
float ki_r_r = 0.05;
float kd_r_r = 0.01423;
float nRoll = 19.5924;

float kp_r_y = 6.9389;
float ki_r_y = 0.22747;
float kd_r_y = -0.42597;
float nYaw = 4.4174;

float kp_a_p = 4.6527;
float ki_a_p = 0.2005;
float kd_a_p = 0.11256;
float nPitchA = 47.9596;

float kp_a_r = 4.6527;
float ki_a_r = 0.2005;
float kd_a_r = 0.11256;
float nRollA = 47.9596;

//general purpose index variables
uint16_t i;
uint16_t j;
uint8_t k;
uint32_t timer;

//set up the attitude estimator 
openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&dt);
//set up the PID loops
MPID PitchAngle(&pitchSetPoint,&imu.pitch,&rateSetPointY,&integrate,&kp_a_p,&ki_a_p,&kd_a_p,&nPitchA,&dt,500,500);
MPID RollAngle(&rollSetPoint,&imu.roll,&rateSetPointX,&integrate,&kp_a_r,&ki_a_r,&kd_a_r,&nRollA,&dt,500,500);
MPID PitchRate(&rateSetPointY,&degreeGyroY,&adjustmentY,&integrate,&kp_r_p,&ki_r_p,&kd_r_p,&nPitch,&dt,500,500);
MPID RollRate(&rateSetPointX,&degreeGyroX,&adjustmentX,&integrate,&kp_r_r,&ki_r_r,&kd_r_r,&nRoll,&dt,500,500);
MPID YawRate(&rateSetPointZ,&degreeGyroZ,&adjustmentZ,&integrate,&kp_r_y,&ki_r_y,&kd_r_y,&nYaw,&dt,500,500);

//saftey related variables
boolean failSafe = false;
boolean hold = true;
boolean toggle;
long failSafeTimer;


void setup(){
  pinMode(RED,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  pinMode(GREEN,OUTPUT);
  digitalWrite(YELLOW,HIGH);
  digitalWrite(RED,HIGH);
  MotorInit();//start the motor signals
  DetectRC();
  Arm();//move the rudder to the right to begin calibration
  I2c.begin();
  I2c.setSpeed(1);
  GyroInit();
  AccInit();
  LevelAngles();
  Reset();
  SafetyCheck();
  digitalWrite(YELLOW,LOW);
  digitalWrite(RED,HIGH);
  digitalWrite(GREEN,HIGH);
  failSafeTimer = millis();
  timer = micros();
}

void loop(){
  if (micros() - timer > 2500){//~400 hz  
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
    failSafeTimer = millis();
    ProcessChannels();
  }  
  if (millis() - failSafeTimer > 1000){
    failSafe = true;
  }
  if (failSafe == true ){
    Motor1WriteMicros(1000);//set the output compare value
    Motor2WriteMicros(1000);
    Motor3WriteMicros(1000);
    Motor4WriteMicros(1000);
    digitalWrite(GREEN,LOW);
    while(1){
      digitalWrite(RED,HIGH);
      delay(500);
      digitalWrite(RED,LOW);
      delay(500);
    }
  }
}

//simple low pass filter
void Smoothing(int16_t *raw, float *smooth){
  *smooth = (*raw * (0.10)) + (*smooth * 0.90);
}
//mapping function
void MapVar (uint16_t *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}








































