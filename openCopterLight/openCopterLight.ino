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
#include <I2C.h>
#include "openIMUL.h"
#include "MPIDL.h"//the L is for local in case there is already a library by that name
#include <Streaming.h>

#define LIFTOFF 1175 
#define TRANSMIT_TIME 20
#define radio Serial1


//LED defines
#define RED 13
#define YELLOW 6
#define GREEN 4

//gyro defines
#define L3GD20_ADDRESS    0x6A
#define L3G_WHO_AM_I      0x0F

#define L3G_CTRL_REG1     0x20
#define L3G_CTRL_REG2     0x21
#define L3G_CTRL_REG3     0x22
#define L3G_CTRL_REG4     0x23
#define L3G_CTRL_REG5     0x24
#define L3G_REFERENCE     0x25
#define L3G_OUT_TEMP      0x26
#define L3G_STATUS_REG    0x27

#define L3G_OUT_X_L       0x28
#define L3G_OUT_X_H       0x29
#define L3G_OUT_Y_L       0x2A
#define L3G_OUT_Y_H       0x2B
#define L3G_OUT_Z_L       0x2C
#define L3G_OUT_Z_H       0x2D

//acc defines - Analog Devices ADXL345
#define ADXL435_ADDR 0x53

#define OFSX 0x1E //15.6 mg/LSB (that is, 0x7F = +2 g)
#define OFSY 0x1F
#define OFSZ 0x20

#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31

#define DATAX0 0x32
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37
#define FIFO_CTL 0x38


//RC defines
#define DSM2 0
#define DSMX 1
#define SBUS 2
#define RC 3
#define HEX_ZERO 0x00


//motor defines
#define FREQ 400
#define PRESCALE 8
#define PERIOD ((F_CPU/PRESCALE/FREQ) - 1)

#define Motor1WriteMicros(x) OCR3A = x * 2
#define Motor2WriteMicros(x) OCR1B = x * 2
#define Motor3WriteMicros(x) OCR1A = x * 2
#define Motor4WriteMicros(x) OCR1C = x * 2



#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f

#define DebugOutput() DDRD |= 1<<3
#define DebugHigh() PORTD |= 1<<3
#define DebugLow() PORTD &= ~(1<<3)

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

//RC vars
uint8_t rcType,readState,inByte;
boolean detected = false;
boolean newRC = false;


uint8_t syncArray1[14] = {
  1,0,11,10,3,2,9,8,7,6,13,12,5,4};
uint8_t syncArray2[14] = {
  1,0,11,10,9,8,3,2,13,12,5,4,7,6};

int bufferIndex=0;

typedef union{
  struct{
    uint16_t aileron;//A8
    uint16_t elevator;//A10
    uint16_t throttle;//A14
    uint16_t rudder;//A12
    uint16_t gear;//A11
    uint16_t aux1;//A9
    uint16_t aux2;//A13
    uint16_t aux3;//A15 only for futaba or standard RC
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
//float degreeGyroX,degreeGyroY,degreeGyroZ;
float smoothAccX,smoothAccY,smoothAccZ;
float accToFilterX,accToFilterY,accToFilterZ;
float dt;

//control related vars
boolean integrate = false;
boolean gainToggle = true;
uint16_t motorCommand1,motorCommand2,motorCommand3,motorCommand4;


typedef union {
  struct{
    long timestamp;
    float pitchSetPoint;
    float pitchAngle;
    float rollSetPoint;
    float rollAngle;
    float degreeGyroX;
    float degreeGyroY;
    float degreeGyroZ;
    float rateSetPointX;    
    float rateSetPointY;
    float rateSetPointZ;
    float adjustmentX;
    float adjustmentY;
    float adjustmentZ; 
    float throttle;
  }
  v;
  byte buffer[60];
}
transmissionUnion_t;

//radio vars
byte inGainBuffer[7];
byte radioInByte;
uint8_t inGainBufferIndex = 0;
uint8_t gainBufferIndex = 0;

transmissionUnion_t t;
typedef union{
  struct{
    float kp_r_p;
    float ki_r_p;
    float kd_r_p;
    float nPitch;

    float kp_r_r;
    float ki_r_r;
    float kd_r_r;
    float nRoll;

    float kp_r_y;
    float ki_r_y;
    float kd_r_y;
    float nYaw;

    float kp_a_p;
    float ki_a_p;
    float kd_a_p;
    float nPitchA;

    float kp_a_r;
    float ki_a_r;
    float kd_a_r;
    float nRollA;

  }
  v;
  byte buffer[80];
}
gainUnion_t;

gainUnion_t g;//contains the gains
/*float pitchSetPoint;
 float rollSetPoint;
 float pitchAngle;
 float rollAngle;
 float rateSetPointX;    
 float rateSetPointY;
 float rateSetPointZ;
 float adjustmentX;
 float adjustmentY;
 float adjustmentZ; */


/*float kp_r_p = 0.65;
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
 
 float kp_a_p = 2.75;
 float ki_a_p = 0.1;
 float kd_a_p = 0.05;
 float nPitchA = 47.9596;
 
 float kp_a_r = 2.75;
 float ki_a_r = 0.1;
 float kd_a_r = 0.05;
 float nRollA = 47.9596;*/





uint8_t loopCount;
uint16_t i;//index for buffering in the data
uint16_t j;
uint8_t k;//index for RC signals
uint32_t timer,printTimer;
//this is how you use the AHRS and Altimeter
openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&dt);

MPID PitchAngle(&t.v.pitchSetPoint,&imu.pitch,&t.v.rateSetPointY,&integrate,&g.v.kp_a_p,&g.v.ki_a_p,&g.v.kd_a_p,&g.v.nPitchA,&dt,600,600);
MPID RollAngle(&t.v.rollSetPoint,&imu.roll,&t.v.rateSetPointX,&integrate,&g.v.kp_a_r,&g.v.ki_a_r,&g.v.kd_a_r,&g.v.nRollA,&dt,600,600);

MPID PitchRate(&t.v.rateSetPointY,&t.v.degreeGyroY,&t.v.adjustmentY,&integrate,&g.v.kp_r_p,&g.v.ki_r_p,&g.v.kd_r_p,&g.v.nPitch,&dt,500,500);
MPID RollRate(&t.v.rateSetPointX,&t.v.degreeGyroX,&t.v.adjustmentX,&integrate,&g.v.kp_r_r,&g.v.ki_r_r,&g.v.kd_r_r,&g.v.nRoll,&dt,500,500);
MPID YawRate(&t.v.rateSetPointZ,&t.v.degreeGyroZ,&t.v.adjustmentZ,&integrate,&g.v.kp_r_y,&g.v.ki_r_y,&g.v.kd_r_y,&g.v.nYaw,&dt,500,500);

boolean failSafe = false;
boolean toggle;

long failSafeTimer;


void setup(){
  pinMode(RED,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  pinMode(GREEN,OUTPUT);
  digitalWrite(YELLOW,HIGH);
  digitalWrite(RED,HIGH);
  Serial.begin(115200);
  MotorInit();
  DetectRC();
  if (rcType == RC){
    DDRB &= 0xE0;
    PORTB |= 0x1F;
    PCMSK0 |= 0x1F;
    PCICR |= 1<<0;
    delay(100);//wait for a few frames
    Center();
  } 

  //arming procedure
  newRC = false;
  timer = millis();
  while (newRC == false){
    if (rcType == RC){
      delay(100);
    }
    if (rcType != RC){
      FeedLine();
    }
    if (millis() - timer > 1000){//in case it has incorrectly detected serial RC
      rcType = RC;
      DDRB &= 0xE0;
      PORTB |= 0x1F;//turn on pull ups
      PCMSK0 |= 0x1F;//set interrupt mask for all of PORTK
      PCICR |= 1<<0;//enable the pin change interrupt for K
      delay(100);//wait for a few frames
      Center();
      timer = millis();
    }
  }

  while (rcCommands.values.rudder < 1850){
    if (rcType == RC){
      delay(100);//wait for a few frames
    }
    if (rcType != RC){
      FeedLine();
    }
  } 
  newRC = false;


  digitalWrite(RED,LOW);

  I2c.begin();
  I2c.setSpeed(1);

  GyroInit();
  AccInit();

  LevelAngles();


  PitchAngle.reset();
  RollAngle.reset();

  PitchRate.reset();
  RollRate.reset();
  YawRate.reset();
  printTimer = millis();
  while (rcCommands.values.throttle > 1020){
    if (rcType != RC){
      FeedLine();
    }
    if (millis() - printTimer > 500){
      digitalWrite(GREEN,toggle);
      toggle = ~toggle;
      printTimer = millis();
    }

  }
  digitalWrite(YELLOW,LOW);
  digitalWrite(GREEN,HIGH);
  DebugOutput();
  g.v.kp_r_p = 0.65;
  g.v.ki_r_p = 0.05;
  g.v.kd_r_p = 0.01423;
  g.v.nPitch = 19.5924;

  g.v.kp_r_r = 0.65;
  g.v.ki_r_r = 0.05;
  g.v.kd_r_r = 0.01423;
  g.v.nRoll = 19.5924;

  g.v.kp_r_y = 6.9389;
  g.v.ki_r_y = 0.22747;
  g.v.kd_r_y = -0.42597;
  g.v.nYaw = 4.4174;

  g.v.kp_a_p = 4.6527;
  g.v.ki_a_p = 0.2005;
  g.v.kd_a_p = 0.11256;
  g.v.nPitchA = 47.9596;

  g.v.kp_a_r = 4.6527;
  g.v.ki_a_r = 0.2005;
  g.v.kd_a_r = 0.11256;
  g.v.nRollA = 47.9596;

  radio.end();
  radio.begin(115200);

  loopCount = 0;
  printTimer = millis();
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
      imu.GetEuler();
      t.v.pitchAngle = imu.pitch;
      t.v.rollAngle = imu.roll;
      Angle();
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
    //Serial<<imu.pitch<<","<<imu.roll<<"\r\n";

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
  if (gainToggle == true){
    Gains();
  }
  else{
    Transmit();
  }
}

void Smoothing(int16_t *raw, float *smooth){
  *smooth = (*raw * (0.10)) + (*smooth * 0.90);
}
void MapVar (float *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void MapVar (int16_t *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void MapVar (uint16_t *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}









































