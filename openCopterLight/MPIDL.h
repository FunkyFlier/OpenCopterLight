#ifndef MPIDL_h
#define MPIDL_h

#include <Arduino.h>

class MPID{
public:
  MPID(float*, float*, float*,boolean*,float*, float*, float*, float*,const float*,float,float);
  void calculate();
  void reset();
  float integralLimitHigh;
  float integralLimitLow;
  float outputLimitHigh;
  float outputLimitLow;
  const float *dt;
  float *setPoint;
  float *actual;
  float *adjustment;
  boolean *integrate;
  float *kp;
  float *ki;
  float *kd;
  float *fc;
  float prevError;
  float prevActual;
  float error;
  float iError;
  float dError;
  float dErrorPrev;
private:
  

};
class MYAW{
public:
  MYAW(float*, float*, float*,boolean*,float*, float*, float*, float*,float*,float,float);
  void calculate();
  void reset();
private:
  float integralLimitHigh;
  float integralLimitLow;
  float outputLimitHigh;
  float outputLimitLow;
  float *dt;
  float *setPoint;
  float *actual;
  float *adjustment;
  boolean *integrate;
  float *kp;
  float *ki;
  float *kd;
  float *fc;
  float prevError;
  float prevActual;
  float error;
  float iError;
  float dError;
  float dErrorPrev;
  int singularityState;
  float PIDAngle;
  float errorDiff;

};


#endif

