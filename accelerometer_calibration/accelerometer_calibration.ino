/*
# copyright 2013 by Michael Baker
 # This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License. 
 # To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/ or send a
 # letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 # Adapted from the work of 
 # Available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
 #
 # Described at http://wp.me/p1CQVj-1k
 */
#include <I2C.h>
/*
http://dsscircuits.com/articles/arduino-i2c-master-library.html
 */
#include <Streaming.h>
/*
http://arduiniana.org/libraries/streaming/
 */


//acc defines - Analog Devices ADXL345
#define ADXL435_ADDR 0x53
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define FIFO_CTL 0x38
#define DATAX0 0x32


const int NUM_READINGS = 6;

//data collection structures
int *data;   //dynamically allocated array for data storage.  Don't let it get too big!
int samp_capacity = 0;             //the capacity of the sample data array
int n_samp = 0;                    // Number of samples used for calibration
int sample_size=32;                // Number of measurements averaged to produce 1 sample

// Basic UI functions
int ledPin = 13;        //turn a light on when data is being collected



float beta[6];                        //parameters for model.  beta[0], beta[1], and beta[2] are the 0-G marks (about 512),
// while beta[3], beta[4], and beta[5] are the scaling factors.  So, e.g., if xpin reads
// value x, number of G's in the x direction in beta[3]*(x - beta[0]).


//matrices for Gauss-Newton computations
float JS[6][6];
float dS[6];
float delta[6];


void calibrate_model_matrices();
void find_delta();
void calibrate_model();
void reset_calibration_matrices();

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

Sensor_t acc;

//RC vars

//RC defines
#define DSM2 0
#define DSMX 1
#define SBUS 2
#define RC 3
#define HEX_ZERO 0x00

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

long timer;

boolean sample = false,calculate = false;

void setup()
{
  // initialize the serial communications:
  Serial.begin(115200);

  while (!Serial) ;
  Serial.println("start");

  I2c.begin();
  I2c.setSpeed(1);
  DetectRC();
  if (rcType == RC){
    DDRB &= 0xE0;
    PORTB |= 0x1F;
    PCMSK0 |= 0x1F;
    PCICR |= 1<<0;
    delay(100);//wait for a few frames
    Center();
  } 
  AccInit();
  pinMode(ledPin, OUTPUT);



  //initialize to handle 20 samples
  samp_capacity = 20;
  data = (int*)malloc(samp_capacity*3*sizeof(int));

  reset_calibration_matrices();

  //initialize beta to something reasonable

  beta[0] = beta[1] = beta[2] = 0.0;
  beta[3] = beta[4] = beta[5] = 0.00390625; 


  Serial.println("ES");
}

void loop(){
  if (rcType != RC){
    FeedLine();
  }  
  if (newRC == true){
    newRC = false;
    if (rcCommands.values.rudder > 1700) {
      sample = true;
    }
    if (rcCommands.values.rudder < 1200) {
      calculate = true;
    }
  }
  if (sample == true) {
    sample = false;
    Serial.println("Take sample");
    delay(2000);
    take_sample(data + 3*(n_samp%samp_capacity));
    n_samp++;
    while(Serial1.available() > 0){//flush the port to get the newest frame
      Serial1.read();
    }
  }

  if (calculate == true) {
    calculate = false;
    Serial.println("Calibrate");
    delay(2000);
    calibrate_model();
    while(Serial1.available() > 0){//flush the port to get the newest frame
      Serial1.read();
    }
  }

}

//pass in a length-3 array where the data will be written
void take_sample(int* sample_out) {
  int i=0;
  int first_pass_size = 5;
  int success = 0;


  while (success == 0) {
    //First, run through 32 samples and accumulate the mean and variance.
    //Make all variables longs because we will do some aritmetic that 
    // will overflow an int.
    long sum[] = {
      0,0,0                            };
    long sum_squares[] = {
      0,0,0                            };
    long variance[] = {
      0,0,0                            };
    long x,y,z;
    for(i=0;i< (1<<first_pass_size);++i) {

      GetAcc();
      x = (long)acc.v.x;
      y = (long)acc.v.y;
      z = (long)acc.v.z;

      delay(20);


      sum[0] += x;
      sum[1] += y;
      sum[2] += z;

      sum_squares[0] += x*x;
      sum_squares[1] += y*y;
      sum_squares[2] += z*z;
    }

    //now compute the variance onh each axis. Don't divide by 32 -- keep the extra digits
    //around to reduce quantization errors.  So really computing variance*32 here. Also make sure it is > 0.
    for(i=0;i<3; i++) {

      variance[i] = 1+ sum_squares[i] - (sum[i]*sum[i])/32;
    }


    //with mean and variance in place, start collecting real samples but filter out outliers.
    //Track the success rate and start over if we get too many fails.
    unsigned int success_count = 0;
    unsigned int fail_count = 0;
    i=0;
    sample_out[0] = sample_out[2] = sample_out[1] = 0;


    while(i < sample_size) {

      GetAcc();
      x = (long)acc.v.x;
      y = (long)acc.v.y;
      z = (long)acc.v.z;

      delay(20);

      long dx = x*32 - sum[0];
      long dy = y*32 - sum[1];
      long dz = z*32 - sum[2];

      //check to see if it is any good (within 3 std deviations)
      if((dx*dx)/32 < 9*variance[0]
        &&(dy*dy)/32 < 9*variance[1]
        &&(dz*dz)/32 < 9*variance[2]) {
        success_count++;
        sample_out[0] += x;
        sample_out[1] += y;
        sample_out[2] += z;

        ++i;
      } 
      else {        
        fail_count++;
      }

      if(fail_count > success_count && i > 10) {
        //we're failing too much, start over!
        Serial.println("#Sample fail 1");
        break;
      } 

    }


    //if we got our samples, mark the success.  Otherwise we'll start over.
    if (i == sample_size) {
      success = 1;

      Serial.print("# ");
      Serial.print(sample_out[0]/32.0);
      Serial.print(" ");
      Serial.print(sample_out[1]/32.0);
      Serial.print(" ");
      Serial.print(sample_out[2]/32.0);
      Serial.print(";\r\n");

    }
  }
}

//Gauss-Newton functions

void reset_calibration_matrices() {
  int j,k;
  for(j=0;j<6;++j) {
    dS[j] = 0.0;
    for(k=0;k<6;++k) {
      JS[j][k] = 0.0;
    }
  }

}

void update_calibration_matrices(const int* data) {
  int j, k;
  float dx, b;
  float residual = 1.0;
  float jacobian[6];

  for(j=0;j<3;++j) {
    b = beta[3+j];
    dx = ((float)data[j])/sample_size - beta[j];
    residual -= b*b*dx*dx;
    jacobian[j] = 2.0*b*b*dx;
    jacobian[3+j] = -2.0*b*dx*dx;
  }

  for(j=0;j<6;++j) {
    dS[j] += jacobian[j]*residual;
    for(k=0;k<6;++k) {
      JS[j][k] += jacobian[j]*jacobian[k];
    }
  }

}



void compute_calibration_matrices() {
  int i, j, k;
  float dx, b;

  reset_calibration_matrices();
  int ub = n_samp < samp_capacity ? n_samp : samp_capacity;
  for(i=0;i<ub;i++) {    
    update_calibration_matrices(data+3*i);
  }

}

void find_delta() {
  //Solve 6-d matrix equation JS*x = dS
  //first put in upper triangular form
  int i,j,k;
  float mu;

  //make upper triangular
  for(i=0;i<6;++i) {
    //eliminate all nonzero entries below JS[i][i]
    for(j=i+1;j<6;++j) {
      mu = JS[i][j]/JS[i][i];
      if(mu != 0.0) {
        dS[j] -= mu*dS[i];
        for(k=j;k<6;++k) {
          JS[k][j] -= mu*JS[k][i];
        } 
      }
    }
  }

  //back-substitute
  for(i=5;i>=0;--i) {
    dS[i] /= JS[i][i];
    JS[i][i] = 1.0;
    for(j=0;j<i;++j) {
      mu = JS[i][j];
      dS[j] -= mu*dS[i];
      JS[i][j] = 0.0;
    }
  }

  for(i=0;i<6;++i) {
    delta[i] = dS[i];
  }
}

void calibrate_model() {
  int i;
  float eps = 0.000000001;
  int num_iterations = 20;
  float change = 100.0;
  while (--num_iterations >=0 && change > eps) {
    compute_calibration_matrices();
    find_delta();
    change = delta[0]*delta[0] + delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2] + delta[3]*delta[3]/(beta[3]*beta[3]) + delta[4]*delta[4]/(beta[4]*beta[4]) + delta[5]*delta[5]/(beta[5]*beta[5]); 

    for(i=0;i<6;++i) {
      beta[i] -= delta[i];
    }

    reset_calibration_matrices();

  }
  Serial<<"\r\n#define ACC_OFFSET_X: "<<_FLOAT(beta[0],7)<<"\r\n#define ACC_OFFSET_Y: "<<_FLOAT(beta[1],7)<<"\r\n#define ACC_OFFSET_Z: "<<_FLOAT(beta[2],7)
    <<"\r\n#define ACC_SCALE_X: "<<_FLOAT((beta[3]*9.8),7)<<"\r\n#define ACC_SCALE_Y: "<<_FLOAT((beta[4]*9.8),7)<<"\r\n#define ACC_SCALE_Z: "<<_FLOAT((beta[5]*9.8),7)<<"\r\n";

  /*Serial.print("\n");
   for(i=0;i<6;++i) {
   Serial.print(beta[i], 7);
   Serial.print(" ");
   }
   Serial.println();*/
}














