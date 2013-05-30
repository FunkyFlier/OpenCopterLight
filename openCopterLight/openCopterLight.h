//head with the defines

#define LIFTOFF 1175 

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
#define ADXL345_ADDR 0x53
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

//trigonometric defines
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
