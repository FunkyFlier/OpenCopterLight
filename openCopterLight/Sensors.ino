void LevelAngles(){
  timer = micros();
  delay(5);
  for( int l = 0; l < 1000; l++){//run the IMU so that the error can be driven to zero - keep it still for this
    dt = ((micros() - timer) / 1000000.0);
    timer = micros();
    GetGyro();
    GetAcc();
    imu.IMUupdate();
    delay(5);
  }
  imu.GetEuler();
  imu.pitchOffset = imu.pitch;
  imu.rollOffset = imu.roll;
}


void AccInit(){
  I2c.write(ADXL435_ADDR,BW_RATE,0x0C);//400Hz update
  delay(10);
  I2c.write(ADXL435_ADDR,POWER_CTL,0x08);//start measurment
  delay(10);
  I2c.write(ADXL435_ADDR,DATA_FORMAT,0x0B);//full resolution + / - 16g
  delay(10);
  I2c.write(ADXL435_ADDR,FIFO_CTL,0x00);  
  delay(10);
  for (int i = 0; i<500; i++){
    GetAcc();
    delay(3);
  }
  
}

void GyroInit(){
  I2c.write(L3GD20_ADDRESS,L3G_CTRL_REG2,0x00);
  delay(10);
  I2c.write(L3GD20_ADDRESS,L3G_CTRL_REG3,0x00);
  delay(10);
  I2c.write(L3GD20_ADDRESS,L3G_CTRL_REG4,0x20);
  delay(10);
  I2c.write(L3GD20_ADDRESS,L3G_CTRL_REG5,0x02);
  delay(10);
  I2c.write(L3GD20_ADDRESS,L3G_CTRL_REG1,0xCF);
  delay(10);
  //this section takes an average of 500 samples to calculate the offset
  //if this step is skipped the IMU will still work, but this simple step gives better results
  offsetX = 0;
  offsetY = 0;
  offsetZ = 0;
  gyroSumX = 0;
  gyroSumY = 0;
  gyroSumZ = 0;
  for (j = 0; j < 250; j ++){//give the internal LPF time to warm up
    GetGyro();
    delay(3);
  }
  for (j = 0; j < 250; j ++){//give the internal LPF time to warm up
    GetGyro();
    gyroSumX += gyro.v.x;
    gyroSumY += gyro.v.y;
    gyroSumZ += gyro.v.z;
    delay(3);
  }
  offsetX = gyroSumX / 250.0;
  offsetY = gyroSumY / 250.0;
  offsetZ = gyroSumZ / 250.0;

}

void GetGyro(){
  I2c.read(L3GD20_ADDRESS,L3G_OUT_X_L | (1 << 7),6);
  for (i = 0; i < 6; i++){//the endianness matches as does the axis order
    gyro.buffer[i] = I2c.receive();
  }
  //don't forget to convert to radians per second. This absolutely will not work otherwise
  //check the data sheet for more info on this
  t.v.degreeGyroX = (gyro.v.x - offsetX) * 0.07;
  t.v.degreeGyroY = -1.0 * ((gyro.v.y - offsetY) * 0.07);
  t.v.degreeGyroZ = -1.0 * ((gyro.v.z - offsetZ) * 0.07);
  radianGyroX = ToRad(t.v.degreeGyroX);
  radianGyroY = ToRad(t.v.degreeGyroY);
  radianGyroZ = ToRad(t.v.degreeGyroZ);
}

void GetAcc(){
  I2c.read(ADXL435_ADDR,DATAX0,6);
  for (i = 0; i < 6; i++){//the endianness matches as does the axis order
    acc.buffer[i] = I2c.receive();
  }

  
  //the filter expects gravity to be in NED coordinates
  Smoothing(&acc.v.x,&smoothAccX);//this is a very simple low pass digital filter
  Smoothing(&acc.v.y,&smoothAccY);//it helps significiantlly with vibrations. 
  Smoothing(&acc.v.z,&smoothAccZ);//if the applicaion is not prone to vibrations this can skipped and the raw value simply recast as a float
  
  accToFilterX = -1.0 * smoothAccX;//if the value from the smoothing filter is sent it will not work when the algorithm normalizes the vector
  accToFilterY = smoothAccY;
  accToFilterZ = smoothAccZ;


}





