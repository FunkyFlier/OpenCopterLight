void GetCalibrationValues(){
  inByte = EEPROM.read(0);
  if (inByte != 0xAA){
    while(1){//hold here because the device is not calibrated 
      digitalWrite(GREEN,HIGH);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(RED,HIGH);
      delay(500);
      digitalWrite(GREEN,LOW);
      digitalWrite(YELLOW,LOW);
      digitalWrite(RED,LOW);
      delay(500);
    }
  }
  for (uint8_t k = 1; k < 25; k++){
    accCal.buffer[k-1] = EEPROM.read(k);
  }
}



void AccInit(){
  //set up the accelerometer
  I2c.write(ADXL345_ADDR,BW_RATE,0x0C);//400Hz update
  delay(10);
  I2c.write(ADXL345_ADDR,POWER_CTL,0x08);//start measurment
  delay(10);
  I2c.write(ADXL345_ADDR,DATA_FORMAT,0x0B);//full resolution + / - 16g
  delay(10);
  I2c.write(ADXL345_ADDR,FIFO_CTL,0x00);  
  delay(10);
  //take a sample and use the values to initilize the smoothing filter
  I2c.read(ADXL345_ADDR,DATAX0,6);
  for (i = 0; i < 6; i++){//the endianness matches as does the axis order
    acc.buffer[i] = I2c.receive();
  }
  smoothAccX = (float)acc.v.x;
  smoothAccY = (float)(-1.0 * acc.v.y);
  smoothAccZ = (float)(-1.0 * acc.v.z);;
}

void GyroInit(){
  //set up the gyroscope
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
  //take a number of samples to find the gyro's zero rate offset
  offsetX = 0;
  offsetY = 0;
  offsetZ = 0;
  gyroSumX = 0;
  gyroSumY = 0;
  gyroSumZ = 0;
  for (j = 0; j < 100; j ++){
    GetGyro();
    delay(3);
  }
  for (j = 0; j < 100; j ++){
    GetGyro();
    gyroSumX += gyro.v.x;
    gyroSumY += gyro.v.y;
    gyroSumZ += gyro.v.z;
    delay(3);
  }
  offsetX = gyroSumX / 100.0;
  offsetY = gyroSumY / 100.0;
  offsetZ = gyroSumZ / 100.0;

}

void GetGyro(){
  //take a reading
  I2c.read(L3GD20_ADDRESS,L3G_OUT_X_L | (1 << 7),6);
  for (i = 0; i < 6; i++){//the endianness matches as does the axis order
    gyro.buffer[i] = I2c.receive();
  }
  //don't forget to convert to radians per second. This absolutely will not work otherwise
  //check the data sheet for more info on this
  //convert the raw gyro data to degrees
  //notice that the sensor must be in North East Down convention
  //the PID loops operate in degrees
  degreeGyroX = (gyro.v.x - offsetX) * 0.07;
  degreeGyroY = -1.0 * ((gyro.v.y - offsetY) * 0.07);
  degreeGyroZ = -1.0 * ((gyro.v.z - offsetZ) * 0.07);
  //convert to radians
  //the complimentary filter works in radians
  radianGyroX = ToRad(degreeGyroX);
  radianGyroY = ToRad(degreeGyroY);
  radianGyroZ = ToRad(degreeGyroZ);
}

void GetAcc(){
  //read the accelerometer and put the data into the North East Down convention
  I2c.read(ADXL345_ADDR,DATAX0,6);
  for (i = 0; i < 6; i++){//the endianness matches as does the axis order
    acc.buffer[i] = I2c.receive();
  }
  acc.v.y *= -1;
  acc.v.z *= -1;
  //the data goes through the low pass filter 
  Smoothing(&acc.v.x,&smoothAccX);//this is a very simple low pass digital filter
  Smoothing(&acc.v.y,&smoothAccY);//it helps significiantlly with vibrations. 
  Smoothing(&acc.v.z,&smoothAccZ);
  //the offset and scaling factor to meters per second is applied
  //the values are generate by the accelerometer calibration sketch
  //notice the sign negation. The axes must be in North East Down convention
  //however gravity is measured as negative in that convention by the accelerometer
  //the complimentary filter expects gravity to be positive in the North East Down convention
  accToFilterX = -1.0 * ((smoothAccX - accCal.v.ACC_OFFSET_X) * accCal.v.ACC_SCALE_X);//if the value from the smoothing filter is sent it will not work when the algorithm normalizes the vector
  accToFilterY = -1.0 * ((smoothAccY - accCal.v.ACC_OFFSET_Y) * accCal.v.ACC_SCALE_Y);
  accToFilterZ = -1.0 * ((smoothAccZ - accCal.v.ACC_OFFSET_Z) * accCal.v.ACC_SCALE_Z);


}





