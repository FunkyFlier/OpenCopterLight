void AccInit(){
  //set up the accelerometer
  I2c.write(ADXL345_ADDR,BW_RATE,0x0B);
  delay(10);
  I2c.write(ADXL345_ADDR,POWER_CTL,0x08);//start measurment
  delay(10);
  I2c.write(ADXL345_ADDR,DATA_FORMAT,0x0B);//full resolution + / - 16g
  delay(10);
  I2c.write(ADXL345_ADDR,FIFO_CTL,0x00);  
  delay(10);
  
}

void GetAcc(){
  I2c.read(ADXL345_ADDR,DATAX0,6);
  for (int i = 0; i < 6; i++){//the endianness matches as does the axis order
    acc.buffer[i] = I2c.receive();
  }
  
  acc.v.y *= -1;
  acc.v.z *= -1;
  Serial<<acc.v.x<<","<<acc.v.y<<","<<acc.v.z<<"\r\n";
}
