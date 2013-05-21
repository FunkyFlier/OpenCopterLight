#include <Streaming.h>

#define GAIN_STATE 0
#define DATA_STATE 1


#define terminal Serial
#define radio Serial3

uint8_t state;

byte inByte;
byte buffer[350];
int bufferIndex = 0;
boolean parseReady = false;

typedef union {
  struct{
    long timestamp;
    float pitchSetPoint;
    float pitchAngle;
    float rollSetPoint;
    float rollAngle;
    float gyroX;
    float gyroY;
    float gyroZ;
    float rateSetPointX;    
    float rateSetPointY;
    float rateSetPointZ;
    float adjustmentX;
    float adjustmentY;
    float adjustmentZ; 
    float throttle;
  }
  vs;
  byte buffer[60];
}
transmissionUnion_t;



transmissionUnion_t tU;

int command;

typedef union{
  float outVar;
  byte buffer[4];
}
OutFloat_t;

OutFloat_t outFloat;

uint8_t gainSelect;
long recieveTimer;

void setup(){
  radio.begin(115200);

  terminal.begin(115200);

  state = GAIN_STATE;


}

void loop(){
  switch (state){
  case GAIN_STATE:
    //terminal<<"GAIN_STATE\r\n";
    GainHandler();
    break;
  case DATA_STATE:
    //terminal<<"DATA_STATE\r\n";
    DataHandler();
    break;
  }
}

void GainHandler(){
  terminal<<"Enter 0 to switch to Data mode\r\n";
  terminal<<"Gains:\r\n1 - Rate\r\n2 - Angle\r\n3 - query gains\r\n";
  terminal.read();
  //terminal<<gains.available()<<"\r\n";
  while(terminal.available() == 0 ){
  }

  command = terminal.parseInt();

  switch (command){
  case 0:
    state = DATA_STATE;
    terminal<<"\r\nEntering data mode\r\n";
    return;
    break;
  case 1:
    Rate();
    break;
  case 2:
    Angle();
    break;
  case 3:
    Query();
    break;
  default:
    terminal<<"invalid command1\r\n";
    return;
    break;
  }
}


void Rate(){
  terminal<<"\r\n1 - pitch \r\n2 - roll\r\n3 - yaw\r\n";
  while(terminal.available() == 0 ){
  }

  command = terminal.parseInt();
  switch (command){
  case 0:
    state = DATA_STATE;
    terminal<<"\r\nEntering data mode\r\n";
    return;
    break;
  case 1:
    gainSelect = 0;
    break;
  case 2:
    gainSelect = 4;
    break;
  case 3:
    gainSelect = 8;
    break;  
  default:
    terminal<<"invalid command3\r\n";
    return;
    break;
  }

  terminal<<"\r\n1 - Kp\r\n2 - Ki\r\n3 - Kd\r\n4 - Fc\r\n";
  while(terminal.available() == 0 ){
  }

  command = terminal.parseInt();
  switch (command){
  case 0:
    state = DATA_STATE;
    terminal<<"\r\nEntering data mode\r\n";
    return;
    break;
  case 1:
    gainSelect += 1;
    break;
  case 2:
    gainSelect += 2;
    break;
  case 3:
    gainSelect += 3;
    break;
  case 4:
    gainSelect += 4;
    break;
  default:
    terminal<<"invalid command4\r\n";
    return;
    break;
  }

  terminal<<"Enter value:\r\n";
  while(terminal.available() == 0 ){
  }

  outFloat.outVar = terminal.parseFloat();

  TransmitGains();

}
void Angle(){
  //terminal<<"\r\n1 - pitch\r\n2 - roll\r\n3 - yaw\r\n";
  terminal<<"\r\n1 - pitch\r\n2 - roll\r\n";
  while(terminal.available() == 0 ){
  }

  command = terminal.parseInt();
  switch (command){
  case 0:
    state = DATA_STATE;
    terminal<<"\r\nEntering data mode\r\n";
    return;
    break;
  case 1:
    gainSelect = 12;
    break;
  case 2:
    gainSelect = 16;
    break;
  /*case 3:
    gainSelect = 20;
    break;*/
  default:
    terminal<<"invalid command5\r\n";
    return;
    break;
  }

  terminal<<"\r\n1 - Kp\r\n2 - Ki\r\n3 - Kd\r\n4 - Fc\r\n";
  while(terminal.available() == 0 ){
  }

  command = terminal.parseInt();
  switch (command){
  case 0:
    state = DATA_STATE;
    terminal<<"\r\nEntering data mode\r\n";
    return;
    break;
  case 1:
    gainSelect += 1;
    break;
  case 2:
    gainSelect += 2;
    break;
  case 3:
    gainSelect += 3;
    break;
  case 4:
    gainSelect += 4;
    break;    
  default:
    terminal<<"invalid command\r\n";
    return;
    break;
  }

  terminal<<"Enter value:\r\n";
  while(terminal.available() == 0 ){
  }

  outFloat.outVar = terminal.parseFloat();

  TransmitGains();  
}
void Query(){
  gainSelect = 0;
  outFloat.outVar = 0.0;
  TransmitGains();
}

void TransmitGains(){
  radio.write(0xAA);
  radio.write(gainSelect);
  for(int i = 0;i<4;i++){
    radio.write(outFloat.buffer[i]);
  }
  radio.write(0x55);
  recieveTimer = millis();

  while(millis() - recieveTimer < 3000){
    if (radio.available()>0){
      while(radio.available()>0){
        inByte = radio.read();
        terminal.write(inByte); 
      }
    }
  }
}
void DataHandler(){
  if (radio.available() > 60){
    while(radio.available() > 0){
      inByte = radio.read();
      buffer[bufferIndex] = inByte;
      bufferIndex++;
      if (inByte == 0x55){
        inByte = radio.read();
        buffer[bufferIndex] = inByte;
        bufferIndex++;
        if(inByte == 0xAA){
          parseReady = true;
          break;
        }
      }
    }
  }

    
  if (parseReady == true){
    if (bufferIndex == 62){
      memcpy(tU.buffer,buffer,60);

      terminal << tU.vs.timestamp << "," << tU.vs.pitchSetPoint << "," << tU.vs.pitchAngle << "," << tU.vs.rollSetPoint << ","
        << tU.vs.rollAngle<< "," << tU.vs.gyroX << "," << tU.vs.gyroY << "," << tU.vs.gyroZ  << "," << tU.vs.rateSetPointX  
        << "," << tU.vs.rateSetPointY << "," <<  tU.vs.rateSetPointZ<< "," << tU.vs.adjustmentX << "," <<  tU.vs.adjustmentY
        << "," << tU.vs.adjustmentZ << "," <<  tU.vs.throttle << "\r\n";
    }
    bufferIndex=0;
    parseReady = false;
  }
  if(terminal.available() >0){
    while(terminal.available() >0){
      terminal.read();
    }
    terminal<<"Exiting data logging mode\r\n";
    state = GAIN_STATE;
  }

}












