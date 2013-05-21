void Transmit(){
  if (millis() - printTimer >= TRANSMIT_TIME){

    printTimer = millis();
    t.v.timestamp = printTimer;
    radio.write(t.buffer,60);
    radio.write(0x55);
    radio.write(0xAA); 

  }
}
void Gains(){
  while (radio.available() > 0 ){
    inByte = radio.read();
    if (inByte == 0xAA){
      inGainBufferIndex = 0;
    }
    inGainBuffer[inGainBufferIndex] = inByte;
    inGainBufferIndex++;
    if(inGainBufferIndex == 7){
      ParseBuffer();
      inGainBufferIndex = 0;
    }
  }
}

void ParseBuffer(){
  if (inGainBuffer[0] != 0xAA && inGainBuffer[6] != 0x55){
    radio.flush();
    return;
  }
  switch (inGainBuffer[1]){
  case 0:
    TransmitGains();
    return;
    break;
  case 1:
    gainBufferIndex = 0;
    BufferIn();
    break;
  case 2:
    gainBufferIndex = 4;
    BufferIn();
    break;
  case 3:
    gainBufferIndex = 8;
    BufferIn();
    break;
  case 4:
    gainBufferIndex = 12;
    BufferIn();
    break;
  case 5:
    gainBufferIndex = 16;
    BufferIn();
    break;
  case 6:
    gainBufferIndex = 20;
    BufferIn();
  case 7:
    gainBufferIndex = 24;
    BufferIn();
    break;
  case 8:
    gainBufferIndex = 28;
    BufferIn();
    break;
  case 9:
    gainBufferIndex = 32;
    BufferIn();
    break;
  case 10:
    gainBufferIndex = 36;
    BufferIn();
    break;
  case 11:
    gainBufferIndex = 40;
    BufferIn();
    break;
  case 12:
    gainBufferIndex = 44;
    BufferIn();
    break;
  case 13:
    gainBufferIndex = 48;
    BufferIn();
    break;
  case 14:
    gainBufferIndex = 52;
    BufferIn();
    break;
  case 15:
    gainBufferIndex = 56;
    BufferIn();
    break;
  case 16:
    gainBufferIndex = 60;
    BufferIn();
    break;
  case 17:
    gainBufferIndex = 64;
    BufferIn();
    break;
  case 18:
    gainBufferIndex = 68;
    BufferIn();
    break;
  case 19:
    gainBufferIndex = 72;
    BufferIn();
    break;
  case 20:
    gainBufferIndex = 76;
    BufferIn();
    break;
  default:
    break;
  }
  TransmitGains();
}

void BufferIn(){
  for (inGainBufferIndex = 2; inGainBufferIndex < 6; inGainBufferIndex++){
    g.buffer[gainBufferIndex]=inGainBuffer[inGainBufferIndex];
    gainBufferIndex++;
  }  
}

void TransmitGains(){
  radio.println("rate:");

  radio.println("pitch :");
  radio.print("kp: ");
  radio.println(g.v.kp_r_p,3);
  radio.print("ki: ");
  radio.println(g.v.ki_r_p,3);
  radio.print("kd: ");
  radio.println(g.v.kd_r_p,3);
  radio.print("fc: ");
  radio.println(g.v.nPitch,3);
  delay(1);

  radio.println("roll:");
  radio.print("kp: ");
  radio.println(g.v.kp_r_r,3);
  radio.print("ki: ");
  radio.println(g.v.ki_r_r,3);
  radio.print("kd: ");
  radio.println(g.v.kd_r_r,3);
  radio.print("fc: ");
  radio.println(g.v.nRoll,3);
  delay(1);
  radio.println("yaw:");
  radio.print("kp: ");
  radio.println(g.v.kp_r_y,3);
  radio.print("ki: ");
  radio.println(g.v.ki_r_y,3);
  radio.print("kd: ");
  radio.println(g.v.kd_r_y,3);
  radio.print("fc: ");
  radio.println(g.v.nYaw,3);
  delay(1);
  radio.println("angle:");

  radio.println("pitch:");
  radio.print("kp: ");
  radio.println(g.v.kp_a_p,3);
  radio.print("ki: ");
  radio.println(g.v.ki_a_p,3);
  radio.print("kd: ");
  radio.println(g.v.kd_a_p,3);
  radio.print("fc: ");
  radio.println(g.v.nPitchA,3);
  delay(1);
  radio.println("roll:");
  radio.print("kp: ");
  radio.println(g.v.kp_a_r,3);
  radio.print("ki: ");
  radio.println(g.v.ki_a_r,3);
  radio.print("kd: ");
  radio.println(g.v.kd_a_r,3);
  radio.print("fc: ");
  radio.println(g.v.nRollA,3);
  delay(1);


}









