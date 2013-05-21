void MotorInit(){
  DDRC |= B01000000;//set the ports as outputs
  DDRB |= B11100000;


                                     
  // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTOM
  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);                 // Prescaler set to 8, that gives us a resolution of 0.5us
  ICR1 = PERIOD;                                // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.  

  TCCR3A = (1<<WGM31)|(1<<COM3A1);
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
  ICR3 = PERIOD;

  Motor1WriteMicros(1000);//set the output compare value
  Motor2WriteMicros(1000);
  Motor3WriteMicros(1000);
  Motor4WriteMicros(1000);
  
}

void MotorHandler(){
  if (rcCommands.values.throttle < 1100){
    integrate = false;
    PitchAngle.reset();
    RollAngle.reset();
    PitchRate.reset();
    RollRate.reset();
    YawRate.reset();


    Motor1WriteMicros(1000);
    Motor2WriteMicros(1000);
    Motor3WriteMicros(1000);
    Motor4WriteMicros(1000); 
    //add arm hold condition here
    if (rcCommands.values.rudder > 1700){
      digitalWrite(YELLOW,HIGH);
      gainToggle = false;
    }
    if (rcCommands.values.rudder < 1300){
      digitalWrite(YELLOW,LOW);
      gainToggle = true;
    }
  }
  else{
    motorCommand1 = constrain((uint16_t)(rcCommands.values.throttle  + t.v.adjustmentX + t.v.adjustmentY - t.v.adjustmentZ),1000,2000);
    motorCommand2 = constrain((uint16_t)(rcCommands.values.throttle - t.v.adjustmentX + t.v.adjustmentY + t.v.adjustmentZ),1000,2000);
    motorCommand3 = constrain((uint16_t)(rcCommands.values.throttle - t.v.adjustmentX - t.v.adjustmentY - t.v.adjustmentZ),1000,2000);
    motorCommand4 = constrain((uint16_t)(rcCommands.values.throttle + t.v.adjustmentX - t.v.adjustmentY + t.v.adjustmentZ),1000,2000);
    
    Motor1WriteMicros(motorCommand1);
    Motor2WriteMicros(motorCommand2);
    Motor3WriteMicros(motorCommand3);
    Motor4WriteMicros(motorCommand4);
   
  }
}


