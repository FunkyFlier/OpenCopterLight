void ISRConfig(){
  TCCR4B = (1<<CS43)|(1<<CS40);
  //TCCR4B = (1<<CS43);
  TIMSK4 = (1<<OCIE4A);
  OCR4C = 165;
  OCR4A = 165;
}


ISR(TIMER4_COMPA_vect, ISR_NOBLOCK){
  if (watchDogStartCount == true){
    watchDogFailSafeCounter++;
    RCFailSafeCounter++;
  }
  if (watchDogFailSafeCounter >=190){
    TIMSK4 = (0<<OCIE4A);
    digitalWrite(RED,LOW);
    digitalWrite(YELLOW,LOW);
    digitalWrite(GREEN,LOW);
    Motor1WriteMicros(1000);//set the output compare value
    Motor2WriteMicros(1000);
    Motor3WriteMicros(1000);
    Motor4WriteMicros(1000);
    while(1){
      digitalWrite(RED,HIGH);
      delay(500);
      digitalWrite(RED,LOW);
      delay(500);
    }
  }

}

