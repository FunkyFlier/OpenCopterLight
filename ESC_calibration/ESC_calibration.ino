
//motor defines
#define FREQ 400
#define PRESCALE 8
#define PERIOD ((F_CPU/PRESCALE/FREQ) - 1)

#define Motor1WriteMicros(x) OCR3A = x * 2
#define Motor2WriteMicros(x) OCR1B = x * 2
#define Motor3WriteMicros(x) OCR1A = x * 2
#define Motor4WriteMicros(x) OCR1C = x * 2


void setup(){
  MotorInit();
  Motor1WriteMicros(2000);
  Motor2WriteMicros(2000);
  Motor3WriteMicros(2000);
  Motor4WriteMicros(2000);
  //adjust the delay if your ESC goes into program mode
  delay(2000);
  Motor1WriteMicros(1000);
  Motor2WriteMicros(1000);
  Motor3WriteMicros(1000);
  Motor4WriteMicros(1000);
}
void loop(){
  
}



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



}

