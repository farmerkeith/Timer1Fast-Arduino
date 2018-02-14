// functions to inspect the values of various registers in the Atmega328P micro controller 

class inspector {
  public:
    bool readBit (byte reg, byte bitPos);
    void all();
    void wgm();
    void com1a();
    void com1b();
    void clock_select();
    void readWgm();
    void tifr1();
    void timsk1();
    void globalInterrupt();

  private:
    byte Tccr1a=TCCR1A;
    byte Tccr1b=TCCR1B;
    byte Tccr1c=TCCR1C;
    byte Tifr1=TIFR1;
    byte Timsk1=TIMSK1;
    uint16_t ocr1a=OCR1A;
    uint16_t ocr1b=OCR1B;
    uint16_t Tcnt1=TCNT1;
    byte wgm1=0;
    
} inspector ;

bool inspector::readBit (byte reg, byte bitPos){
  return (reg>>bitPos)&1;
}

void inspector::all(){
  wgm();
  com1a();
  com1b();
  clock_select();
  tifr1();
  timsk1();
  globalInterrupt();
}

void inspector::readWgm(){
  wgm1 = readBit(TCCR1A,0);
  wgm1 += 2* readBit(TCCR1A,1);
  wgm1 += 4* readBit(TCCR1B,3);
  wgm1 += 8* readBit(TCCR1B,4);
}

void inspector::wgm(){
  readWgm();
  Serial.print ("\n wgm=");
  Serial.print (wgm1);
  switch (wgm1) {
    case 0: Serial.print (F(" normal operation")); break;
    case 1: Serial.print (F(" PWM Phase correct TOP 255")); break;
    case 2: Serial.print (F(" PWM Phase correct TOP 511")); break;
    case 3: Serial.print (F(" PWM Phase correct TOP 1023")); break;
    case 4: Serial.print (F(" CTC match TOP OCR1A")); break;
    case 5: Serial.print (F(" PWM Fast TOP 255")); break;
    case 6: Serial.print (F(" PWM Fast TOP 511")); break;
    case 7: Serial.print (F(" PWM Fast TOP 1023")); break;
    case 8: Serial.print (F(" PWM P&F correct TOP ICR1")); break;
    case 9: Serial.print (F(" PWM P&F correct TOP OCR1A")); break;
    case 10: Serial.print (F(" PWM Phase correct TOP ICR1")); break;
    case 11: Serial.print (F(" PWM Phase correct TOP OCR1A")); break;
    case 12: Serial.print (F(" CTC match TOP ICR1")); break;
    case 13: Serial.print (F(" reserved ")); break;
    case 14: Serial.print (F(" PWM Fast TOP ICR1")); break;
    case 15: Serial.print (F(" PWM Fast TOP OCR1A")); break;
  }
}

void inspector::com1a(){
  byte com1a = readBit(TCCR1A,6);
  com1a += 2* readBit(TCCR1A,7);
  Serial.print (F("\n com1a="));
  Serial.print (com1a);
  readWgm();
  if (wgm1==0|wgm1==4|wgm1==12){ // normal or CTC match
    switch (com1a){
      case 0: Serial.print (F(" normal port operation")); break;
      case 1: Serial.print (F(" toggle pin on compare match")); break;
      case 2: Serial.print (F(" pin low on compare match, clear")); break;
      case 3: Serial.print (F(" pin hi  on compare match, set")); break;
    }
  }
  if (wgm1==5|wgm1==6|wgm1==7|wgm1==14|wgm1==15){ // Fast PWM
    switch (com1a){
      case 0: Serial.print (F(" normal port operation")); break;
      case 1:
        if (wgm1==14|wgm1==15) Serial.print (F(" toggle A pin on compare match")); 
        else Serial.print (F(" normal port operation"));
        break;
      case 2: Serial.print (F(" pin low & clear on compare match (non-inv)")); break;
      case 3: Serial.print (F(" pin hi & set on compare match (inverting)")); break;
    }
  }

  if (wgm1==1|wgm1==2|wgm1==3|wgm1==8|wgm1==9|wgm1==10|wgm1==11){ // Phase or P&F correct PWM
    switch (com1a){
      case 0: Serial.print (F(" normal port operation")); break;
      case 1:
        if (wgm1==9|wgm1==11) Serial.print (F(" toggle A pin on compare match")); 
        else Serial.print (F(" normal port operation"));
        break;
      case 2: Serial.print (F(" pin low on compare match Up, Hi on Down")); break;
      case 3: Serial.print (F(" pin hi  on compare match Up, Lo on Down")); break;
    }
  }
  if (wgm1==13){ // wgm value reserved, not in use 
    Serial.print (F(" reserved wgm1 value, com1a not defined"));
  }
}

void inspector::com1b(){
  byte com1b = readBit(TCCR1A,4);
  com1b += 2* readBit(TCCR1A,5);
  Serial.print (F("\n com1b="));
  Serial.print (com1b);
  readWgm();
  if (wgm1==0|wgm1==4|wgm1==12){ // normal or CTC match
    switch (com1b){
      case 0: Serial.print (F(" normal port operation")); break;
      case 1: Serial.print (F(" toggle pin on compare match")); break;
      case 2: Serial.print (F(" pin low on compare match, clear")); break;
      case 3: Serial.print (F(" pin hi  on compare match, set")); break;
    }
  }
  if (wgm1==5|wgm1==6|wgm1==7|wgm1==14|wgm1==15){ // Fast PWM
    switch (com1b){
      case 0: Serial.print (F(" normal port operation")); break;
      case 1: Serial.print (F(" normal port operation")); break;
      case 2: Serial.print (F(" pin low & clear on compare match (non-inv)")); break;
      case 3: Serial.print (F(" pin hi & set on compare match (inverting)")); break;
    }
  }

  if (wgm1==1|wgm1==2|wgm1==3|wgm1==8|wgm1==9|wgm1==10|wgm1==11){ // Phase or P&F correct PWM
    switch (com1b){
      case 0: Serial.print (F(" normal port operation")); break;
      case 1: Serial.print (F(" normal port operation")); break;
      case 2: Serial.print (F(" pin low on compare match Up, Hi on Down")); break;
      case 3: Serial.print (F(" pin hi  on compare match Up, Lo on Down")); break;
    }
  }
  if (wgm1==13){ // wgm value reserved, not in use 
    Serial.print (F(" reserved wgm1 value, com1b not defined"));
  }
}

void inspector::clock_select(){
  byte csb = readBit(TCCR1B,0);
  csb += 2* readBit(TCCR1B,1);
  csb += 4* readBit(TCCR1B,2);
  int prescaler = 1;
  switch (csb){
    case 1: prescaler = 1; break;
    case 2: prescaler = 8; break;
    case 3: prescaler = 64; break;
    case 4: prescaler = 256; break;
    case 5: prescaler = 1024; break;
    default: prescaler = 1; break;
  }
  Serial.print (F("\n csb="));
  Serial.print (csb);
  switch (csb){
    case 0: Serial.print (F(" T/Counter stopped")); break;
    case 1: case 2: case 3: case 4: case 5: 
      Serial.print (F(" prescaler=")); 
      Serial.print (prescaler);
      break;
    case 6: Serial.print (F(" ext. clock on T1, falling")); break;
    case 7: Serial.print (F(" ext. clock on T1, rising")); break;
  }
  Serial.print (F(" clocks=")); Serial.print ((long)ICR1 * prescaler);
}

void inspector::tifr1(){
  byte tov1 = readBit(TIFR1,0);
  byte ocf1a = readBit(TIFR1,1);
  byte ocf1b = readBit(TIFR1,2);
  byte icf1 = readBit(TIFR1,5);
  Serial.print (F("\n tifr1 interrupt flags icf1=")); Serial.print (icf1);
  Serial.print (F(" ocf1b=")); Serial.print (ocf1b);
  Serial.print (F(" ocf1a=")); Serial.print (ocf1a);
  Serial.print (F(" tov1=")); Serial.print (tov1);
}

void inspector::timsk1(){
  byte toie1 = readBit(TIMSK1,0);
  byte ocie1a = readBit(TIMSK1,1);
  byte ocie1b = readBit(TIMSK1,2);
  byte icie1 = readBit(TIMSK1,5);
  Serial.print (F("\n timsk1 interrupt masks icie1=")); Serial.print (icie1);
  Serial.print (F(" ocie1b=")); Serial.print (ocie1b);
  Serial.print (F(" ocie1a=")); Serial.print (ocie1a);
  Serial.print (F(" toie1=")); Serial.print (toie1);
}

void inspector::globalInterrupt(){
  bool gi = readBit(SREG,7);
  if (gi) Serial.print (F("\n global interrupts enabled in SREG"));
  else  (F("\n global interrupts disabled in SREG"));
}


