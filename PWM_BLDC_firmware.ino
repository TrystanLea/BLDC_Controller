// --------------------------------------------------------------------------
// WORK IN PROGRESS, not working quite right I dont think...
// --------------------------------------------------------------------------

// https://sites.google.com/site/qeewiki/books/avr-guide/pwm-on-the-atmega328

char C = 7;
char c = 6;

char B = 4;
char b = 5;

char A = 2;
char a = 3;

byte out = 0;
byte speedv = 100;

void setup() {
  DDRD = DDRD | B11111100;

  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  TCCR2A = 0b00100011; // A:11, B:3 enabled & Fast PWM
  TCCR2B = 0b00000001; // Clock/8, Fast PWM frequency: 16000000÷(8×(1+255)) = 7812 Hz
  OCR2A = speedv;
  OCR2B = speedv;
  TCCR2A = 0;
  
  TCCR0A = 0b00000011; // Fast PWM
  TCCR0B = 0b00000001; // Clock/8, Fast PWM frequency: 16000000÷(8×(1+255)) = 7812 Hz
  OCR0A = speedv;
  OCR0B = speedv;
  TCCR0A = 0;
  /*
  TCCR1A = 0b10100001; // A:9, B:10 enabled & Fast PWM
  TCCR1B = 0b00001010; // Clock/8, Fast PWM frequency: 16000000÷(8×(1+255)) = 7812 Hz
  OCR1A = 128;
  OCR1B = 128;
  */  
}

void loop() {
  byte h = PINC;
  // HALL:    CBA   DRIVER:  CcBbAa
  if (h==0b000001) {
    TCCR2A = 0;
    TCCR0A = 0;
    
    out = 0;
    out |= (0<<C);
    out |= (0<<c); // PWM Timer 0 A 
    out |= (0<<B);
    out |= (1<<b); // OFF
    out |= (1<<A);
    out |= (1<<a); // OFF

    PORTD = out; // 0b00011100; // C:-, B:0, A:+
    PORTB = 0b00000011;

    TCCR0A = 0b10000011;
  }
  if (h==0b000011) {
    TCCR2A = 0;
    TCCR0A = 0;
    
    out = 0;
    out |= (0<<C);
    out |= (0<<c); // PWM Timer 0 A 
    out |= (1<<B);
    out |= (1<<b); // OFF
    out |= (0<<A);
    out |= (1<<a); // OFF
    
    PORTD = out; // 0b00110100;  // C:-, B:+, A:0
    PORTB = 0b00000011;

    TCCR0A = 0b10000011;
  }
  if (h==0b000010) {
    TCCR2A = 0;
    TCCR0A = 0;
    
    out = 0;
    out |= (0<<C);
    out |= (1<<c); // OFF
    out |= (1<<B);
    out |= (1<<b); // OFF
    out |= (0<<A);
    out |= (0<<a); // PWM Timer 2 B
    
    //        CcBbAa
    PORTD = out; // 0b01110000;  // C:0, B:+, A:-
    PORTB = 0b00000110;

    TCCR2A = 0b00100011;
  }
  if (h==0b000110) {
    TCCR2A = 0;
    TCCR0A = 0;
    
    out = 0;
    out |= (1<<C);
    out |= (1<<c); // OFF
    out |= (0<<B);
    out |= (1<<b); // OFF
    out |= (0<<A);
    out |= (0<<a); // PWM Timer 2 B
    
    //        CcBbAa
    PORTD = out; // 0b11010000;  // C:+, B:0, A:-
    PORTB = 0b00000110;

    TCCR2A = 0b00100011;
  }
  if (h==0b000100) {
    TCCR2A = 0;
    TCCR0A = 0;
    
    out = 0;
    out |= (1<<C);
    out |= (1<<c); // OFF
    out |= (0<<B);
    out |= (0<<b); // PWM Timer 0 B
    out |= (0<<A);
    out |= (1<<a); // OFF
    
    //        CcBbAa
    PORTD = out; // 0b11000100;  // C:+, B:-, A:0
    PORTB = 0b00000101;

    TCCR0A = 0b00100011;
  }
  if (h==0b000101) {
    TCCR2A = 0;
    TCCR0A = 0;
    
    out = 0;
    out |= (0<<C);
    out |= (1<<c); // OFF
    out |= (0<<B);
    out |= (0<<b); // PWM Timer 0 B
    out |= (1<<A);
    out |= (1<<a); // OFF
    
    //        CcBbAa
    PORTD = out; // 0b01001100;  // C:0, B:-, A:+
    PORTB = 0b00000101;

    TCCR0A = 0b00100011;
  }

  
}
