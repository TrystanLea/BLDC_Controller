# BLDC Controller

Learning about 3 phase brushless DC motor control with the eventual aim of building an ebike controller.

![1.jpg](images/1.jpg)

Video demo of basic arduino BLDC controller: [https://youtu.be/3LG14Kc8SRs](https://youtu.be/3LG14Kc8SRs)


Basic hall sensor reader and commutation driver:

    #define DEBUG 0

    void setup() {
      Serial.begin(115200);
      // Set digital pins 2 to 7 as outputs
      DDRD = DDRD | B11111100;
    }

    void loop() {
     
      byte b = PINC;
      if (b==0b000001) {
        if (DEBUG) Serial.println("p100");
        //        CCBBAA
        // PORTD = 0b01001000;
           PORTD = 0b00011100;
      }
      if (b==0b000011) {
        if (DEBUG) Serial.println("p110");
        //        CCBBAA
        // PORTD = 0b01100000;
           PORTD = 0b00110100;
      }
      if (b==0b000010) {
        if (DEBUG) Serial.println("p010");
        //        CCBBAA
        // PORTD = 0b00100100;
           PORTD = 0b01110000;
      }
      if (b==0b000110) {
        if (DEBUG) Serial.println("p011");
        //        CCBBAA
        // PORTD = 0b10000100;
           PORTD = 0b11010000;
      }
      if (b==0b000100) {
        if (DEBUG) Serial.println("p001");
        //         CCBBAA
        // PORTD = 0b10010000;
           PORTD = 0b11000100;
      }
      if (b==0b000101) {
        if (DEBUG) Serial.println("p101");
        //         CCBBAA
        // PORTD = 0b00011000;
           PORTD = 0b01001100;
      }
    }


