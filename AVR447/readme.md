### AVR447 port to AVR-GCC (compiles but not yet tested)

To compile:

    avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o main.o main.c
    avr-gcc -mmcu=atmega328p main.o -o main
    avr-objcopy -O ihex -R .eeprom main main.hex
