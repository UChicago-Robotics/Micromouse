#include <Arduino_LSM9DS1.h>
#include <Encoder.h>
#include "pins_arduino.h"
#include "Arduino.h"

// Change these two numbers to the pins connected to your encoder
// or shift register circuit which emulates a quadrature encoder
//  case 1: both pins are interrupts
//  case 2: only first pin used as interrupt
Encoder myEnc(5, 6);

// Connect a DC voltmeter to this pin.
const int outputPin = 12;

/* This simple circuit, using a Dual Flip-Flop chip, can emulate
   quadrature encoder signals.  The clock can come from a fancy
   function generator or a cheap 555 timer chip.  The clock
   frequency can be measured with another board running FreqCount
   http://www.pjrc.com/teensy/td_libs_FreqCount.html

                        +5V
                         |        Quadrature Encoder Signal Emulator
 Clock                   |
 Input o----*--------------------------      ---------------------------o Output1
            |            |14           |    |
            |     _______|_______      |    |     _______________ 
            |    |    CD4013     |     |    |    |    CD4013     |
            |  5 |               | 1   |    |  9 |               | 13
        ---------|  D         Q  |-----|----*----|  D         Q  |------o Output2
       |    |    |               |     |         |               |
       |    |  3 |               |     |      11 |               |
       |     ----|> Clk          |      ---------|> Clk          |
       |         |               |               |               |
       |       6 |               |             8 |               |
       |     ----|  S            |           ----|  S            |
       |    |    |               |          |    |               |
       |    |  4 |            _  | 2        | 10 |            _  | 12
       |    *----|  R         Q  |---       *----|  R         Q  |----
       |    |    |               |          |    |               |    |
       |    |    |_______________|          |    |_______________|    |
       |    |            |                  |                         |
       |    |            | 7                |                         |
       |    |            |                  |                         |
        --------------------------------------------------------------
            |            |                  |
            |            |                  |
          -----        -----              -----
           ---          ---                ---
            -            -                  -
*/


void setup() {
  pinMode(outputPin, OUTPUT);
}

#if defined(__AVR__) || defined(TEENSYDUINO)
#define REGTYPE unsigned char
#else
#define REGTYPE unsigned long
#endif

void loop() {
  volatile int count = 0;
  volatile REGTYPE *reg = portOutputRegister(digitalPinToPort(outputPin));
  REGTYPE mask = digitalPinToBitMask(outputPin);

  while (1) {
    myEnc.read();	// Read the encoder while interrupts are enabled.
    noInterrupts();
    *reg |= mask;	// Pulse the pin high, while interrupts are disabled.
    count = count + 1;
    *reg &= ~mask;
    interrupts();
  }
}

