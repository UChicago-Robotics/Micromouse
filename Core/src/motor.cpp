#include <stdlib.h>
#include <math.h>

#include "motor.h"
#include "Arduino.h"
#include "const.h"
#include "Encoder.h"

// constants measured from bot
const double ticks_per_rev = 360;
const double wheel_diam = 3.2;
const double wheel_circ = M_PI * 3.2;
const double dist_between_treads = 9.25;

const double turn_diam = dist_between_treads * 2;
const double turn_circ = M_PI * turn_diam;



void motor::turn_right(double angle) {
    Serial.println("Turning Right...");
    Encoder enc(RR_ENC, RL_ENC); 

    // based on angle, calculate distance (arc length) for pivot
    double dist = (angle / 360.0) * turn_circ;
    Serial.println(dist);

    // calculate number of wheel revolutions
    double num_rev = dist / wheel_circ;

    // calculate number of encoder ticks for turn
    int enc_ticks = num_rev * ticks_per_rev;
    int start_enc = enc.read();
    int c = 0;

    analogWrite(RF_MOTOR, 256);

    Serial.print(enc.read() - start_enc);
    Serial.print("  ");
    Serial.println(enc_ticks);

    while (((c = enc.read()) - start_enc) < enc_ticks) {
        delay(1); // maybe not necessary?
    }

    analogWrite(RF_MOTOR, 0);
    Serial.println("Done");
}

