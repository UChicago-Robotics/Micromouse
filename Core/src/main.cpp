/*
 * This should be enough to test that everything is working properly. Make sure to check that the builtin
 * light blinks on the arduino and the the serial output is working. See the README for how to do this.
 */
#include "Arduino.h"
#include "const.h"
#include "sensor.h"
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

void setup() {
    for (auto i : IR_INPUT_PINS)
        pinMode(i, INPUT);

    for (auto i : IR_OUTPUT_PINS)
        pinMode(i, OUTPUT);

    Serial.begin(115200);
}

void loop() {
    // digitalWrite(LED_BUILTIN, HIGH);

    // delay(1000);
    // digitalWrite(LED_BUILTIN, LOW);

    // IR ir_sensor;
    // ir_sensor.print_test();

    // for (auto input_pin : IR_INPUT_PINS){
    //     digitalWrite(input_pin, HIGH);
    // }
    digitalWrite(D9, HIGH);
    digitalWrite(D10, HIGH);
    digitalWrite(D11, HIGH);
    digitalWrite(D12, HIGH);

    IR ir_sensor;
    ir_sensor.print_test();
    for (auto output_pin : IR_OUTPUT_PINS){
        int val = analogRead(output_pin);
        Serial.print(val);
        Serial.print(" ");
    }
    Serial.println();
    delay(50);
}