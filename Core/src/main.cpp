/*
 * This should be enough to test that everything is working properly. Make sure to check that the builtin
 * light blinks on the arduino and the the serial output is working. See the README for how to do this.
 */
#include "Arduino.h"
#include "sensor.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif



void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);

  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  IR ir_sensor;
  ir_sensor.print_test();

  delay(1000);
}