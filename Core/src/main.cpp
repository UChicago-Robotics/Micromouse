/*
 * This should be enough to test that everything is working properly. Make sure to check that the builtin
 * light blinks on the arduino and the the serial output is working. See the README for how to do this.
 */
#include "Arduino.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);

  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("test\n");
  delay(1000);
}