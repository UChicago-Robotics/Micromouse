#include "Arduino.h"
#include "const.h"
#include "sensor.h"
#include "LSM9DS1.h"

#define HIGH 1
#define LOW 0

//This will run only one time.
void setup(){
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Started");

    //Set pins as outputs
    pinMode(A6, OUTPUT);
    pinMode(A7, OUTPUT);

    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
}


void loop(){
    
    //This code will turn Motor B clockwise for 2 sec.
    Serial.println("Running Left Motor");
    digitalWrite(A6, HIGH);
    digitalWrite(A7, LOW);

    delay(1000);

    Serial.println("Rinning Right Motor");
    digitalWrite(A1, HIGH);
    digitalWrite(A2, LOW);

    delay(1000);
}
