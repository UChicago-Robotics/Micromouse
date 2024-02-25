#include "Arduino.h"
#include "const.h"
//#include "sensor.h"
#include "LSM9DS1.h"

#define HIGH 1
#define LOW 0

//This will run only one time.
void setup(){
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Started");

    //Set pins as outputs
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);
    pinMode(D4, OUTPUT);
    pinMode(D5, OUTPUT);
}


void loop(){

    delay(1000);
    Serial.println("Motor Test");
    digitalWrite(D2, HIGH);
    digitalWrite(D3, LOW);

    digitalWrite(D4, HIGH);
    digitalWrite(D5, LOW);
}
