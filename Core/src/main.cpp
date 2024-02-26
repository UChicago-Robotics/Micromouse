#include "Arduino.h"
#include "const.h"
//#include "sensor.h"
#include "LSM9DS1.h"
#include "motor.h"

//This will run only one time.
void setup(){
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Started");

    //Motors
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);
    pinMode(D4, OUTPUT);
    pinMode(D5, OUTPUT);
    //IR Sensors
    pinMode(RF_IRo,OUTPUT);
    pinMode(RR_IRo,OUTPUT);
    pinMode(LF_IRo,OUTPUT);
    pinMode(LL_IRo,OUTPUT);
    pinMode(RF_IRi,INPUT);
    pinMode(RR_IRi,INPUT);
    pinMode(LL_IRi,INPUT);
    pinMode(LF_IRi,INPUT);

    digitalWrite(RF_IRo,HIGH);
    digitalWrite(RR_IRo,HIGH);
    digitalWrite(LF_IRo,HIGH);
    digitalWrite(LL_IRo,HIGH);
}

void motor_test() {
    motor::turn_right(90.0);
    delay(2000);
}

void IR_test() {
    delay(50);
    // Serial.print(analogRead(LL_IRi));
    // Serial.print("   ");
    // Serial.print(analogRead(RR_IRi));
    Serial.print("   ");
    Serial.print(analogRead(LF_IRi));
    Serial.print("   ");
    Serial.print(analogRead(RF_IRi));
    Serial.println();
}

void loop(){
    motor_test();
}