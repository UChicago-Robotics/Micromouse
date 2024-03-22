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

void printEncs() {

}

void motor_test() {
    Serial.println("Resetting");
    for (int s = -250; s <= 250; s += 20) {
        motor::resetEncs();
        motor::setSpeed(s,s);
        delay(2000);
        motor::read();
        // Serial.print(s);
        // Serial.print(",");
        // Serial.print(motor::get_LEnc());
        // Serial.print(",");
        // Serial.println(motor::get_REnc());
        motor::setSpeed(0,0);
        delay(500);
    }
}

void IR_test() {
    delay(100);
    Serial.print(analogRead(LF_IRi));
    Serial.print(",");
    Serial.print(analogRead(LL_IRi));
    Serial.print(",");
    Serial.print(analogRead(RR_IRi));
    Serial.print(",");
    Serial.print(analogRead(RF_IRi));
    Serial.println("");
}

void LF_test() {
    motor::resetEncs();
    delay(15000);
    motor::setSpeed(-50, -50);
    while(motor::get_LEnc() > -1075) {
        motor::read();
        Serial.print(motor::get_LEnc());
        Serial.print(",");
        Serial.print(motor::get_REnc());
        Serial.print(",");
        Serial.println(analogRead(LF_IRi));
        delay(20);
    }
    motor::setSpeed(0,0);
}


void loop(){
    //motor_test();
    //IR_test();
    LF_test();
}