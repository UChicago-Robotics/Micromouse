#include "Arduino.h"
#include "const.h"
//#include "sensor.h"
#include "LSM9DS1.h"

#define HIGH 1
#define LOW 0
#define RF_IRi A0
#define RR_IRi A1
#define LL_IRi A2
#define LF_IRi A3
#define RF_IRo D12
#define RR_IRo D11
#define LL_IRo D10
#define LF_IRo D9


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


void loop(){

    // delay(1000);
    // Serial.println("Motor Test");
    // digitalWrite(D2, LOW);
    // digitalWrite(D3, HIGH);

    // digitalWrite(D4, LOW);
    // digitalWrite(D5, HIGH);
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
