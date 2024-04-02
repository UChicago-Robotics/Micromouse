#include <Wire.h>
#include "Arduino.h"
#include "const.h"
#include "sensor.h"
#include "motor.h"
#include "comm.h"
#include "PID.h"
#include "madgwickFilter.h"
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
SensorData SH(3);
PID wheelPID(150, 0, 0); // stupid but works - may require tuning later at high speeds etc, testing at 50

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;

    if (!BLE.begin())
    {
        // If the BLE module fails to initialize, enter an infinite loop
        while (1)
        {
            Serial.println("BAD");
        }
    }

    // Initialize the IMU
    if (!IMU.begin())
    {
        // If the IMU fails to initialize, enter an infinite loop
        while (1)
        {
            Serial.println("IMU BAD");
        }
    }
    Serial.println("Started");

    // Motors
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);
    pinMode(D4, OUTPUT);
    pinMode(D5, OUTPUT);
    // IR Sensors
    pinMode(RF_IRo, OUTPUT);
    pinMode(RR_IRo, OUTPUT);
    pinMode(LF_IRo, OUTPUT);
    pinMode(LL_IRo, OUTPUT);
    pinMode(RF_IRi, INPUT);
    pinMode(RR_IRi, INPUT);
    pinMode(LL_IRi, INPUT);
    pinMode(LF_IRi, INPUT);

    bt_setup();
}

void printEncs()
{
}

void linearmotor_test()
{
    for (int s = -250; s <= 250; s += 20)
    {
        if (abs(s) < 30)
        {
            continue;
        }
        motor::resetEncs();
        motor::setSpeed(s, s);
        delay(2000);
        motor::read();
        // Serial.print(s);
        // Serial.print(",");
        // Serial.print(motor::get_LEnc());
        // Serial.print(",");
        // Serial.println(motor::get_REnc());
        motor::setSpeed(0, 0);
        delay(500);
    }
}

void drive_straight(double dist, double defspeed)
{
    Serial.println("STRAIGHT");
    wheelPID.resetI();
    motor::resetEncs();
    double L, R, diff = 0;
    long int t = millis();
    while (motor::get_LEnc() < dist)
    {
        motor::read();
        L = motor::get_LEnc();
        R = motor::get_REnc();
        diff = L - R;
        long int ct = millis();
        int dt = ct - t + 1;
        double op = wheelPID.feedback(diff, dt);
        motor::setSpeed(defspeed, op + defspeed);
        t = ct;
        Serial.print(ct);
        Serial.print("\t");
        Serial.print(dt);
        Serial.print("\t");
        Serial.print(L);
        Serial.print("\t");
        Serial.print(R);
        Serial.print("\t");
        Serial.print(diff);
        Serial.print("\t");
        Serial.println(op);
    }
    motor::setSpeed(0, 0);
}

void turn(double angle, double angular)
{
    // angle -pi to pi CCW
    Serial.println("TURN");
    wheelPID.resetI();
    motor::resetEncs();
    double L, R, diff = 0;
    long int t = millis();
    double angle_dist = motor::dist_between_treads / 2 * angle;
    int flip = 1;
    if (angle < 0)
    {
        flip = -1;
    }

    while (abs(motor::get_LEnc()) < abs(angle_dist))
    {
        motor::read();
        L = motor::get_LEnc();
        R = motor::get_REnc();
        diff = L + R;
        long int ct = millis();
        int dt = ct - t + 1;
        double op = wheelPID.feedback(diff, dt);
        motor::setSpeed(-angular * flip, -op + angular * flip);
        t = ct;
        Serial.print(ct);
        Serial.print("\t");
        Serial.print(dt);
        Serial.print("\t");
        Serial.print(L);
        Serial.print("\t");
        Serial.print(R);
        Serial.print("\t");
        Serial.print(diff);
        Serial.print("\t");
        Serial.println(op);
    }
    motor::setSpeed(0, 0);
}

void IR_test()
{
    SH.frontOn();
    delay(10);
    int *newdat = SH.readAll();
    SH.allOff();
    SH.push(newdat);
    double *newsmooth = SH.expSmooth(.1);

    for (int i = 0; i < 2; i++)
    {
        Serial.print(newdat[i]);
        Serial.print(",");
        Serial.print(newsmooth[i]);
        Serial.print(",");
    }
    Serial.println();
}

void LF_test(bool a)
{
    motor::resetEncs();
    int s = 0;
    if (a)
    {
        s = -50;
    }
    else
    {
        s = 50;
    }
    motor::setSpeed(s, s);
    while (1)
    {
        if (a)
        {
            if (motor::get_LEnc() < -1075)
            {
                break;
            }
        }
        else
        {
            if (motor::get_LEnc() > 1075)
            {
                break;
            }
        }
        motor::read();
        Serial.print(motor::get_LEnc());
        Serial.print(",");
        Serial.print(motor::get_REnc());
        Serial.print(",");
        SH.allOn();
        int *newdat = SH.readAll();
        SH.allOff();
        SH.push(newdat);
        double *newsmooth = SH.expSmooth(.1);
        Serial.print(newdat[0]);
        Serial.print(",");
        Serial.print(newsmooth[0]);
        Serial.print(",");
        Serial.print(newdat[3]);
        Serial.print(",");
        Serial.print(newsmooth[3]);
        Serial.print(",");
        Serial.println();
        delay(20);
    }
    motor::setSpeed(0, 0);
}

void loop()
{

    // motor_test();

    // IR_test();
    // delay(100);

    // LF_test(0);
    // LF_test(1);

    // drive_straight(18, 60);
    // delay(500);
    // turn(PI / 2, 90);
    // delay(500);
    // drive_straight(18, 60);
    // delay(500);
    // turn(-PI / 2, 90);
    // delay(2000);

    // SH.readIMU();
    bt_loop(String(millis()));
}