#include "Arduino.h"
#include "ArduinoBLE.h"
#include "Wire.h"
#include "const.h"
#include "sensor.h"
#include "pid.h"
#include "motor.h"
#include "comm.h"
SensorController sensor(3);
MotorController motor;
BluetoothController bt;

void setup() {
    Serial.begin(115200);

    Serial.println("Initializing...");

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
    Serial.println("Finished initializing pins.");

    sensor.init();
    Serial.println("Finished initializing sensors.");

    motor.init();
    Serial.println("Finished initializing motors.");

    bt.init();
    Serial.println("Finished initializing bluetooth.");
}

void linearmotor_test() {
    for (int s = -250; s <= 250; s += 20) {
        if (abs(s) < 30) {
            continue;
        }
        motor.resetEncs();
        motor.setSpeed(s, s);
        delay(2000);
        motor.read();
        // Serial.print(s);
        // Serial.print(",");
        // Serial.print(motor.getEncL());
        // Serial.print(",");
        // Serial.println(motor.getEncR());
        motor.setSpeed(0, 0);
        delay(500);
    }
}

/*
void drive_straight(double dist, double defspeed) {
    Serial.println("STRAIGHT");
    wheelPID.resetI();
    motor.resetEncs();
    double L, R, diff = 0;
    long int t = millis();
    while (motor.getEncL() < dist) {
        motor.read();
        L = motor.getEncL();
        R = motor.getEncR();
        diff = L - R;
        long int ct = millis();
        int dt = ct - t + 1;
        double op = wheelPID.feedback(diff, dt);
        motor.setSpeed(defspeed, op + defspeed);
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
    motor.setSpeed(0, 0);
}

*/

/*
void turn(double angle, double angular) {
    // angle -pi to pi CCW
    Serial.println("TURN");
    wheelPID.resetI();
    motor.resetEncs();
    double L, R, diff = 0;
    long int t = millis();
    double angle_dist = BASE_WIDTH / 2 * angle;
    int flip = 1;
    if (angle < 0) {
        flip = -1;
    }

    while (abs(motor.getEncL()) < abs(angle_dist)) {
        motor.read();
        L = motor.getEncL();
        R = motor.getEncR();
        diff = L + R;
        long int ct = millis();
        int dt = ct - t + 1;
        double op = wheelPID.feedback(diff, dt);
        motor.setSpeed(-angular * flip, -op + angular * flip);
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
    motor.setSpeed(0, 0);
}
*/


void IR_test() {
    sensor.frontOn();
    delay(10);
    int *newdat = sensor.readAll();
    sensor.allOff();
    sensor.push(newdat);
    double *newsmooth = sensor.expSmooth(.1);

    for (int i = 0; i < 2; i++) {
        Serial.print(newdat[i]);
        Serial.print(",");
        Serial.print(newsmooth[i]);
        Serial.print(",");
    }
    Serial.println();
}

void LF_test(bool a) {
    motor.resetEncs();
    int s = 0;
    if (a) {
        s = -50;
    } else {
        s = 50;
    }
    motor.setSpeed(s, s);
    while (1) {
        if (a) {
            if (motor.getEncL() < -1075) {
                break;
            }
        } else {
            if (motor.getEncL() > 1075) {
                break;
            }
        }
        motor.read();
        Serial.print(motor.getEncL());
        Serial.print(",");
        Serial.print(motor.getEncR());
        Serial.print(",");
        sensor.allOn();
        int *newdat = sensor.readAll();
        sensor.allOff();
        sensor.push(newdat);
        double *newsmooth = sensor.expSmooth(.1);
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
    motor.setSpeed(0, 0);
}

void loop() {
    // OLD CODE
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

    // sensor.readIMU();
    // END OF OLD CODE


    if (!motor.isInMotion()) {
        // send in the next task
        motor.driveStraight(18, 50);
    }
    
    // workflow:
    // readings = sensor.read();
    // motor.consume(readings);
    motor.control();
    bt.publish(String("millis"));
}