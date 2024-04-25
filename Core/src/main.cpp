#include "Arduino.h"
#include "ArduinoBLE.h"
#include "Wire.h"
#include "const.h"
#include "sensor.h"
#include "pid.h"
#include "motor.h"
#include "comm.h"
// #include "madgwickFilter.h"

SensorController sensor(3,.1);
MotorController motor;
int cells = 0;
// BluetoothController bt;

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
    // mode button
    pinMode(BUTTON, INPUT);
    Serial.println("Finished initializing pins.");

    sensor.init();
    Serial.println("Finished initializing sensors.");

    motor.init();
    Serial.println("Finished initializing motors.");

    // bt.init();
    Serial.println("Finished initializing bluetooth.");

    //TODO make based on when switch flipped
    sensor.resetWallBase();
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
        Serial.print(",");
        Serial.print(dt);
        Serial.print(",");
        Serial.print(L);
        Serial.print(",");
        Serial.print(R);
        Serial.print(",");
        Serial.print(diff);
        Serial.print(",");
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
        Serial.print(",");
        Serial.print(dt);
        Serial.print(",");
        Serial.print(L);
        Serial.print(",");
        Serial.print(R);
        Serial.print(",");
        Serial.print(diff);
        Serial.print(",");
        Serial.println(op);
    }
    motor.setSpeed(0, 0);
}
*/

std::array<bool, 3> readCurrentWalls() {
    // Example logic to generate boolean values
    sensor.read();
    sensor.push();
    bool L = (sensor.getLLs() > .75*sensor.getBaseL());
    bool F = (sensor.getLFs() + sensor.getRFs() >  450); //TODO
    bool R = (sensor.getRRs() > .75*sensor.getBaseR());
    return {L,F,R};
}

bool controlMaze() {
    // motor.control but with wall sensing
    // drives straight with encoder target while actively avoiding walls
    // should be able to handle presence and absence of walls
    // returns whether at target
    motor.read();
    double l = motor.getEncL();
    double r = motor.getEncR();
    double L = motor.getTargetL();
    if (l < L) {
        // motor differential
        double diffEnc = l - r;
        // wall differential
        sensor.read();
        sensor.push();
        float diffLWall = sensor.getLLs()-sensor.getBaseL();
        float diffRWall = sensor.getRRs()-sensor.getBaseR();

        float LWcontrib = 0;
        if (diffLWall > sensor.getLLCut()) {
            LWcontrib = diffLWall*sensor.getLLCoeff();
        }
        float RWcontrib = 0;
        if (diffRWall > sensor.getRRCut()) {
            RWcontrib = diffRWall*sensor.getRRCoeff();
        } 
        float totalDiff = diffEnc - LWcontrib + RWcontrib;
        
        long int ct = millis();
        int dt = ct - motor.getLastRun() + 1;
        double op = motor.PIDfeedback(totalDiff, dt);
        double ls = motor.getBaseSpeed();
        double rs = ls + op;
        
        // ramps down speed linearly until target
        // https://www.desmos.com/calculator/50fkyhu6ek
        // double frac = .8;
        // double lowspeed = motor.getMinSpeed();
        // if (l > frac*L) {
        //     ls = (lowspeed-ls)*l/(L*(1-frac)) + lowspeed - (lowspeed - ls)/(1-frac);
        //     rs = (lowspeed-rs)*l/(L*(1-frac)) + lowspeed - (lowspeed - rs)/(1-frac);
        // }

        motor.setSpeed(ls,rs);
        motor.setLastRun(ct);
        Serial.print("t");
        Serial.print(ct);
        Serial.print(",dt ");
        Serial.print(dt);
        Serial.print(",l ");
        Serial.print(l);
        Serial.print(",r ");
        Serial.print(r);
        Serial.print(",sL ");
        Serial.print(diffLWall);
        Serial.print(",sR ");
        Serial.print(diffRWall);
        Serial.print(",dE ");
        Serial.print(diffEnc);
        Serial.print(",dL ");
        Serial.print(LWcontrib);
        Serial.print(",dR ");
        Serial.print(RWcontrib);
        Serial.print(",dT ");
        Serial.print(totalDiff);
        Serial.print(",op ");
        Serial.println(op);
        return false; // ie not there yet
    }
    else {
        motor.setMotion(false);
        motor.setSpeed(0, 0);
        return true; // ie there
    }
}

void turnDeg(double deg) { // positive = CCW
    double angle = 0;
    long int hold_time = micros();
    double turn_speed = 45; // anything less stalls it
    double prefact = -1;
    if (deg < 0) {prefact = 1;}
    motor.setSpeed(turn_speed*prefact,turn_speed*prefact*-1);
    while (abs(angle) < abs(deg*.7)) {
        sensor.readIMU();
        long int curr_time = micros();
        angle += (float)(.000001)*sensor.getGz()*(curr_time-hold_time);
        hold_time = curr_time;
    }
    motor.setSpeed(0,0);
}

void loop() {
    // <OLDCODE>
    // linear_motor_test();
    // <\OLDCODE>

    //full sensor printout dump
    // sensor.read();
    // sensor.push();
    // sensor.readIMU();
    // Serial.print(sensor.dumpString());
    // motor.read();
    // Serial.print(",");
    // Serial.print(motor.getEncL());
    // Serial.print(",");
    // Serial.println(motor.getEncR());

    //partial IMU dump
    // sensor.readIMU();
    // Serial.print(",");
    // Serial.print(sensor.getAz());
    // Serial.print(",");
    // Serial.println(sensor.getGz());
    // delay(100);

    // driving

    if (!motor.isInMotion()) {
        motor.driveStraight(15, 50);
    }
    // workflow:
    if (controlMaze()) {
        delay(1000);
        cells++;
        // dump walls
        std::array<bool,3> currentWalls = readCurrentWalls();
        for (bool b : currentWalls) {
            Serial.print(b);
            Serial.print(",");
        }
        Serial.println();
        if (cells == 3) {
            turnDeg(-90);
        }
    }



    // bt.publish(String("millis"));
}