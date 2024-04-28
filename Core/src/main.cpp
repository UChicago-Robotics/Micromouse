#include "Arduino.h"
#include "ArduinoBLE.h"
#include "Arduino_BMI270_BMM150.h"
#include "Wire.h"
#include "comm.h"
#include "const.h"
#include "motor.h"
#include "pid.h"
#include "sensor.h"
#include "pid.h"
#include "motor.h"
#include "comm.h"
#include <vector>
#define DEBUGGING false

SensorController sensor(3, 0.1);
MotorController motor;
// BluetoothController bt;
std::vector<int> nav = {0,1,1,0,-1,0,-1,0,0};
int cells = 0;
int last = 0;
bool justTurned = false;
void setup() {
    Serial.begin(115200);

    // RED: BEFORE SET UP
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);

    while (DEBUGGING && !Serial) delay(50);
    Serial.println("Initializing...");

    // BLUE: SETTING UP
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, LOW);

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
    // bt_setup();
    // Serial.println("Finished initializing bluetooth.");

    // calibrating
    Serial.println("Calibrating IMU...");

    sensor.calibrate();
    sensor.resetWallBase();

    Serial.println("Finished calibrating IMU");

    // GREEN: DONE SET UP
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);

    bt_setup();
    //Serial.println("Finished initializing bluetooth.");

    //TODO make based on when switch flipped
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

std::array<bool, 3> readCurrentWalls() {
    // Example logic to generate boolean values
    sensor.read();
    sensor.push();
    bool L = (sensor.getLLs() - sensor.getBaseL() > -30);
    bool F = (sensor.getLFs() + sensor.getRFs() - sensor.getLFCut() - sensor.getRFCut() > 0); //TODO
    bool R = (sensor.getRRs() - sensor.getBaseR() > -30);
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
    sensor.read();
    sensor.push();
    if (l < L && (l < .85*L || sensor.getLFs() + sensor.getRFs() >  .95*(sensor.getLFCut() + sensor.getRFCut()))) {
        // only drive if l < L and it's not too close to the wall beyond 70% of the way to the next
        // motor differential
        double diffEnc = l - r;
        // wall differential

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
        double op = motor.wheelPIDfeedback(totalDiff, dt);
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
        motor.setInMotion(false);
        motor.setSpeed(0, 0);
        return true; // ie there
    }
}

int currentTheoreticalYaw = 0;
const double TURN_THRESHOLD = 2;
int stableCount = 0;
bool timeBased = true;
double turnAngle = 0;
void control() {
    if (motor.isInMotion()) {
        // only do things when inMotion
        if (motor.isInTurn()) {
            if (timeBased) {
                long int start_time = millis();
                int coef = (turnAngle > 0) ? 1 : -1;
                motor.setSpeed(- coef * motor.mturnSpeed(), coef * motor.mturnSpeed());
                while (millis()-start_time < motor.getTurnTime()) {
                }
                motor.setSpeed(0,0);
                motor.setInMotion(false);
                motor.setInTurn(false);
            } else{
            // turning motion
            double diff = fmod((motor.getTargetYaw() - sensor.getYawDeg() + 720), 360);
            int coef = (diff < 180) ? 1 : -1;
            double dAngle = (diff < 180) ? diff : (360 - diff);
            if ( abs(dAngle) > TURN_THRESHOLD) {
                stableCount = 0;
                // if still out of tolerance range
                long int ct = millis();
                int dt = ct - motor.getLastRun() + 1;
                motor.setSpeed(- coef * motor.mturnSpeed(), coef * motor.mturnSpeed());
                Serial.println("dir: " + String(coef) + " diff: " + String(diff, 2) + " dt: " + String(dt));
                bt_loop("TargYaw: " + String(motor.getTargetYaw(), 2) + " Yaw: " + String(sensor.getYawDeg(), 2) + "dir: " + String(coef) + " dAngle: " + String(dAngle, 2) + " dt: " + String(dt) + "gz: " + String(sensor.getGz(), 2) + "imu dt: " + String(sensor.runTime));
                motor.setLastRun(ct);
            } else {
                stableCount++;

                if (stableCount >= 2) {
                    motor.setInMotion(false);
                    motor.setInTurn(false);
                    motor.setSpeed(0, 0);
                    stableCount = 0;
                }
            }
            }
        } else {
            // driving motion
                motor.read();
                double l = motor.getEncL();
                double r = motor.getEncR();
                double L = motor.getTargetL();
                sensor.read();
                sensor.push();
                if (l < L) { // && (l < .85*L || sensor.getLFs() + sensor.getRFs() >  .95*(sensor.getLFCut() + sensor.getRFCut()))) {
                    // only drive if l < L and it's not too close to the wall beyond 70% of the way to the next
                    // motor differential
                    double diffEnc = l - r;
                    // wall differential

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
                    double op = motor.wheelPIDfeedback(totalDiff, dt);
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
                    Serial.println("t " + String(ct) + ",dt " + String(dt) + ",l " + String(l, 2) + ", r " + String(r, 2) + ", sL " + String(diffLWall, 2) + ", sR" + String(diffRWall, 2) + ", dE " + String(diffEnc, 2) +  ", dL " + String(LWcontrib, 2) + ", dR " + String(RWcontrib, 2) + ", dT " + String(totalDiff, 2) + ", op " + String(op, 2));
                }
                else {
                    motor.setInMotion(false);
                    motor.setSpeed(0, 0);
                }
        }
    }
}

void loop() {
    //full IR sensor printout dump
    // sensor.read();
    // sensor.push();
    // sensor.readIMU();
    // Serial.print(sensor.dumpIRString());
    // motor.read();
    // Serial.print(",");
    // Serial.print(motor.getEncL());
    // Serial.print(",");
    // Serial.println(motor.getEncR());

    //partial IMU dump
    // sensor.readIMU();
    // Serial.print(sensor.getAz());
    // Serial.print(",");
    // Serial.println(sensor.getGz());
    // Serial.println(sensor.getBAngle());
    // delay(100);

    // driving

    /*
    
    if (!motor.isInMotion()) {
        motor.driveStraight(14, 50); // TODO FIGURE OUT WHY DISTANCE FUCKED
        motor.setSpeed(motor.getMinSpeed()*1.1,motor.getMinSpeed()*1.1);
    }
    // workflow:
    if (controlMaze()) {
        delay(500);
        // dump walls
        std::array<bool,3> currentWalls = readCurrentWalls();
        for (int i = 0; i < 3; i ++) {
            Serial.print(currentWalls[i]);
            Serial.print(",");
        }
        Serial.println();
        //autonav
        if (!currentWalls[0]) {
            turnDeg(90); // left
            delay(250);
        } else if (!currentWalls[2]) {
            turnDeg(-90); // right
            delay(250);
        }
        //preset nav
        // if (nav[cells] == 1) { //right
        //     turnDeg(-90);
        //     delay(250);
        // } else if (nav[cells] == -1) {
        //     turnDeg(90);
        //     delay(250);
        // }
        // cells++;
    }
    */
    // sensor.readIMU();
    BLE.poll();
    int currTime = millis();
    if (!motor.isInMotion() && currTime - last > 2000) {
        if (justTurned) {
            motor.driveStraight(14, 50); 
            justTurned = false;
            motor.setSpeed(motor.mdriveSpeed()*1.1,motor.mdriveSpeed()*1.1);
        } else {
            std::array<bool,3> currentWalls = readCurrentWalls();
                for (int i = 0; i < 3; i ++) {
                    Serial.print(currentWalls[i]);
                    Serial.print(",");
                }
                Serial.println();
                if (!currentWalls[0]) {
                    turnAngle = 90;
                    //motor.turnToYaw(sensor.getYawDeg()+60);
                    motor.turnToYaw(0);
                    justTurned = true;
                    delay(250);
                } else if (!currentWalls[2]) {
                    turnAngle = -90;
                    // motor.turnToYaw(sensor.getYawDeg()-60);
                    motor.turnToYaw(0);
                    justTurned = true;
                    delay(250);
                } else {
                    motor.driveStraight(14, 50); 
                    motor.setSpeed(motor.mdriveSpeed()*1.1,motor.mdriveSpeed()*1.1);
                }
        }
        last = currTime;
    }
    control();
    // bt_loop(String(millis()));
}