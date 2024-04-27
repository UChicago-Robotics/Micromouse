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
#define DEBUGGING true

SensorController sensor(3, 0.1);
MotorController motor;
// BluetoothController bt;
std::vector<int> nav = {0,1,1,0,-1,0,-1,0,0};
int cells = 0;
int last = 0;
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

    // bt.init();
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
    bool L = (sensor.getLLs() > .85*sensor.getBaseL());
    bool F = (sensor.getLFs() + sensor.getRFs() >  .65*(sensor.getLFCut() + sensor.getRFCut())); //TODO
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
    while (abs(angle) < abs(deg*.5)) { // TODO figure out why angle fucked
        sensor.readIMU();
        long int curr_time = micros();
        angle += (float)(.000001)*sensor.getGz()*(curr_time-hold_time);
        hold_time = curr_time;
    }
    motor.setSpeed(0,0);
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
    // Serial.print(",");
    // Serial.print(sensor.getAz());
    // Serial.print(",");
    // Serial.println(sensor.getGz());
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
    sensor.readIMU();
    // Serial.println(sensor.dumpIMUString());
    motor.yaw = sensor.mahony.yaw * 57.29578f;
    int currTime = millis();
    if (!motor.isInMotion() & currTime - last > 5000) {
        motor.turnToYaw(90); // TODO FIGURE OUT WHY DISTANCE FUCKED
    }
    motor.control();
    // bt_loop(String(millis()));
}