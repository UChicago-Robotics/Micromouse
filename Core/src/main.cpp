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
#include "algo.h"
#include <vector>
#include <stack>
#define DEBUGGING false
struct Task {
    Instruction instr;
    double value;
};

SensorController sensor(3, 0.1);
MotorController motor;
int pickedup_hysteresis = 10;
int pickedup_counter = 0;
bool picked = false;
// BluetoothController bt;
std::vector<int> nav = {0,1,1,0,-1,0,-1,0,0};
int cells = 0;
int lastTime = 0;
bool justTurned = false;
bool preturned = true; // false if needs to adjust, true if good
void setup() {
    Serial.begin(115200);

    // RED: BEFORE SET UP
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);

    delay(2000);
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
    // buttons
    pinMode(ONOFF, INPUT_PULLDOWN); // wired to be a pulldown of ONE of the batteries
    pinMode(BUTTON_1, INPUT_PULLDOWN);
    pinMode(BUTTON_2, INPUT_PULLDOWN);

    Serial.println("Finished initializing pins.");
    sensor.init();
    Serial.println("Finished initializing sensors.");
    motor.init();
    Serial.println("Finished initializing motors.");
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
    Serial.println("READY");
}

Wall readCurrentWalls() {
    // Example logic to generate boolean values
    // 1 = wall there, 0 = not
    sensor.read();
    sensor.push();
    int L = sensor.isLWall();
    int F = sensor.isFWall();
    int R = sensor.isRWall();
    return {L,F,R};
}

int buttonMode() {
    // 0 = OFF (0xx)
    // +1 = mapping (10x)
    // +2 = racing (11x)
    // +0 = algo 1 (1x0)
    // +10 = algo 2 (1x1)
    int buttonmode = 0;
    if (digitalRead(ONOFF)) {
        Serial.println("ON");
        if (digitalRead(BUTTON_1)) {
            Serial.println("\t\tMAPPING");
            buttonmode+=1;
        } else {
            Serial.println("\t\tRACING");
            buttonmode+=2;
        }
        if (digitalRead(BUTTON_2)) {
            Serial.println("\t\t\t\tALGO 1"); // place holder
        } else {
            Serial.println("\t\t\t\tALGO 2"); // place holder
            buttonmode+=10;
        }
    } else {
        Serial.println("OFF");
    }
    return buttonmode;
}

bool pickedup() {
    // reads gyro and makes sure is on the ground WITH HYSTERESIS HANDLING
    if (fabs(sensor.getAz()-1) > .03 && fabs(sensor.getAx()) > .03 && fabs(sensor.getAy()) > .03) { // transient pickup
        if (!picked) {
            pickedup_counter++;
            if (pickedup_counter > pickedup_hysteresis) {
                picked = true;
            }
        } else {
            pickedup_counter = 0;
        }  
    } else {
        if (picked) {
            pickedup_counter++;
            if (pickedup_counter > pickedup_hysteresis) {
                picked = false;
            }
        } else {
            pickedup_counter = 0;
        }  
    }
    if (picked) {
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, HIGH);
    } else {
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, HIGH);
    }
    return picked;
}

std::stack<Task> taskstack;
// turning thresholds
int currentTheoreticalYaw = 0;
const double TURN_THRESHOLD = 2;
int stableCount = 0;
int method = 1;
double cumYaw = 0;
// handling rollover on long straights
double addon = 0;
double last_l = 0;
int stabilisedL = 1;
// handling gap calculation
double lastLW = -1; // < 0 means currently is a wall
int wasLastLW = 0;
double lastRW = -1;
int wasLastRW = 0;


void printstr(String s){
    Serial.println(s);
    bt_loop("<"+String(millis()) + ">"+s);
}
String stackstr(std::stack<Task> s) {
    String out = "STACK:";
    while (!s.empty()) {
        Task pstr = s.top();
        out += "(";
        switch(pstr.instr) {
            case DRIVE_STRAIGHT:
                out += "STRAIGHT";
                break;
            case TURN_LEFT:
                out += "LEFT";
                break;
            case TURN_RIGHT:
                out += "RIGHT";
                break;
        }
        out += "," + String(pstr.value) + "),";
        s.pop();
    }
    return out;
}

void control() {
    if (motor.isInMotion()) {
        // only do things when inMotion
        if (motor.isInTurn()) {
            switch (method) {
                case 0: // time based
                    {
                        long int start_time = millis();
                        int coef = (motor.getTargetYaw() > 0) ? 1 : -1;
                        motor.setSpeed(- coef * motor.mturnSpeed(), coef * motor.mturnSpeed());
                        while (millis()-start_time < motor.getTurnTime()) {
                        }
                        motor.setSpeed(0,0);
                        motor.setInMotion(false);
                        motor.setInTurn(false);
                        break;
                    }   
                case 1: // gyro based "turndeg"
                    {
                        long int hold_time = micros();
                        double turn_speed = 55; // anything less stalls it
                        double prefact = -1;
                        if (motor.getTargetYaw() < 0) {prefact = 1;}
                        motor.setSpeed(turn_speed*prefact,turn_speed*prefact*-1);
                        float tYawF = motor.getTargetYaw(); // final target
                        float tYaw = tYawF*.27; // when to start coasting
                        if (fabs(cumYaw) < fabs(tYaw)) { 
                            sensor.readIMU();
                            long int curr_time = micros();
                            cumYaw += (float)(.000001)*sensor.getGz()*(curr_time-hold_time);
                            hold_time = curr_time;
                            printstr("TargYaw: " + String(motor.getTargetYaw(), 2) + " Cum Yaw: " + String(cumYaw, 2));
                        } else {
                            motor.setSpeed(0,0);
                        // } TODO
                        // if (fabs(cumYaw) > fabs(.9*tYawF)) { // TODO needs compensation break
                            cumYaw = 0;
                            motor.setInMotion(false);
                            motor.setInTurn(false);
                        }
                        break;
                    }
                case 2: // "PID" "stablecount"
                    {
                        // turning motion
                        double diff = fmod((motor.getTargetYaw() - sensor.getYawDeg() + 720), 360);
                        int coef = (diff < 180) ? 1 : -1;
                        double dAngle = (diff < 180) ? diff : (360 - diff);
                        if ( fabs(dAngle) > TURN_THRESHOLD) {
                            stableCount = 0;
                            // if still out of tolerance range
                            long int ct = millis();
                            int dt = ct - motor.getLastRun() + 1;
                            motor.setSpeed(- coef * motor.mturnSpeed(), coef * motor.mturnSpeed());
                            printstr("dir: " + String(coef) + " diff: " + String(diff, 2) + " dt: " + String(dt));
                            printstr("TargYaw: " + String(motor.getTargetYaw(), 2) + " Yaw: " + String(sensor.getYawDeg(), 2) + "dir: " + String(coef) + " dAngle: " + String(dAngle, 2) + " dt: " + String(dt) + "gz: " + String(sensor.getGz(), 2) + "imu dt: " + String(sensor.runTime));
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
                        break;
                    }
            }
        } else {
            // driving motion
            motor.read();
            double l = motor.getEncL();
            double r = motor.getEncR();
            double Lfinal = motor.getTargetL();
            double threshdist = .66;
            double L = threshdist*Lfinal;
            sensor.read();
            sensor.push();
            // if there is currently a wall, keep "last Wall" at -1, if there is not a wall track when it disappeared (and then substract distance later to get the distance travelled past the wall)
            if (sensor.isLWall()) {
                lastLW = -1;
            } else if (lastLW < 0) {
                lastLW = l;
            }
            if (sensor.isRWall()) {
                lastRW = -1;
            } else if (lastRW < 0) {
                lastRW = l;
            }
            printstr("\t\t\t\tLW:" + String(lastLW,2) + " RW:" + String(lastRW,2));
            bool cond;
            // OLD SCHEME - gives equal weight to all 3 cases where might stop early
            // bool wallGapNotTriggered = (l - lastLW <= 3 || lastLW < 0) && (l - lastRW <= 3 || lastRW < 0); // ie last not initialized or not past cutoff ie 3 = 18*(threshdist-1/2) for both
            // bool frontWallNotTriggered = (l < .6 * L || sensor.getLFs() < sensor.getLFCut()) &&  (l < .6 * L || sensor.getRFs() < sensor.getRFCut()); // ie not close to wall or not at end for both
            // bool distanceNotTriggered = l < L; // not there yet
            // printstr("conditions: WallGap:" + String(wallGapNotTriggered) + " FrontWall:" + String(frontWallNotTriggered) + " Distance: " + String(distanceNotTriggered));
            // cond = wallGapNotTriggered && frontWallNotTriggered && distanceNotTriggered;
            // // NEW SCHEME - default to avoid crashes based on wall gap and front wall, otherwise distance
            if (lastLW > 0) {
                cond = l-lastLW < 3;
                printstr("Cond Left Gap:" + String(cond));
            } else if (lastRW > 0) {
                cond = l-lastRW < 3;
                printstr("Cond Right Gap:" + String(cond));
            } else if (sensor.getLFs() > sensor.getLFCut() || sensor.getRFs() > sensor.getRFCut()) {
                cond = false;
                printstr("Cond Front Wall:" + String(cond));
            } else {
                cond = l < L;
                printstr("Cond Distance:" + String(cond));
            }
            if (cond) {
                // only drive if l < L and it's not too close to the wall beyond X% of the way to the next
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
                // printstr("t " + String(ct) + ",dt " + String(dt) + ",l " + String(l, 2) + ", r " + String(r, 2) + ", sL " + String(diffLWall, 2) + ", sR " + String(diffRWall, 2) + ", dE " + String(diffEnc, 2) +  ", dL " + String(LWcontrib, 2) + ", dR " + String(RWcontrib, 2) + ", dT " + String(totalDiff, 2) + ", op " + String(op, 2));
                printstr(sensor.dumpIRString());
            } else {
                motor.setSpeed(0, 0);
                lastLW = -1;
                lastRW = -1;
                if (stabilisedL % 20 == 0) {
                    last_l = l;
                }
                ++stabilisedL;
                if (fabs(l-last_l) < .1) {//TODO way to get out of stop
                    motor.setInMotion(false);
                    // if straight, find how much off from target
                    addon = Lfinal - l;
                }
            } 
        }
        printstr(sensor.dumpIRString());
    }
}

void loop() {

    sensor.readIMU();
    // Serial.println("\t" + String(buttonMode()));
    // Serial.println("\t\t\t"+ String(pickedup()));
    BLE.poll();
    int currTime = millis();
    if (!motor.isInMotion() && currTime - lastTime > 1200) {
        // if stack empty read walls
        // do stack

        // read walls
        // floodfill
        // --> left turn
            // push straight 18
            // push left turn
            // push adjustment if necessary
        // --> right turn
            // push straigh 18
            // push right turn
            // push adjustment if necessary
        // --> straight
            // push straight + addon
        Wall currentWalls = readCurrentWalls();
        printstr("addon: " + String(addon,2) + " lastLW:" + String(lastLW,2)+ " lastRW:" + String(lastRW,2) + " wasLastLW:" + String(wasLastLW) + " wasLastRW:" + String(wasLastRW));
        if (taskstack.empty()) {
            printstr("<<<<<<DECISION>>>>>>");
            printstr("If there's wall: L:" + String(currentWalls.left) + "\tF:" + String(currentWalls.front) + "\tR:" + String(currentWalls.right));
            if (!currentWalls.left) {
                printstr("Left Turn Pushed");
                taskstack.push({DRIVE_STRAIGHT,18});
                taskstack.push({TURN_LEFT,-1});
            } else if (!currentWalls.right) {
                printstr("Right Turn Pushed");
                taskstack.push({DRIVE_STRAIGHT,18});
                taskstack.push({TURN_RIGHT,-1});
            } else if (!currentWalls.front) { 
                printstr("Straight Pushed");
                taskstack.push({DRIVE_STRAIGHT,18+addon});
            } else {
                printstr("Turning Around Pushed");
                taskstack.push({TURN_LEFT,-1});
                taskstack.push({TURN_LEFT,-1});
            }
            // but first if front wall too close move back
            printstr("FRONTWALL THINGY: " + String(sensor.CFWall(),3));
            if (currentWalls.front && sensor.CFWall() > 100) {
                taskstack.push({DRIVE_STRAIGHT,sensor.CFWall()*(-.025)});
            }
            addon = 0;
        }
        wasLastLW = currentWalls.left;
        wasLastRW = currentWalls.right;
        printstr(stackstr(taskstack));
        Task instruction = taskstack.top();
        if (instruction.instr == DRIVE_STRAIGHT) {
            printstr("Driving Straight (" + String(instruction.value,2) + ")...");
            motor.driveStraight(instruction.value,50);
            motor.setSpeed(motor.mdriveSpeed()*1.1,motor.mdriveSpeed()*1.1);
        } else if (instruction.instr == TURN_LEFT) {
            printstr("Turning Left...");
            motor.turnToYaw(90);
        } else {
            printstr("Turning Right...");
            motor.turnToYaw(-90);
        }
        taskstack.pop();
        delay(250);
        lastTime = currTime;
    }
    control();
}