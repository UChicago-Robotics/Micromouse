#include "Arduino.h"
#include "Encoder.h"
#include "const.h"
#include "pid.h"
#ifndef MOTOR_H
#define MOTOR_H
const double BASE_WIDTH = 9.25;
const double WHEEL_DIAM = 7;//3.3;
const double WHEEL_CIRC = M_PI * 7;//3.3;
const int TICKS_PER_REV = 360;
class MotorController {
    private:
        Encoder encR = Encoder(RR_ENC, RL_ENC);
        Encoder encL = Encoder(LR_ENC, LL_ENC);
        PIDController wheelPID = PIDController(10, 0, 0);  // stupid but works - may require tuning later at high speeds etc, testing at 50
        PIDController turnPID = PIDController(0.3, 0, 0);
        int encLTicks = 0;
        int encRTicks = 0;
        int encLStart = 0;
        int encRStart = 0;
        double targL = 0;
        double targR = 0;
        double targYaw = 0;
        long int lastRun = 0;
        int baseSpeed = 0;
        double driveMinSpeed = 30;
        double turnMinSpeed = 45;
        double yaw = 0;
        bool inMotion = false;
        bool isTurn = false;
        float turnTime = 230; // ms
    public:
        MotorController();
        void init();
        void setSpeed(double l, double r);
        void read();
        double getEncL();
        double getEncR();
        void resetEncs();
        double getTargetL();
        void setBaseSpeed(double bs);
        double getBaseSpeed();
        void setLastRun(long int lr);
        long int getLastRun();
        bool isInMotion();
        bool isInTurn();
        void setInMotion(bool a);
        void setInTurn(bool a);
        void driveStraight(double dist, int bSpeed);
        double wheelPIDfeedback(double diff, double difft);
        double turnPIDfeedback(double diff, double difft);
        void turnToYaw(int yaw);
        float getTargetYaw();
        void setTargetYaw(float yawIn);
        double mdriveSpeed();
        double mturnSpeed();
        float getTurnTime();
};
#endif