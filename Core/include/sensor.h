#include "Arduino.h"
#include "const.h"
// #include "madgwickFilter.h"
#include "MahonyAHRS.h"
#ifndef SENSOR_H
#define SENSOR_H

class SensorController {
    public:
        Mahony mahony;
        // quaternion q_est;
        int** sensorHistory;  // 2D array for sensor history
        int L;                // Length of each list in sensor history
        float lambda;         // discount rate of exponential smoothing
        int RR, RF, LF, LL;   // most recent raw sensor values
        float RRs, RFs, LFs, LLs;
        int currentIndex;     // Index to keep track of the current position to insert new data
        float gx, gy, gz, ax, ay, az, roll, pitch, yaw;
        float LL_base, RR_base;
        float LL_cutoff, RR_cutoff, LF_cutoff, RF_cutoff; // cutoff for missing wall ("__ Wall absent")
        float LL_coeff, RR_coeff, LF_coeff, RF_coeff;
        float mx, my, mz;
        float gx0, gy0, gz0;
        float ax0, ay0, az0;
        float mx0, my0, mz0;
        unsigned long lastReadingTime;
        float dt;
        long runTime;
        bool calibrated = false;
        SensorController(int L_val, double lambda_val);
        void init();
        void allOn();
        void allOff();
        void frontOn();
        void frontOff();
        void sidesOn();
        void sidesOff();
        void push(); 
        void readAll();
        void readFront();
        void readSides();
        void read();
        int getRR();
        int getRF();
        int getLF();
        int getLL();
        float getRRs();
        float getRFs();
        float getLFs();
        float getLLs();
        void readIMU();
        void readFilterIMU();
        float getYawDeg();
        String dumpIMUString();
        String dumpIRString();
        void resetWallBase();
        void setBaseL(float l);
        void setBaseR(float r);
        float getBaseL();
        float getBaseR();
        float getLLCut();
        float getRRCut();
        float getLFCut();
        float getRFCut();
        float getLLCoeff();
        float getRRCoeff();
        float getLFCoeff();
        float getRFCoeff();
        float getAx();
        float getAy();
        float getAz();
        float getGz();
        void calibrate();
};
#endif