#include "Arduino.h"
#include "const.h"
// #include "madgwickFilter.h"
#ifndef SENSOR_H
#define SENSOR_H

class SensorController {
    private:
        // quaternion q_est;
        int** sensorHistory;  // 2D array for sensor history
        int L;                // Length of each list in sensor history
        float lambda;         // discount rate of exponential smoothing
        int RR, RF, LF, LL;   // most recent raw sensor values
        float RRs, RFs, LFs, LLs;
        int currentIndex;     // Index to keep track of the current position to insert new data
        float gx, gy, gz, ax, ay, az, roll, pitch, yaw;

    public:
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
        float getRoll();
        float getPitch();
        float getYaw();
        String dumpString();
};
#endif