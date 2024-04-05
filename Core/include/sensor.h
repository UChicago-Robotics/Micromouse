#include "Arduino.h"
#include "const.h"
#ifndef SENSOR_H
#define SENSOR_H

class SensorController {
   private:
    int** sensorHistory;  // 2D array for sensor history
    int L;                // Length of each list
    int currentIndex;     // Index to keep track of the current position to insert new data
    float gx, gy, gz;
    float ax, ay, az;

   public:
    SensorController(int L_val);
    void init();
    void allOn();
    void allOff();
    void frontOn();
    void frontOff();
    void sidesOn();
    void sidesOff();
    void push(int* inIR);
    double* expSmooth(double lambda);
    int* readAll();
    int* readFront();
    int* readSides();
    void readIMU();
};
#endif