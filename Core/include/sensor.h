#include "const.h"
#include "Arduino.h"
#include <vector>
#include <cmath>
#include <Arduino_LSM9DS1.h>
#ifndef SENSOR_H 
#define SENSOR_H

class SensorData {
    private:
        int **sensorHistory; // 2D array for sensor history
        int L; // Length of each list
        int currentIndex; // Index to keep track of the current position to insert new data
        float a1,a2,a3,b1,b2,b3;
    public:
        // Constructor
        SensorData(int L_val) {
            IMU.begin();
            L = L_val;
            sensorHistory = new int* [4];
            for (int i = 0; i < 4; ++i) {
                sensorHistory[i] = new int[L];
                for (int j = 0; j < L; ++j) {
                    sensorHistory[i][j] = 0; // Initialize all elements to 0
                }
            }
            currentIndex = 0; // Start from the first slot
        }

        // Destructor to deallocate dynamically allocated memory
        ~SensorData() {
            for (int i = 0; i < 4; ++i) {
                delete[] sensorHistory[i];
            }
            delete[] sensorHistory;
        }

        // Function to push new data into the sensor history
        void push(int* inIR) {
            for (int i = 0; i < 4; i++) {
                // Shift elements up by 1
                for (int j = L - 1; j > 0; j--) {
                    sensorHistory[i][j] = sensorHistory[i][j - 1];
                }
                // Insert new data into the first slot
                sensorHistory[i][0] = inIR[i];
            }
            currentIndex++;
        }

        // Function to calculate exponentially weighted average
        double* expSmooth(double lambda) {
            double *result = new double[4];
            int ci = min(L,currentIndex);
            for (int i = 0; i < 4; i++) {
                double weightedSum = 0.0;
                double denominator = 0.0;
                for (int j = 0; j < ci; j++) {
                    double weight = exp(-lambda*j);
                    weightedSum += weight * sensorHistory[i][j];
                    denominator += weight;
                }
                result[i] = weightedSum / denominator;
            }
            return result;
        }
        void allOn() {
            digitalWrite(RF_IRo,HIGH);
            digitalWrite(RR_IRo,HIGH);
            digitalWrite(LF_IRo,HIGH);
            digitalWrite(LL_IRo,HIGH);
        }
        void allOff() {
            digitalWrite(RF_IRo,LOW);
            digitalWrite(RR_IRo,LOW);
            digitalWrite(LF_IRo,LOW);
            digitalWrite(LL_IRo,LOW);
        }
        void frontOn() {
            digitalWrite(RF_IRo,HIGH);
            digitalWrite(LF_IRo,HIGH);
        }
        void frontOff() {
            digitalWrite(RF_IRo,LOW);
            digitalWrite(LF_IRo,LOW);
        }
        void sidesOn() {
            digitalWrite(RR_IRo,HIGH);
            digitalWrite(LL_IRo,HIGH);
        }
        void sidesOff() {
            digitalWrite(RR_IRo,LOW);
            digitalWrite(LL_IRo,LOW);
        }
        int* readAll() {
            int* arr = new int[4];
            arr[0] = analogRead(LF_IRi);
            arr[1] = analogRead(LL_IRi);
            arr[2] = analogRead(RR_IRi);
            arr[3] = analogRead(RF_IRi);
            return arr;
        }
        int* readFront() {
            int * arr = new int[2];
            arr[0] = analogRead(LF_IRi);
            arr[1] = analogRead(RF_IRi);
            return arr;
        }
        int * readSides() {
            int * arr = new int[2];
            arr[0] = analogRead(LL_IRi);
            arr[1] = analogRead(RR_IRi);
            return arr;
        }
        void readIMU() {
            IMU.readGyroscope(a1,a2,a3);
            IMU.readMagneticField(b1,b2,b3);
            Serial.print(a1);
            Serial.print(" ");
            Serial.print(a2);
            Serial.print(" ");
            Serial.print(a3);
            Serial.print(" ");
            Serial.print(b1);
            Serial.print(" ");
            Serial.print(b2);
            Serial.print(" ");
            Serial.println(b3);
        }


};

#endif