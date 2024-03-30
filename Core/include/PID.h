#include "const.h"
#include <cmath>
#ifndef PID_H 
#define PID_H
class PID {
    private:
        double kp, ki, kd, er_i, i;
    public:
        PID(double k1, double k2, double k3) {
            kp = k1;
            ki = k2;
            kd = k3;
            er_i = 0;
            i = 0;
        }
        ~PID() {
        }
        double feedback(double er, double dt) { // dt in millis
            i += er * dt;
            double d = (er - er_i) / dt;
            er_i = er;
            return (kp * er) + (ki * i) + (kd * d);
        }
        void resetI() {
            i = 0;
        }
};

#endif