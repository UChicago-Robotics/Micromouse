#include "const.h"
#include <cmath>
#ifndef PID_H 
#define PID_H
class PIDController {
    private:
        double kp, ki, kd, er_i, i;
    public:
        PIDController(double k1, double k2, double k3);
        double feedback(double er, double dt);
        void resetI();
};
#endif