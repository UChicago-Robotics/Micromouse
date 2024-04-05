#include "const.h"
#include <cmath>
#ifndef PID_H 
#define PID_H
class PIDController {
    private:
        double kp, ki, kd, er_i, i;
    public:
        PIDController(double kp, double ki, double kd);
        double feedback(double er, double dt);
        void resetI();
};
#endif