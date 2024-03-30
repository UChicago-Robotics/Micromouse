#ifndef MOTOR_H
#define MOTOR_H

namespace motor {
    const double dist_between_treads = 9.25;
    const double wheel_diam = 3.2;
    const double wheel_circ = M_PI * 3.2;
    const int ticks_per_rev = 360;
    void turn(double theta);
    void setSpeed(double l, double r);
    void read();
    double get_LEnc();
    double get_REnc();
    void resetEncs();
};

#endif