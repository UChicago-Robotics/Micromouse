#ifndef MOTOR_H
#define MOTOR_H

namespace motor {
    void turn(double theta);
    void setSpeed(double l, double r);
    void read();
    int get_LEnc();
    int get_REnc();
    void resetEncs();
};

#endif