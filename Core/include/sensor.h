#ifndef SENSOR_H 
#define SENSOR_H

#include <vector>

class IR {
    public:
        const int pulseIntervalMicros = 250000; // 4 times per second
        void print_test();
        std::vector<double> get_sensor_data();
};

class Gyro {

};

#endif