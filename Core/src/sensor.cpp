#include <stdlib.h>
#include <vector>

#include "sensor.h"
#include "Arduino.h"
#include "MahonyAHRS.h"

void IR::print_test() {
    Serial.println("Everything's working!");
}

std::vector<double> IR::get_sensor_data() {
    std::vector<double> data = {1, 2, 3};
    /*
    code goes here
    */

    return data;
}

