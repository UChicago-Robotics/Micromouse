#include <stdlib.h>
#include <vector>

#include "sensor.h"
#include "Arduino.h"
#include "MahonyAHRS.h"
#include "const.h"

void IR::print_test() {
    Serial.println("Everything's working!");
}

std::vector<double> IR::get_sensor_data() {
    return {1, 2, 3};
}

