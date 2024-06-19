#include "Arduino.h"
#include "ArduinoBLE.h"
#ifndef COMM_H
#define COMM_H
void bt_setup();
void printstr(String data);
void setVxy(byte *x, byte *y);
#endif