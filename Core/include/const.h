#include "Arduino.h"
#ifndef CONST_H
#define CONST_H

const int IR_INPUT_PINS[4] = {A0, A1, A2, A3};
const int IR_OUTPUT_PINS[4] = {D9, D10, D11, D12};
const int MOTOR_PINS[4] = {D2, D3, D4, D5};

#define HIGH 1
#define LOW 0

// motor pins <POS><DIR>_MOTOR
#define RB_MOTOR D5
#define RF_MOTOR D4
#define LF_MOTOR D3
#define LB_MOTOR D2

// encoder pins <POS><DIR>_ENC
#define LR_ENC A7
#define LL_ENC A6
#define RR_ENC A5
#define RL_ENC A4

// IR pins <POS><ORIENTATION>_IRi
#define RF_IRi A1
#define RR_IRi A0
#define LL_IRi A3
#define LF_IRi A2
#define RF_IRo D11
#define RR_IRo D12
#define LL_IRo D9
#define LF_IRo D10

// button input
#define ONOFF D6
#define BUTTON_1 D7
#define BUTTON_2 D8

#endif