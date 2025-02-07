#include "Arduino.h"
#ifndef CONST_H
#define CONST_H

const int IR_INPUT_PINS[4] = {A0, A1, A2, A3};
const int IR_OUTPUT_PINS[4] = {D9, D1, D12, D13};
const int MOTOR_PINS[4] = {D2, D3, D4, D5};

#define HIGH 1
#define LOW 0

// motor pins <POS><DIR>_MOTOR
#define RB_MOTOR D4
#define RF_MOTOR D5
#define LF_MOTOR D3
#define LB_MOTOR D2

// encoder pins <POS><DIR>_ENC
#define LR_ENC A7
#define LL_ENC A6
#define RR_ENC A5
#define RL_ENC A4

// IR pins <POS><ORIENTATION>_IRi
#define SR_IRi A0
#define FR_IRi A1
#define FL_IRi A2
#define SL_IRi A3
#define FR_IRo D1
#define SR_IRo D9
#define FL_IRo D12
#define SL_IRo D13

// button input
#define BUTTON D8

#endif
