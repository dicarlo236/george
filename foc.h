#ifndef _foc
#define _foc
#include "config.h"
#include "pwm_in.h"

float fmaxf(float a, float b);
float fminf(float a, float b);
void init_control();
void motor_control(PWM_IN* p_in);
void foc(float ia, float ib, PWM_IN* p_in);
void zero_current();
float get_debug(int num);

#endif