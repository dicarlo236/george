#ifndef _foc
#define _foc
#include "config.h"

struct sensorData{
    float v_bus, i_a, i_b;
};


struct Inverter{
    float v_a, v_b, v_c;
};

extern Inverter *inverter;

extern sensorData *sensors;
float fmaxf(float a, float b);
float fminf(float a, float b);
void initControl();
void sampleCurrent();
void doFOC(float current_time);
#endif