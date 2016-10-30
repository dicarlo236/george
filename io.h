#ifndef _io
#define _io
#include "mbed.h"
#include "foc.h"

void setup();
Serial *get_serial(int port);


void set_inverter(float v_a, float v_b, float v_c);
void ser_send(Serial *ser, const void* start, size_t size);
int ser_read(Serial *ser, void* start, size_t size);
float get_position();
float get_throttle();
float get_velocity();
#endif