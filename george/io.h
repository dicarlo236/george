#ifndef _io
#define _io
#include "mbed.h"
#include "foc.h"

void setup();
Serial *getSerial(int port);

bool getEnabled();
void setEnabled(bool _enabled);

bool getError();
void setError();
void set_inverter(Inverter* invt);
void ser_send(Serial *ser, const void* start, size_t size);
int ser_read(Serial *ser, void* start, size_t size);
#endif