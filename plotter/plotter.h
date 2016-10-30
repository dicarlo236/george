#ifndef _plotter
#define _plotter

void init_plotter(int num_vars);
void plot(int index, float value);

void send_message();
void ser_send(Serial *ser, const void* start, size_t size);
#endif