#include "mbed.h"
#include "plotter.h"

volatile float* data_1;
volatile float* data_2;
size_t buffer_size;
volatile bool send_now;
volatile float* plot_ptr;

Serial pc(USBTX, USBRX);
//
int samples_per_plot = -1;
int count = 0;
int plot_count = 0;
bool initialized = false;
void send_message()
{
    //printf("in serial send\n\r");
    char start = 236;
    ser_send(&pc, &start, 1);
    ser_send(&pc, &start, 1);
    ser_send(&pc, (void*)data_1, buffer_size);
}
    
void init_plotter(int num_vars)
{
    pc.baud(115200);
    buffer_size = num_vars * sizeof(float);    
    data_1 = (volatile float*)malloc(buffer_size);
    memset((void*)data_1, 0, buffer_size);
}

void plot(int index_v, float value)
{
    volatile float* data_ptr = data_1;
    data_ptr[index_v] = value;  
}
