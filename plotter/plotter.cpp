#include "mbed.h"
#include "plotter.h"

volatile float* data_1;
volatile float* data_2;
size_t buffer_size;
volatile bool send_now;
volatile float* plot_ptr;
float fminf(float a, float b)
{
    if(a < b) return a;
    return b;
}

float fmaxf(float a, float b)
{
    if(a > b) return a;
    return b;
}

Serial pc(USBTX, USBRX);
//
int samples_per_plot = -1;
int count = 0;
int plot_count = 0;
bool initialized = false;
void send_message()
{
    count++;
    //pc.printf("in send message...\n\r");
    if(!send_now) return;
    count = 0;
    char start = 236;
    //float data_6[] = {12.3456, 2.2, 33.4, 45.34, -52.34, 6, 734.234, 84.3, 99};
    ser_send(&pc, &start, 1);
    ser_send(&pc, &start, 1);
    ser_send(&pc, (void*)plot_ptr, buffer_size);
    memset((void*)plot_ptr, 0, buffer_size);
    send_now = false;
}
    
void init_plotter(int num_vars, int loop_freq, int plot_freq)
{
    pc.baud(115200);
    
    buffer_size = 3 * num_vars * sizeof(float);    
    send_now = false;
    data_1 = (volatile float*)malloc(buffer_size);
    data_2 = (volatile float*)malloc(buffer_size);
    memset((void*)data_1, 0, 3 * num_vars * sizeof(float));
    memset((void*)data_2, 0, 3 * num_vars * sizeof(float));
    samples_per_plot = loop_freq/plot_freq;
    buffer_size = 3 * num_vars * sizeof(float);
    if(samples_per_plot < 1)
        printf("Bad inputs for plooter. try again next time.\n\r");
    else
        initialized = true;
}

void plot(int index_v, float value)
{
    volatile float* data_ptr = (plot_count % 2) == 0 ? data_1 : data_2;
    if(!initialized)
    {
        //printf("plotter not initialized, not plotting!\n\r");
        return;
    }
    data_ptr[3*index_v] += value/(samples_per_plot + 1);
    data_ptr[3*index_v + 1] = 987;//fmaxf(data_ptr[3*index_v + 1], value);
    data_ptr[3*index_v + 2] = 236;//fminf(data_ptr[3*index_v + 2], value);
    //printf("count %d, samples per plot %d \n\r", count, samples_per_plot);
    
    if(count == samples_per_plot)
    {
        plot_ptr = data_ptr;
        send_now = true;
        plot_count++;
    }
        
}

void ser_send(Serial *ser, const void* start, size_t size)
{
    const char* data_ptr = (const char*)start; //ptr to beginning of buffer
    const char* end = data_ptr + size; //ptr to end of buffer
    while (data_ptr != end) //if we haven't reached the end
    {
        
        //if(ser->writable())
        //printf("put %d \n\r", *data_ptr);
        if(true)
            ser->putc(*data_ptr++);//go write to the serial port and move the buffer ptr forward
        else
            return; //need to go implement return codes.
    }
}