#ifndef _pwm_in
#define _pwm_in
#include "mbed.h"

class PWM_IN
{
    public:
    PWM_IN(PinName pin, int usec_min, int usec_max);
    bool get_enabled();
    float get_throttle();
    
    
    private:
    InterruptIn* int_in;
    DigitalIn*   dig_in;
    Timer timer;
    bool was_on;
    bool enabled;
    void handle_rise();
    void handle_fall();
    int usecs;
    int usec_min, usec_max;
    
};
#endif