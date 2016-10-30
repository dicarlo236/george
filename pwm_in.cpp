#include "pwm_in.h"
#include "mbed.h"

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrain(float in, float min, float max)
{
    if(in > max) return max;
    if(in < min) return min;
    return in;
}

PWM_IN::PWM_IN(PinName pin, int usec_min, int usec_max)
{
    int_in = new InterruptIn(pin);
    dig_in = new DigitalIn(pin);
    int_in->rise(this, &PWM_IN::handle_rise);
    int_in->fall(this, &PWM_IN::handle_fall);
    this->usec_min = usec_min;
    this->usec_max = usec_max;
}


bool PWM_IN::get_enabled()
{
    return enabled;
}

void PWM_IN::handle_rise()
{
    enabled = true;
    timer.stop();
    timer.reset();
    timer.start();
    was_on = true;
}

void PWM_IN::handle_fall()
{
    was_on = false;
    usecs = timer.read_us();
    timer.stop();
    timer.reset();
    timer.start();
}

float PWM_IN::get_throttle()
{
    if(timer.read_us() > 40000) enabled = false;
    if(!enabled) return -1;
    return constrain(map((float)usecs, usec_min, usec_max, 0, 1), 0, 1);
}

