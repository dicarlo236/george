#include "vehicle_controller.h"
#include "config.h"
#include "io.h"
#include "plotter.h"
#include <math.h>

float filtered_velocity = 0;
float alpha = .3;
float vel_int = 0;

//simple current controller
float current_control(float throttle)
{
    return throttle * K_Q_CURRENT_MAX;
}

//servo controller
float servo_control(float throttle)
{
    float velocity = get_velocity();
    velocity = isnan(velocity) ? 0 : velocity;
    filtered_velocity = alpha * velocity + (1 - alpha) * filtered_velocity;
    float v_err = -filtered_velocity + throttle * 500;
    vel_int += v_err;
    return v_err * K_SERVO_KP + vel_int * K_SERVO_KI;
}


float voltage_control(float throttle)
{
    float velocity = get_velocity();
    velocity = isnan(velocity) ? 0 : velocity;
    //plot(0, velocity);
    filtered_velocity = alpha * velocity + (1 - alpha) * filtered_velocity;
    float v_err = -filtered_velocity + throttle * 500;
    vel_int += v_err;
    if(v_err < 0) v_err = 0;
    //plot(1, v_err);
    return v_err * K_VOLTAGE_KP;
}