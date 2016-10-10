#include "foc.h"
#include "io.h"
#include "config.h"
#include "mbed.h"

Inverter *inverter;
sensorData *sensors;

bool drive_enabled = false;

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

void initControl()
{
    inverter = new Inverter();
    sensors = new sensorData();
    inverter->v_a = 0.0f;
    inverter->v_b = 0.0f;
    inverter->v_c = 0.0f;
    sensors->i_a = 0.0f;
    sensors->i_b = 0.0f;
    sensors->v_bus = 0.0f;
    
}

void sampleCurrent()
{
    //adc stuff here
    doFOC(0.0f);
}

//*************FOC VARIABLES***************//
float q_command = 20.0f;//q current setpoint, amps
float d_command = 0.0f;//d current setpoint, amps
float K_P = 9.0f;//proportional gain, q and d axis current PI controller
float K_I = 0.0f;//integral gain, q and d axis current PI controller
float integral_max = 200.0f;//maximum integral.
float axis_v_max = 20.0f;

//***********MORE FOC VARIABLES***********//
float d_int = 0.0f;
float q_int = 0.0f;
float SQRT_3 = sqrtf(3.0f);

//***************HALL VARIABLES*********//
double theta_elec = 0;
int rotations = 0;
//motor mechanical angle, not wrapped.
double theta_continuous = 0;
bool direction = true;
int hall_state = 0;
int POLE_PAIRS = 10;
double last_theta = 0;
double last_last_theta = 0;
double last_hall_time = 0;
double omega_est = 0;

int next_hall[] = {-1, 5, 3, 1, 6, 4, 2, -1};
int hall_pos[] =  {-1, 5, 3, 4, 1, 0, 2, -1};
float theta_e;
void doFOC(float current_time)
{
    float i_a =     (float)sensors->i_a;
    float i_b =     (float)sensors->i_b;

    theta_e += .0005; //add encoder read here.

    float sin_theta_e = sinf(theta_e);
    float cos_theta_e = cosf(theta_e);

    //calculate third phase current
    float i_c = -i_a - i_b;

    //convert from a, b, c to alpha, beta
    float alpha_i = i_a;
    float beta_i = (i_c - i_b)/SQRT_3;

    //convert from alpha, beta to q,d
    float d_i =  alpha_i * cos_theta_e - beta_i * sin_theta_e;
    float q_i =  alpha_i * sin_theta_e + beta_i * cos_theta_e;

    //PI controller for q,d
    q_int += (q_command - q_i) * K_I;
    d_int += (d_command - d_i) * K_I;

    //prevent integral windup
    q_int = fabs(q_int) > integral_max ? integral_max * (q_int > 0 ? 1 : -1) : q_int;
    d_int = fabs(d_int) > integral_max ? integral_max * (d_int > 0 ? 1 : -1) : d_int;

    //calculate q, d axis voltages
    float q_v = 10;//K_P * (q_command - q_i);// + q_int;
    float d_v = 0; //K_P * (d_command - d_i);// + d_int;

    //limit axis voltages
    q_v = fabs(q_v) > axis_v_max ? axis_v_max * (q_v > 0 ? 1 : -1) : q_v;
    d_v = fabs(d_v) > axis_v_max ? axis_v_max * (d_v > 0 ? 1 : -1) : d_v;

    //convert back to alpha, beta
    float alpha_v = d_v * cos_theta_e + q_v * sin_theta_e;//if theta = 0, alpha_v = 0 - 0 = 0;
    float beta_v =  -d_v * sin_theta_e + q_v * cos_theta_e;
    //getSerial(0)->printf("alpha: %f beta: %f\n\r", alpha_v, beta_v);
    //convert to a, b, c voltages
    float a_v =  alpha_v;
    float b_v = -alpha_v/2.0f - SQRT_3 * beta_v / 2.0f;
    float c_v = -alpha_v/2.0f + SQRT_3 * beta_v / 2.0f;

    float Voff = (fminf(a_v, fminf(b_v, c_v)) + fmaxf(a_v, fmaxf(b_v, c_v)))/2.0f;
    a_v = a_v - Voff;
    b_v = b_v - Voff;
    c_v = c_v - Voff;

    inverter->v_a = a_v;
    inverter->v_b = b_v;
    inverter->v_c = c_v;
    
    set_inverter(inverter);
}


    
