#include "foc.h"
#include "io.h"
#include "config.h"
#include "mbed.h"
#include "plotter.h"
#include "vehicle_controller.h"

//*************FOC VARIABLES***************//
float q_command =      0.0f;                //q current setpoint, amps
float d_command =      0.0f;                //d current setpoint, amps
float K_P =            K_P_CURRENT;         //proportional gain, q and d axis current PI controller
float K_I =            K_I_CURRENT;         //integral gain, q and d axis current PI controller
float integral_max =   K_CURRENT_INT_MAX;   //maximum integral.
float axis_v_max =     K_AXIS_V_MAX;        //maximum alpha/beta axis voltage
float d_int =          0.0f;                //integral of d axis current error
float q_int =          0.0f;                //integral of q axis current error
float SQRT_3 =         sqrtf(3.0f);         //math constant
float theta_e =        0.0f;                //electrical motor angle
float ia_supp_offset = 0.0f;                //calculated current sensor offset         
float ib_supp_offset = 0.0f;                //calculated current sensor offset
bool control_enabled = false;               //gate drive on
DigitalOut             en(PB_15);           //gate drive enable output


//********MATH FUNCTIONS********/
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

//******PUBLIC FUNCTIONS********/

//initialize motor controller
void init_control()
{
    zero_current();
}

//run motor control loop
void motor_control(PWM_IN* p_in)
{
    ADC1->CR2  |= 0x40000000; 
    volatile int delay;
    for (delay = 0; delay < 35; delay++);
    foc(ADC1->DR, ADC2->DR, p_in);
}

//calculate zero current offset
void zero_current(){
    for (int i = 0; i < 1000; i++){
        ia_supp_offset += (float) (ADC1->DR);
        ib_supp_offset += (float) (ADC2->DR);
        ADC1->CR2  |= 0x40000000;
        wait_us(100); 
    }
    ia_supp_offset /= 1000.0f;
    ib_supp_offset /= 1000.0f;
    ia_supp_offset = ia_supp_offset / 4096.0f * AVDD - I_OFFSET;
    ib_supp_offset = ib_supp_offset / 4096.0f * AVDD - I_OFFSET;
}


//*****************PRIVATE*****************//

void go_enabled()
{
    d_int = 0.0f;
    q_int = 0.0f;
    control_enabled = true;
    en = 1;
}

void go_disabled()
{
    control_enabled = false;
    en = 0;
}

void foc(float ia, float ib, PWM_IN* p_in)
{
    if(control_enabled && !p_in->get_enabled()) go_disabled();
    if(!control_enabled && p_in->get_enabled()) go_enabled();
    //get q current command from vehicle_controller and throttle value
    q_command = current_control(get_throttle());
    
    //check q current min/max
    if(q_command > K_Q_CURRENT_MAX) q_command = K_Q_CURRENT_MAX;
    if(q_command < K_Q_CURRENT_MIN) q_command = K_Q_CURRENT_MIN;
    
    //scale and offset a and b current values
    float i_a = ((float) ia / 4096.0f * AVDD - I_OFFSET - ia_supp_offset) / I_SCALE;
    float i_b = ((float) ib / 4096.0f * AVDD - I_OFFSET - ib_supp_offset) / I_SCALE;
    
    //scale and offset electrical position
    theta_e = get_position() + K_POS_OFFSET;
    
    //set theta to a positive angle
    if(theta_e < 0) theta_e += 2 * M_PI;

    //compute sin/cos ahead of time
    float sin_theta_e = sinf(theta_e);
    float cos_theta_e = cosf(theta_e);

    //calculate third phase current
    float i_c = -i_a - i_b;

    //convert from a, b, c to alpha, beta
    float alpha_i = i_a;
    float beta_i = (i_c - i_b)/SQRT_3;

    //convert from alpha, beta to q,d
    float alpha = 0.1;
    float d_i =  alpha * (alpha_i * cos_theta_e - beta_i * sin_theta_e) + (1 - alpha) * d_i;
    float q_i =  alpha * (alpha_i * sin_theta_e + beta_i * cos_theta_e) + (1 - alpha) * q_i;
    
    //calculate error integral
    q_int += (q_command - q_i) * K_I;
    d_int += (d_command - d_i) * K_I;

    //prevent integral windup
    q_int = fabs(q_int) > integral_max ? integral_max * (q_int > 0 ? 1 : -1) : q_int;
    d_int = fabs(d_int) > integral_max ? integral_max * (d_int > 0 ? 1 : -1) : d_int;

    //calculate q, d axis voltages
    float q_v = K_P * (q_command - q_i) + q_int;
    float d_v = K_P * (d_command - d_i) + d_int;
plot(0, q_v); plot(1, d_v);
    //limit axis voltages
    q_v = fabs(q_v) > axis_v_max ? axis_v_max * (q_v > 0 ? 1 : -1) : q_v;
    d_v = fabs(d_v) > axis_v_max ? axis_v_max * (d_v > 0 ? 1 : -1) : d_v;

    //convert back to alpha, beta
    float alpha_v = d_v * cos_theta_e + q_v * sin_theta_e;
    float beta_v =  -d_v * sin_theta_e + q_v * cos_theta_e;

    //convert to a, b, c voltages
    float a_v =  alpha_v;
    float b_v = -alpha_v/2.0f - SQRT_3 * beta_v / 2.0f;
    float c_v = -alpha_v/2.0f + SQRT_3 * beta_v / 2.0f;

    //SVPWM
    float offset_voltage = (fminf(a_v, fminf(b_v, c_v)) + fmaxf(a_v, fmaxf(b_v, c_v)))/2.0f;
    a_v = a_v - offset_voltage;
    b_v = b_v - offset_voltage;
    c_v = c_v - offset_voltage;
    
    set_inverter(a_v, b_v, c_v);
}


    
