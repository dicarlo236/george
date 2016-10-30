//Configuration constants
#ifndef _config
#define _config

#define M_PI 3.1415926535897932384f
#define nullptr 0 

#define K_SERIAL_BAUD 115200 //serial over USB baud rate

#define K_V_BUS             40.8f   //HV battery voltage
#define K_Q_CURRENT_MAX     20.0f   //max torque 
#define K_Q_CURRENT_MIN     -1.0f   //max regen torque
#define K_P_CURRENT         1.0f    //current loop kp
#define K_I_CURRENT         0.1f    //current loop ki
#define K_CURRENT_INT_MAX  20.0f    //current loop integral max
#define K_AXIS_V_MAX       11000.0f //current loop axis voltage max


#define K_POS_OFFSET       (- 5.78f + M_PI + 0.03f) //electrical position offset

#define K_VOLTAGE_KP  0.1f
#define K_SERVO_KP    0.1f
#define K_SERVO_KI    0.001f


//from bayley
#define POLE_PAIRS 3.0f
#define RESOLVER_LOBES 3.0f
#define CPR 4096
#define I_SCALE_RAW 25.0f //mv/A
#define R_UP 12000.0f //ohms
#define R_DOWN 3600.0f //ohms
#define R_BIAS 3600.0f //ohms
#define AVDD 3300.0f //mV
#define I_OFFSET (AVDD * R_DOWN * R_UP / (R_DOWN * R_UP + R_BIAS * (R_DOWN + R_UP)))
#define I_SCALE (R_BIAS * R_DOWN * I_SCALE_RAW / (R_DOWN * R_UP + R_BIAS * (R_DOWN + R_UP)))

#endif