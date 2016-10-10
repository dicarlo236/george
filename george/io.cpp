#include "mbed.h"
#include "config.h"
#include "io.h"
#include "foc.h"

Serial pc_ser(USBTX, USBRX);
//Serial debug_ser(PC_10, PC_11);
//Serial panel_ser(PC_6, PC_7);
DigitalOut dout(PC_9);
PwmOut a(PA_8);
PwmOut b(PA_9);
PwmOut c(PA_10);

float tick_const = SystemCoreClock/1000000.0f;

volatile bool enabled = false;
volatile bool error_ = false;
bool arduino_connected = false;

bool getError()
{
    return error_;
}

void setError()
{
    error_ = true;
}

bool getEnabled()
{
    return enabled;
}

void setEnabled(bool _enabled)
{
    enabled = _enabled;
}
void setup()
{
    getSerial(0)->baud(SERIAL_BAUD); //set serial baud
 //   getSerial(1)->baud(SERIAL_BAUD); //set serial baud
//    getSerial(2)->baud(SERIAL_BAUD); //set serial baud
    
    getSerial(0)->printf("GEORGE\n");
    getSerial(0)->printf("*******INITIALIZING**********\n");
    
    getSerial(0)->printf("Setting up clocks...\n");
    /*************************************
                   CLOCKS 
    **************************************/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //gpio a
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //gpio b
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //gpio c
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  //tim1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  //adc1
//    RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;  //adc2
//    RCC->APB1ENR |= RCC_APB1ENR_DACEN;   //dac
    
    getSerial(0)->printf("Setting up interrupts...\n");
    /*************************************
                   INTERRUPTS 
    **************************************/
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); //setup interrupt controller
    TIM1->DIER |= TIM_DIER_UIE;         //enable timer interrupts

    getSerial(0)->printf("Setting up PWM...\n");
    /************************************
               PWM CONFIGURATION
    *************************************/
    //set center aligned mode bits to 10 (might be 01 though).
    //TIM1->CR1  |= TIM_CR1_CMS_1; //counts up and down, but only interrupts when counting up.
    TIM1->CR1 = 0x40;
    TIM1->CR1 |= TIM_CR1_ARPE;
    TIM1->RCR |= 0x01; //repetition counter set for 2 repetitions
    TIM1->EGR  |= TIM_EGR_UG; //reference manual says I need to go this.
    //enable and set polarity of output compares (using 1, 2, 3)
    TIM1->CCER |= TIM_CCER_CC3E;
    TIM1->CCER |= TIM_CCER_CC3P;
    TIM1->CCER |= TIM_CCER_CC2E;
    TIM1->CCER |= TIM_CCER_CC2P;
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->CCER |= TIM_CCER_CC1P;
    //TIM1->CCER = 0x1111;
    TIM1->CCER |= ~(TIM_CCER_CC1NP);
    
    getSerial(0)->printf("Setting up Timers...\n\r");
    /************************************
              TIMER CONFIGURATION
    *************************************/
    //buffer the auto reload register (ARR) to update only on update event

    TIM1->PSC = 0x0; //no prescaler.
    //set counter maximum to value for 10 kHz PWM
    TIM1->ARR = 0x2328; //this is for 20 kHz, which is think is a needed correction.
    //counter needs to do two cycles for 1 full pwm cycle I think.
    //this is due to the center aligned mode.

    getSerial(0)->printf("Setting up ADC...\n");
    /************************************
                  ADC CONFIG
    *************************************/
    ADC->CCR |= 0b110; //set adc 1, 2 to sample together.
    ADC1->SQR3 |= 0x4; //on ADC1, sample ADC12_IN channel 4 (PA4)
//    ADC2->SQR3 |= 0x8; //on ADC2, sample ADC12_IN channel 8 (PB0)
    
    getSerial(0)->printf("Setting up GPIO...\n");
    getSerial(0)->printf("PWM pins are set up as 'alternate function'...\n");
    getSerial(0)->printf("May eventually need to set speed, pullup...\n");
    /************************************
                GPIO CONFIG
    *************************************/
    //pwm outs
    //0x2A0000 = 0b 10 10 10 00 00 00 00 00 00 00 00
    GPIOA->MODER |= 0x2AA000;
    //                109876543210
    GPIOA->MODER |= 0b001100000000; //PA4 as analog.
    GPIOB->MODER |= 0b000000000011; //PB0 as analog.
    GPIOA->MODER |= 0b110000000000; //PA5 as analog.
    
    getSerial(0)->printf("Turning on ADC/DAC...\n");
    /************************************
                ADC/DAC START
    *************************************/
    ADC1->CR2 |= ADC_CR2_ADON; //adc 1 on
//    ADC2->CR2 |= ADC_CR2_ADON; //adc 2 on
//    DAC->CR   |= DAC_CR_EN2;   //dac 2 (PA5) on
    
    getSerial(0)->printf("Turning on TIMER\n");
    /************************************
                 TIMER START
    *************************************/
    TIM1->CR1 |= TIM_CR1_CEN; //enable!
    wait_us(100000);
    
    /************************************
                 ZERO CURRENT
    *************************************/
    
    /************************************
                   ENCODER
    *************************************/    
    
    /************************************
               FRONT PANEL BOARD
    *************************************/
    char front_panel_init_value = 236;
    getSerial(0)->printf("Sending init command to arduino..\n");
 //   ser_send(getSerial(2), &front_panel_init_value, 1);
    wait_us(100000);
    getSerial(0)->printf("Waiting for response...\n");
    char response = 0;
 //   if(ser_read(getSerial(2), &response, 1) < 1) getSerial(0)->printf("Got nothing from arduino. Is it on?\n");
    if(response == 118)
    {
         getSerial(0)->printf("Got a good response from arduino.\n");
         arduino_connected = true;
    }     
    else
    {
         getSerial(0)->printf("Got a bad response from arduino.\n");
    }
    initControl();
    dout = 0;
}

Serial *getSerial(int port)
{
    switch(port)
    {
        case 0:
            return &pc_ser;
        case 1:
            //return &debug_ser;
        case 2:
           // return &panel_ser;
           
           
    }
    return &pc_ser;
}


extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
    if(TIM1->SR & TIM_SR_UIF) {
        dout = !dout;
        //if(getEnabled() && !getError()) sampleCurrent();
        //set_inverter(NULL);
        sampleCurrent();
        //else go turn off enabled pin.
       // getSerial(0)->printf("run\n");
       
    }
    TIM1->SR = 0;
}


uint32_t volts_to_ticks(float volts)
{
    float duty_cycle = volts/K_V_BUS + .5f;
    duty_cycle = fmaxf(.04f, duty_cycle);
    duty_cycle = fminf(.96f, duty_cycle);
    //getSerial(0)->printf("duty cycle %d\n\r", (uint32_t)(duty_cycle * 0x2328));
    return (uint32_t)(duty_cycle * 0x2328);
    float width_us = 50.0f*duty_cycle; //this needs to change if I slow down from 20 kHz!
    return (width_us * tick_const + 0.5f); //add .5 to avoid ever setting zero.
}

void set_inverter(Inverter* invt)
{
    TIM1->CCR1 = volts_to_ticks(invt->v_a);
    TIM1->CCR2 = volts_to_ticks(invt->v_b);
    TIM1->CCR3 = volts_to_ticks(invt->v_c);
    //getSerial(0)->printf("set_invt %d\n\r", volts_to_ticks(invt->v_c));
    
}

//why doesn't mbed have something like this....

void ser_send(Serial *ser, const void* start, size_t size)
{
    const char* data_ptr = (const char*)start; //ptr to beginning of buffer
    const char* end = data_ptr + size; //ptr to end of buffer
    while (data_ptr != end) //if we haven't reached the end
    {
        //if(ser->writable())
        if(true)
            ser->putc(*data_ptr++);//go write to the serial port and move the buffer ptr forward
        else
            return; //need to go implement return codes.
    }
}

int ser_read(Serial *ser, void* start, size_t size)
{
    unsigned char* b_ptr = (unsigned char*)start; //start of buffer ptr
    for(int i = 0; i < size; i++) //loop through the buffer
    {
        if(ser->readable()) (*b_ptr++) = ser->getc(); //if there's something to read, read and advance buffer ptr
        else return i; //otherwise, break from loop and return bytes read.
    }
    return size; //we finished the loop so buffer is full, all bytes read.
}