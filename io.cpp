#include "mbed.h"
#include "config.h"
#include "io.h"
#include "foc.h"
#include "pwm_in.h"

Serial      pc_ser(USBTX, USBRX);
DigitalOut  dout(PC_9);
PwmOut      a(PA_9);
PwmOut      b(PA_8);
PwmOut      c(PA_10);
PWM_IN      p_in(PB_8, 1100, 1900);
InterruptIn *index_interrupt;
DigitalIn   *index_in;

void handle_index();

float get_throttle()
{
    return p_in.get_throttle();
}

void setup()
{
    //set serial baud
    get_serial(0)->baud(K_SERIAL_BAUD);
    
    //get_serial(0)->printf("GEORGE\n\r");
    //get_serial(0)->printf("*******INITIALIZING**********\n\r");
    
    //get_serial(0)->printf("Setting up clocks...\n\r");
    /*************************************
                   CLOCKS 
    **************************************/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //gpio a
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //gpio b
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //gpio c
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  //tim1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  //adc1
    RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;  //adc2
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;   //dac
    
    //get_serial(0)->printf("Setting up interrupts...\n\r");
    /*************************************
                   INTERRUPTS 
    **************************************/
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); //setup interrupt controller
    TIM1->DIER |= TIM_DIER_UIE;         //enable timer interrupts

    //
    
    
    //get_serial(0)->printf("Setting up PWM...\n\r");
    /************************************
               PWM CONFIGURATION
    *************************************/
    //set center aligned mode bits to 10 (might be 01 though).
    TIM1->CR1  |= TIM_CR1_CMS_1;   //counts up and down, but only interrupts when counting up.
    TIM1->CR1   = 0x40;
    TIM1->CR1  |= TIM_CR1_ARPE;
    TIM1->RCR  |= 0x01;            //repetition counter set for 2 repetitions
    TIM1->EGR  |= TIM_EGR_UG;      //reference manual says I need to go this.
    //enable and set polarity of output compares (using 1, 2, 3)
    TIM1->CCER |= TIM_CCER_CC3E;
    TIM1->CCER |= TIM_CCER_CC3P;
    TIM1->CCER |= TIM_CCER_CC2E;
    TIM1->CCER |= TIM_CCER_CC2P;
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->CCER |= TIM_CCER_CC1P;
    
    //get_serial(0)->printf("Setting up Timers...\n\r");
    /************************************
              TIMER CONFIGURATION
    *************************************/
    //buffer the auto reload register (ARR) to update only on update event

    TIM1->PSC = 0x0; //no prescaler.
    //set counter maximum to value for 5 kHz PWM
    TIM1->ARR = 0x4650; //5khz for prius inverter
    //counter needs to do two cycles for 1 full pwm cycle I think.
    //this is due to the center aligned mode.

    //get_serial(0)->printf("Setting up ADC...\n");
    /************************************
                  ADC CONFIG
    *************************************/
    ADC->CCR |= 0b110; //set adc 1, 2 to sample together.
    ADC1->SQR3 |= 0x4; //on ADC1, sample ADC12_IN channel 4 (PA4)
    ADC2->SQR3 |= 0x8; //on ADC2, sample ADC12_IN channel 8 (PB0)
    
    //get_serial(0)->printf("Setting up GPIO...\n");
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
    
    //get_serial(0)->printf("Turning on ADC/DAC...\n");
    /************************************
                ADC/DAC START
    *************************************/
    ADC1->CR2 |= ADC_CR2_ADON; //adc 1 on
    ADC2->CR2 |= ADC_CR2_ADON; //adc 2 on
    DAC->CR   |= DAC_CR_EN2;   //dac 2 (PA5) on
    
    //get_serial(0)->printf("Turning on TIMER\n");
    /************************************
                 TIMER START
    *************************************/
    TIM1->CR1 |= TIM_CR1_CEN; //enable!
    wait_us(100000);
    
    /************************************
                 INIT CONTROL
    *************************************/
    init_control();
    /************************************
                   ENCODER
    *************************************/    
    //from ben's encoder code
    GPIOB->MODER   |= GPIO_MODER_MODER3_1;           
    GPIOB->OTYPER  |= GPIO_OTYPER_OT_3 | GPIO_OTYPER_OT_10 ;                 
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR10 ;     
    GPIOB->AFR[0]  |= 0x00001000 ;                                          
    
    GPIOA->MODER   |= GPIO_MODER_MODER15_1;           
    GPIOA->OTYPER  |= GPIO_OTYPER_OT_15;                 
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15;     
    GPIOA->AFR[1]  |= 0x10000000 ;
   
    __TIM2_CLK_ENABLE();
 
    TIM2->CR1   = 0x0001;
    TIM2->SMCR  = TIM_ENCODERMODE_TI12;
    TIM2->CCMR1 = 0xf1f1;
    TIM2->CCMR2 = 0x0000;
    TIM2->CCER  = 0x0011;
    TIM2->PSC   = 0x0000;
    TIM2->ARR   = 0xffffffff;
  
    TIM2->CNT = 0;
    index_interrupt =   new InterruptIn(PB_12);
    index_in =          new DigitalIn(PB_12);
    index_interrupt->   enable_irq();
    index_interrupt->   rise(&handle_index);
    index_interrupt->   mode(PullDown);

}

Serial *get_serial(int port)
{
    switch(port)
    {
        case 0:
            return &pc_ser;
        case 1:
            //return &debug_ser;
        case 2:
           // return &panel_ser;
           break;          
    }
    return &pc_ser;
}

//timer interrupt handler
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
    //check to see if timer1 caused the interrupt
    if(TIM1->SR & TIM_SR_UIF) {
        //togle debugging pin
        dout = !dout;
        //run control loop
        motor_control(&p_in);
    }
    //reset timer 1 interrupt flag
    TIM1->SR = 0;
}

//convert phase voltage to timer ticks
uint32_t volts_to_ticks(float volts)
{
    float duty_cycle = volts/K_V_BUS + .5f;
    //for prius brick, we don't need to limit duty cycle!
    //duty_cycle = fmaxf(.04f, duty_cycle);
    //duty_cycle = fminf(.96f, duty_cycle);
    return (uint32_t)(duty_cycle * 0x4650);
}

void set_inverter(float v_a, float v_b, float v_c)
{
    TIM1->CCR1 = volts_to_ticks(v_a);
    TIM1->CCR2 = volts_to_ticks(v_b);
    TIM1->CCR3 = volts_to_ticks(v_c);  
}

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

//interrupt handler for encoder index
void handle_index()
{
    if (index_in->read())
    {
        if(index_in->read())
            TIM2->CNT=0;
    }
}

float get_position()
{
    int raw = TIM2->CNT;
    if (raw < 0) raw += CPR;
    if (raw >= CPR) raw -= CPR;
    float signed_elec = fmod((POLE_PAIRS / RESOLVER_LOBES * (6.28318530718f * (raw) / (float)CPR)), 6.28318530718f);
    if (signed_elec < 0) 
    {
        return signed_elec + 6.28318530718f;
    }
    else 
    {
        return signed_elec;
    }
}


float position_last;
Timer vel_timer;

float get_velocity()
{
    int usecs = vel_timer.read_us();
    float dx =  get_position() - position_last;
    if(dx >  M_PI) dx -= 2 * M_PI;
    if(dx < -M_PI) dx += 2 * M_PI;
    vel_timer.stop();
    vel_timer.reset();
    vel_timer.start();
    position_last = get_position();
    return 1000000*dx/usecs;
}