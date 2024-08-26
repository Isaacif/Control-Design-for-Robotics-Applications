#define StateGain   0.2
#define SetPoint    430
#define dataSize    10000

#include "PWM_peripheral.hpp"
#include "ADC_peripheral.hpp"
#include "SYS_TIMER_peripheral.hpp"
#include "USART_peripheral.hpp"
#include "DMA_peripheral.hpp"
#include "algorithms.hpp"
#include "controller.hpp"

/* Timer 4 and Timer 3 are reserved for PWM applications
   Timer 2 and other timers can be used for general tasks
   Sys_Tick timer is being used for general counter control
*/

#define LED_PIN GPIO13
#define LED_PORT GPIOC
#define samplenumb  500

#define system_frequency_1Mhz    1e6

SYS_TIMER_peripheral system_counter;
ADC_peripheral  adc_port_a;
PWM_peripheral  pwm_timer_4;
USART_peripheral serial_interface;
DMA_peripheral dma2_interface;
servoIn_Controller servo_system;
controller joint_controller;
ISubject system_manager;

int16_t input_function_stage = 0;
int16_t joint_one = 0;
int16_t joint_two = 0;
int16_t joint_one_h = 0;
int16_t joint_two_h = 0;
int32_t joint_one_s = 0;
int16_t joint_two_s = 0;

uint16_t pwm_value1;
uint16_t pwm_value2;
int8_t voltage1;
int8_t voltage2;
uint16_t reading;
int16_t on = 0;
uint32_t samplesnumb = 0;
uint32_t time_reference1 = 0;
uint32_t time_reference2 = 0;
uint8_t GLOBAL_LINK_SETPOINT = 0;
int8_t ended = 0;
int8_t side = 1;
int8_t stop = 0;

void sys_tick_handler(void)
{
    g_counter_millis++;
}
void joint_read() 
{
    joint_one_h = 0;
    joint_two_h = 0;
    joint_one_s = 0;
    joint_two_s = 0;

    for(uint8_t k = 0; k < 10; k++)
    {
        joint_one_h=dma2_interface.memory_buffer[0];
        joint_two_h=dma2_interface.memory_buffer[1];   
        joint_two_h-=1580;
        if(joint_one_h < 0)
        {
            joint_one_h = 0;
        }
        if(joint_two_h < 0)
        {
            joint_two_h = 0;
        }
        joint_one_s+=joint_one_h;        
        joint_two_s+=joint_two_h;
    }   

    joint_one = joint_one_s*0.007 - 90;
    joint_two = joint_two_s*0.00825 - 90;

}

int main(void)
{
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    system_counter.SYS_TIMER_initialize(system_frequency_1Mhz);
    dma2_interface.DMA_initialize(DMA2, DMA_STREAM0, ADC1_DR, RCC_DMA2);

    adc_port_a.ADC_initialization(ADC1, RCC_ADC1, RCC_GPIOA, GPIOA, 1);
    adc_port_a.gpioSetup(GPIO1);
    adc_port_a.gpioSetup(GPIO0);

    pwm_timer_4.PWM_initialization(TIM4, TIM4_CNT, RCC_TIM4);
    rcc_periph_clock_enable(RCC_GPIOB);
    pwm_timer_4.gpioSetup(TIM_OC1, GPIOB, GPIO6, RCC_GPIOB);
    pwm_timer_4.gpioSetup(TIM_OC2, GPIOB, GPIO7, RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP,
                    GPIO_OSPEED_25MHZ,  GPIO8);
    serial_interface.USART_initialization(GPIO2, GPIO3, GPIOA, 
                                  RCC_USART2, USART2, RCC_GPIOA, 115200); 

    gpio_clear(GPIOB, GPIO8);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP,
                    GPIO_OSPEED_25MHZ,  GPIO9);
    gpio_clear(GPIOB, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO10);
    servo_system.servoIn_initialize(0, 0);
    joint_controller.controller_initialize(0, 1, GPIOB, GPIO8, GPIOB, GPIO9, &adc_port_a,
                                           &servo_system, &pwm_timer_4, RCC_GPIOB);        
    
    while (true)
    {
        reading = gpio_get(GPIOA, GPIO10);
        if(reading == 0)
        {
            break;
        }
    }

    while(true)
    {
          
        if(g_counter_millis - time_reference1 > SYSTEM_TICK_MS(5))
        {
            joint_read();
            joint_controller.loopDMA(joint_one, joint_two);
            time_reference1 = g_counter_millis;
        }

        if(g_counter_millis - time_reference2 > SYSTEM_TICK_MS(275))
        {
            serial_interface.usartSend_char("1: ");
            serial_interface.usartSend_integer(joint_one);
            serial_interface.usartSend_char("2: ");
            serial_interface.usartSend_integer(joint_two);
            time_reference2 = g_counter_millis;
        }
    }
}