/**
 * @file main.cpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief Source file for the stm32 firmware
 * main project file: 
 * Implements all the setup communication with the HTTP server.
 * All the drivers and algorithms for the necessary interface.
 * @version 0.1
 * @date 2023-07-02
 */


//*****************************************************************************
// Macros setup for main function
//*****************************************************************************

#define StateGain 0.2
#define SetPoint 430

#define LED_PIN GPIO8
#define LED_PORT GPIOB
#define USART_PORT USART1

#define system_frequency_10Khz    1e4
#define system_frequency_1Khz    1e3

//*****************************************************************************
// importing all the implemented drivers classes 
// as well as other userful modules
//*****************************************************************************


#include "PWM_peripheral.hpp"
#include "ADC_peripheral.hpp"
#include "SYS_TIMER_peripheral.hpp"
#include "USART_peripheral.hpp"
#include "controller.hpp"
#include "algorithms.hpp"


/* Timer 4 and Timer 3 are reserved for PWM applications
   Timer 2 and other timers can be used for general tasks
   Sys_Tick timer is being used for general counter control
*/

const char *message = "alive";

//*****************************************************************************
// Create all the drivers classes
//*****************************************************************************

ADC_peripheral  adc_port_a(ADC1, RCC_ADC1, RCC_GPIOA, GPIOA);
PWM_peripheral  pwm_timer_4(TIM4, TIM4_CNT, RCC_TIM4);
SYS_TIMER_peripheral system_counter(system_frequency_10Khz);
USART_peripheral serial_interface(GPIO_USART1_TX, GPIO_USART1_RX, GPIOA, 
                                  RCC_USART1, USART1, RCC_GPIOA, 115200);  

ADPI_Controller motor_controller_one(1.35, 0.05, 3200);
ISubject system_manager;

int16_t i;
int16_t blink_flag = 0;
uint32_t now;

float sensor_k;
float pwm_value_k;

volatile char web_server_buffer[MAX_RX_BUFFER_SIZE];
volatile uint16_t rx_buffer_index = 0;
volatile uint8_t rx_data_ready = 0; 
volatile char receivedChar = '\0';

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
    GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
}

/**
 * @brief usart1_isr
 * usart message interrupt handler
 */

void usart1_isr(void) 
{
    if (usart_get_flag(USART1, USART_SR_RXNE)) 
    {
        receivedChar = usart_recv(USART1);
        usart_send_blocking(USART1, receivedChar);
        gpio_toggle(LED_PORT, LED_PIN);
    }
}

/**
 * @brief sys_tick_handler
 * system counter interrupt handler
 */

void sys_tick_handler(void)
{
    g_counter_millis++;
}

int main(void)
{
    int value = 200;
    uint16_t bytes[sizeof(float)];
    gpio_setup();
    pwm_timer_4.gpioSetup(TIM_OC1, GPIOB, GPIO6, RCC_GPIOB);
    //pwm_timer_4.gpioSetup(TIM_OC2, GPIOB, GPIO8, RCC_GPIOB);
    adc_port_a.gpioSetup(GPIO1);
    //adc_port_a.gpioSetup(GPIO2);
    gpio_clear(GPIOB, GPIO7);
    gpio_set(GPIOB, GPIO5);

    controller joint_one(0, GPIOB, GPIO7, 
                         &adc_port_a, &motor_controller_one,
                         &pwm_timer_4, RCC_GPIOB);

    /* controller joint_h(1, GPIOB, GPIO9, 
                         &adc_port_a, &motor_controller_one,
                         &pwm_timer_4, RCC_GPIOB);
                         */

    joint_one.time_period = g_counter_millis;
    //joint_h.time_period = g_counter_millis;
    system_manager.Attach(&joint_one);
    //system_manager.Attach(&joint_h);

    while(true)
    {  
        if(g_counter_millis > 1000)
        {
            joint_one.time_period = g_counter_millis - joint_one.time_period;
            joint_one.loop();
            send_integer(circularBufferGetRelativeElement(&(motor_controller_one.e), -1), USART1);
            g_counter_millis = 0;
        }
    }
}