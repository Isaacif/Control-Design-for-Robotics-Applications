#define StateGain 0.2
#define SetPoint 430

#include "PWM_peripheral.hpp"
#include "ADC_peripheral.hpp"
#include "SYS_TIMER_peripheral.hpp"
#include "USART_peripheral.hpp"
#include <string.h>



/* Timer 4 and Timer 3 are reserved for PWM applications
   Timer 2 and other timers can be used for general tasks
   Sys_Tick timer is being used for general counter control
*/

#define LED_PIN GPIO6
#define LED_PORT GPIOB
#define USART_PORT USART1

#define system_frequency_10Khz    1e4

const char *message = "alive";


SYS_TIMER_peripheral system_counter(system_frequency_10Khz);
USART_peripheral serial_interface(GPIO_USART1_TX, GPIO_USART1_RX, GPIOA, 
                                  RCC_USART1, USART1, RCC_GPIOA, 115200);  

PWM_peripheral  pwm_timer_4(TIM4, TIM4_CNT, RCC_TIM4);

int16_t i;
int16_t blink_flag = 0;
uint32_t now;

volatile char web_server_buffer[MAX_RX_BUFFER_SIZE];
volatile uint16_t rx_buffer_index = 0;
volatile uint8_t rx_data_ready = 0; 
volatile char receivedChar = '\0';

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
}

void usart1_isr(void) 
{
    if (usart_get_flag(USART1, USART_SR_RXNE)) 
    {
        receivedChar = usart_recv(USART1);
        usart_send_blocking(USART1, receivedChar);
        gpio_toggle(LED_PORT, LED_PIN);
    }
}

void sys_tick_handler(void)
{
    g_counter_millis++;
}

int main(void)
{
    gpio_setup();
    while(true)
    {

    }
}