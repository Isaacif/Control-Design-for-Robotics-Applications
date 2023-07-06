#define StateGain 0.2
#define SetPoint 430

#include "PWM_peripheral.hpp"
#include "ADC_peripheral.hpp"
#include "SYS_TIMER_peripheral.hpp"


/* Timer 4 and Timer 3 are reserved for PWM applications
   Timer 2 and other timers can be used for general tasks
   Sys_Tick timer is being used for general counter control
*/

#define LED_PIN GPIO6
#define LED_PORT GPIOB

#define system_frequency_10Khz    1e4

SYS_TIMER_peripheral system_counter(system_frequency_10Khz);
PWM_peripheral  pwm_timer_4(TIM4, TIM4_CNT, RCC_TIM4);

int16_t i;

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
}

void sys_tick_handler(void)
{
    g_counter_millis++;
}

int main(void)
{
    gpio_setup();
    pwm_timer_4.gpioSetup(TIM_OC1, GPIOB, GPIO6, RCC_GPIOB);
    while(true)
    {
        for(i = 100; i >= 0; i--)
        {
            while (1)
            {
                if(g_counter_millis > SYSTEM_TICK_MS(25))
                {
                    pwm_timer_4.pwmWrite(i, TIM_OC1);
                    g_counter_millis = 0;
                    break;
                }
            }
        }
        while(g_counter_millis < SYSTEM_TICK_MS(500));
        g_counter_millis = 0;
    }
}