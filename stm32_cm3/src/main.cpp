#define StateGain 0.2
#define SetPoint 430

#include "PWM_peripheral.hpp"
#include "ADC_peripheral.hpp"
#include "SYS_TIMER_peripheral.hpp"


/* Timer 4 and Timer 3 are reserved for PWM applications
   Timer 2 and other timers can be used for general tasks
   Timer 2 is being used for time schedule control
*/

#define LED_PIN GPIO6
#define LED_PORT GPIOB

#define system_frequency_10Khz    1e4

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

void sys_tick_handler(void)
{
    _millis++;
}

int main(void)
{
    SYS_TIMER_peripheral System_Counter(system_frequency_10Khz);
    gpio_setup();

    while (1)
    {
        if(_millis > 1000)
        {
            //gpio_toggle(GPIOC, GPIO13);
            _millis = 0;
        }
    }
    return 0;
}