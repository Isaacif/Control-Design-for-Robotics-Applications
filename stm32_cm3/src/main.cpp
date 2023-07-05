#define StateGain 0.2
#define SetPoint 430

#include "PWM_peripheral.h"
#include "ADC_peripheral.h"
#include "TIMER_peripheral.h"


/* Timer 4 and Timer 3 are reserved for PWM applications
   Timer 2 and other timers can be used for general tasks
   Timer 2 is being used for time schedule control
*/

#define LED_PIN GPIO6
#define LED_PORT GPIOB

#define TIMER_RESOLUTION_10uS 1e-4
#define TIMER_COUNTER_1S      10000

uint8_t activate_flag = 0;

int main(void)
{
    uint16_t sensor1; 
    uint16_t counting;
    //PWM_peripheral PWMtimer_4(TIM4_CNT, TIM4, RCC_TIM4);
    //ADC_peripheral ADC1_control(ADC1, RCC_ADC1, RCC_GPIOA, GPIOA);
    TIMER_peripheral Time_Counter(TIM2_CNT, TIM2, RCC_TIM2, TIMER_RESOLUTION_10uS);

    //ADC1_control.gpioSetup(GPIO1);

    //PWMtimer_4.gpioSetup(TIM_OC1, GPIO6, GPIOB, RCC_GPIOB);
    //PWMtimer_4.pwmWrite(100, TIM_OC1);

    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
    //gpio_clear(LED_PORT, LED_PIN);

    Time_Counter.timerEnable();
    gpio_set(LED_PORT, LED_PIN);

    while(true)
    {   
        counting = timer_get_counter(TIM2);
        if(counting >= TIMER_COUNTER_1S)
        {   
            if(activate_flag == 0)
            {
                gpio_set(LED_PORT, LED_PIN);
                activate_flag = 1;
                timer_set_counter(TIM2, 0);
            }
            else if(activate_flag == 1)
            {
                gpio_clear(LED_PORT, LED_PIN);
                activate_flag = 0;
                timer_set_counter(TIM2, 0);
            }
        }
    }
}