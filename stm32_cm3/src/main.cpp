#define StateGain 0.2
#define SetPoint 430

#include "PWM_peripheral.hpp"
#include "ADC_peripheral.hpp"
#include "TIMER_peripheral.hpp"


/* Timer 4 and Timer 3 are reserved for PWM applications
   Timer 2 and other timers can be used for general tasks
   Timer 2 is being used for time schedule control
*/

#define LED_PIN GPIO6
#define LED_PORT GPIOB

#define TIMER_RESOLUTION_10uS  1e-5

#define TIMER_COUNTER_1ms      1000
#define TIMER_COUNTER_1s      10000

#define tick_ms(ms)            ms*1000        

uint32_t g_counter_refence = 0;

void tim2_isr(void) 
{
    if (timer_get_flag(TIM2, TIM_SR_UIF)) 
    {
        timer_clear_flag(TIM2, TIM_SR_UIF);
        g_counter_refence++;
    }
}

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
   
   while(true)
   {
        if(g_counter_refence >= tick_ms(120))
        {
            gpio_toggle(GPIOB, GPIO6);
        }
   }
}
