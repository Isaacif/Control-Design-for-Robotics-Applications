#define StateGain 0.2
#define SetPoint 430

#include "pwmControl.h"
#include "adControl.h"

int main(void)
{
    uint16_t sensor1; 
    int16_t error;
    pwmControl PWMtimer_4(TIM4_CNT, TIM4, RCC_TIM4);
    pwmControl PWMtimer_3(TIM3_CNT, TIM3, RCC_TIM3);

    PWMtimer_4.gpioSetup(TIM_OC1, GPIO6, GPIOB, RCC_GPIOB);
    PWMtimer_4.gpioSetup(TIM_OC2, GPIO7, GPIOB, RCC_GPIOB);
    PWMtimer_4.gpioSetup(TIM_OC3, GPIO8, GPIOB, RCC_GPIOB);
    PWMtimer_4.gpioSetup(TIM_OC4, GPIO9, GPIOB, RCC_GPIOB);

    PWMtimer_4.pwmWrite(100, TIM_OC1);
    PWMtimer_4.pwmWrite(5, TIM_OC2);
    PWMtimer_4.pwmWrite(35, TIM_OC3);
    PWMtimer_4.pwmWrite(75, TIM_OC4);

    PWMtimer_3.gpioSetup(TIM_OC4, GPIO1, GPIOB, RCC_GPIOB);
    PWMtimer_3.pwmWrite(100, TIM_OC4);
}