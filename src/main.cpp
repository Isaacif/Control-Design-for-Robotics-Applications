#include "FreeRTOS.h"
#include "pwmControl.h"

int main(void)
{
    pwmControl PWMm1 = pwmControl(TIM_OC2, TIM2_CNT, TIM2, RCC_TIM2,
    GPIO1, GPIOA, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
    GPIO_MODE_OUTPUT_50_MHZ, RCC_GPIOA);

    PWMm1.pwmStart(50);
}