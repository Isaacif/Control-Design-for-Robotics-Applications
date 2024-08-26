#include <libopencm3/stm32/rcc.h>
#include "PWM_peripheral.hpp"

PWM_peripheral pwm;

int main(void) 
{
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]); // Set system clock to 84MHz
    rcc_periph_clock_enable(RCC_GPIOB);
    pwm.gpioSetup(TIM_OC1, GPIOB, GPIO6, RCC_GPIOB);
    pwm.PWM_initialization(TIM4, TIM4_CNT, RCC_TIM4);
    pwm.pwmWrite(50, TIM_OC1);

    while(1) {
        // Your main code goes here
    }

    return 0;
}