#include "TIMER_peripheral.h"

TIMER_peripheral::TIMER_peripheral(uint16_t TC_REG, uint32_t T_REG, 
                                   rcc_periph_clken RCC_TSelect, float resolution)
{
    timer_counter_register = TC_REG, timer_register = T_REG, RCC_TIMER_SELECT = RCC_TSelect;

    frequency = (1/resolution);

    rcc_periph_clock_enable(RCC_TIMER_SELECT);
    rcc_periph_reset_pulse(RST_TIM2);

    timer_set_mode(timer_register, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_set_prescaler(timer_register, ((rcc_apb1_frequency * 2) / frequency));
    timer_set_period(timer_register, 65535);

}

uint16_t TIMER_peripheral::timerGetElapsed()
{
    uint16_t current_time = timer_get_counter(timer_register);
    return current_time;
}

void TIMER_peripheral::timerEnable()
{
    timer_enable_counter(timer_register);
}

void TIMER_peripheral::timerDisable()
{
    timer_disable_counter(timer_register);
}

void TIMER_peripheral::timerReset()
{
  timer_set_counter(timer_register, 0);
}

