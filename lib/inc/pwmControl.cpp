#include "pwmControl.h"

pwmControl::pwmControl(tim_oc_id T_Select,  uint32_t TC_REG, uint32_t T_REG, rcc_periph_clken RCC_TSelect, 
         uint32_t GPIN_Select, uint32_t GPORT_Select, uint32_t GOUTPUT_Mode,
         uint32_t GSpeed, rcc_periph_clken RCC_GPort_Select)
{
    TIMER_SELECT = T_Select, TIMER_COUNTER_REG = TC_REG; 
    TIMER_REG = T_REG, RCC_TIMER_SELECT = RCC_TSelect;
    
    GPIO_PIN_SELECT = GPIN_Select, GPIO_PORT_SELECT = GPORT_Select;
    GPIO_OUTPUT_MODE = GOUTPUT_Mode, GPIO_SPEED = GSpeed;
    RCC_GPIO_PORT_SELECT = RCC_GPort_Select;

    rcc_periph_clock_enable(RCC_TIMER_SELECT);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_GPIO_PORT_SELECT);

    gpio_set_mode(GPIO_PORT_SELECT, GPIO_SPEED, GPIO_OUTPUT_MODE, GPIO_PIN_SELECT);

    timer_disable_counter(TIMER_REG);
    pwmTimer_reset();
    timer_set_mode(TIMER_REG, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIMER_REG, PRESCALER);
    timer_enable_preload(TIMER_REG);
    timer_continuous_mode(TIMER_REG);
    timer_set_period(TIMER_REG, COUNT_UP_TO);

    timer_disable_oc_output(TIMER_REG, TIMER_SELECT);
    timer_set_oc_mode(TIMER_REG, TIMER_SELECT, TIM_OCM_PWM1);
    timer_enable_oc_output(TIMER_REG, TIMER_SELECT);

}   

void pwmControl::pwmTimer_reset()
{
    TIMER_COUNTER_REG = 0;
}

void pwmControl::pwmStart(uint8_t duty_cycle)
{
    uint32_t busy_time = (duty_cycle*COUNT_UP_TO)/100;
    timer_set_oc_value(TIMER_REG, TIMER_SELECT, busy_time);
    timer_enable_counter(TIMER_REG);
}

void pwmControl::pwmStop()
{
    timer_disable_counter(TIMER_REG);
}