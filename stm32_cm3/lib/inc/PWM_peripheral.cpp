/**
 * @file PWM_peripheral.cpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief Source file for the PWM peripheral class drive.
 * @version 0.1
 * @date 2023-07-02
 * 
 */

#include "PWM_peripheral.hpp"

/**
 * @brief PWM_peripheral constructor
 * @param T_REG Selects which TIMER will be used
 * @param TC_REG indicates the counter register for the selected TIMER
 * @param RCC_TSelect Selects which TIMER will be activated
 */

PWM_peripheral::PWM_peripheral(uint32_t T_REG, uint32_t TC_REG, rcc_periph_clken RCC_TSelect)
{
    timer_counter_register = TC_REG, timer_register = T_REG, RCC_TIMER_SELECT = RCC_TSelect;

    rcc_periph_clock_enable(RCC_TIMER_SELECT);
    rcc_periph_clock_enable(RCC_AFIO);

    timer_disable_counter(timer_register);
    
    pwmTimer_reset();

    timer_set_mode(timer_register, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(timer_register, PRESCALER);
    timer_enable_preload(timer_register);
    timer_continuous_mode(timer_register);
    timer_set_period(timer_register, COUNT_UP_TO);

}   

/**
 * @brief PWM_peripheral pwmTimer_reset()
 * manually resets the pwm Timer
 */

void PWM_peripheral::pwmTimer_reset()
{
    timer_counter_register = 0;
}

/**
 * @brief PWM_peripheral gpioSetup
 * @param timer_select Selects which TIMER CHANNEL will be used
 * @param GPIO_Port_Select Selects which GPIO PORT will be used
 * @param GPIO_Pin_Select Selects which GPIO PIN will be used
 * @param RCC_GPort_Select Selects which GPIO PORT will be activated
 */

void PWM_peripheral::gpioSetup(tim_oc_id timer_select, uint32_t GPIO_Port_Select, 
                               uint32_t GPIO_Pin_Select, rcc_periph_clken RCC_GPort_Select)
{
    rcc_periph_clock_enable(RCC_GPort_Select);
    gpio_set_mode(GPIO_Port_Select, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_Pin_Select);

    timer_set_oc_mode(timer_register, timer_select, TIM_OCM_PWM1);

}

/**
 * @brief PWM_peripheral pwmWrite
 * @param timer_select Selects which TIMER CHANNEL will be used
 * @param duty_cycle Sets up the duty_cycle value (0-100%)
 */


void PWM_peripheral::pwmWrite(float duty_cycle, tim_oc_id timer_select)
{
    //uint32_t busy_time = (duty_cycle*COUNT_UP_TO)/100;
    uint32_t busy_time = duty_cycle;
    timer_set_oc_value(timer_register, timer_select, busy_time);
    timer_enable_counter(timer_register);
    timer_enable_oc_output(timer_register, timer_select);
    timer_enable_oc_preload(timer_register, timer_select);
}

/**
 * @brief PWM_peripheral pwmTimer_reset()
 * manually stops the pwm timer
 */

void PWM_peripheral::pwmStop()
{
    timer_disable_counter(timer_register);
}