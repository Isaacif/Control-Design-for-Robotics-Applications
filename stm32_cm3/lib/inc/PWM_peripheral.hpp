/**
 * @file PWM_peripheral.hpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief Head file for the PWM peripheral class drive.
 * @version 0.1
 * @date 2023-07-02
 * 
 */

#ifndef PWM_peripheral_HPP
#define PWM_peripheral_HPP

#define DAC(voltage)    5461*12  // Converts voltage value(0-12V) to digital 


//*****************************************************************************
//  Using libopencm3 for peripheral interface.
//  timer.h General Timers file.
//  gpio.h General Purpose Input Output file.
//  rcc.h  Real Time Clock Control file.
//  nvic.h Nested Vectored Interrupt Controller file.
//*****************************************************************************

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include <stdint.h>             // Using std number format

/**
 * @brief PWM_peripheral class
 * Joins all PWM configurations 
 */
class PWM_peripheral
{
    public:
        tim_oc_id TIMER_SELECT;

        rcc_periph_clken RCC_TIMER_SELECT;
        rcc_periph_clken RCC_GPIO_PORT_SELECT;

        uint32_t timer_register;
        uint32_t timer_counter_register;

        PWM_peripheral(uint32_t T_REG, uint32_t TC_REG, rcc_periph_clken RCC_TSelect);
        
        void gpioSetup(tim_oc_id timer_select, uint32_t GPIO_Port_Select, 
                       uint32_t GPIO_Pin_Select, rcc_periph_clken RCC_GPort_Select);

        void pwmWrite(float duty_cycle, tim_oc_id timer_select);
        void pwmStop();
        
    
    private:
       const uint32_t PRESCALER = 72; 
       const uint32_t COUNT_UP_TO = 1000;

       void pwmTimer_reset();
};

#endif