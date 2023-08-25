/**
 * @file SYS_TIMER_peripheral.cpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief Source file for the SYS_TICK peripheral class drive.
 * @version 0.1
 * @date 2023-07-02
 * 
 */

#include "SYS_TIMER_peripheral.hpp"

/**
 * @brief SYS_TIMER_peripheral constructor
 * @param system_frequency Selects system counter frequency
 */

SYS_TIMER_peripheral::SYS_TIMER_peripheral(uint16_t system_frequency)
{
    system_tick_frequency = system_frequency;

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);     
    systick_clear();                                    
    systick_set_reload(rcc_ahb_frequency / system_tick_frequency - 1);   
    systick_interrupt_enable();                         
    systick_counter_enable();   

    nvic_set_priority(NVIC_SYSTICK_IRQ, 1);

}
