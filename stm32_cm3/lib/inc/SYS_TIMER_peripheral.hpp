/**
 * @file SYS_TIMER_peripheral.hpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief Head file for the SYS_TICK peripheral class drive.
 * @version 0.1
 * @date 2023-07-02
 * 
 */

#ifndef SYS_TIMER_peripheral_HPP
#define SYS_TIMER_peripheral_HPP

//*****************************************************************************
// Macros for Time counter register convertion.
//*****************************************************************************


#define SYSTEM_COUNT_1S         10000       // One second.
#define SYSTEM_COUNT_1mS        10          // One milisecond.
#define SYSTEM_COUNT_100uS      1           // a hundred microseconds (max timer resolution)

#define SYSTEM_TICK_MS(msec)      SYSTEM_COUNT_1mS*msec  // Convert value to miliseconds.
#define SYSTEM_TICK_SEC(sec)        SYSTEM_COUNT_1S*sec  // Convert value to seconds.

//*****************************************************************************
//  Using libopencm3 for peripheral interface.
//  timer.h General Timers file.
//  rcc.h  Real Time Clock Control file.
//  nvic.h Nested Vectored Interrupt Controller file.
//  systick.h System Tick Timer file.
//*****************************************************************************

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

static volatile uint32_t g_counter_millis = 0;      // Global referente timer counter

/**
 * @brief SYS_TIMER peripheral class
 * Joins all SYS_TICK configurations 
 */
class SYS_TIMER_peripheral
{
    public:      
        uint16_t system_tick_frequency;

        SYS_TIMER_peripheral(uint16_t system_frequency);
        
};

#endif