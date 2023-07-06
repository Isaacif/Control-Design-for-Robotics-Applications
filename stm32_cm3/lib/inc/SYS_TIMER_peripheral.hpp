#ifndef SYS_TIMER_peripheral_HPP
#define SYS_TIMER_peripheral_HPP

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>

static volatile uint32_t _millis = 0;

class SYS_TIMER_peripheral
{
    public: 
        
        uint16_t system_tick_frequency;

        SYS_TIMER_peripheral(uint16_t system_frequency);
        
};

#endif