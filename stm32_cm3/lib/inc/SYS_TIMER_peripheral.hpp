#ifndef SYS_TIMER_peripheral_HPP
#define SYS_TIMER_peripheral_HPP

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>

#define SYSTEM_COUNT_1S       10000
#define SYSTEM_COUNT_1mS        10
#define SYSTEM_COUNT_100uS      1

#define SYSTEM_TICK_MS(ms)      ms*SYSTEM_COUNT_1mS

static volatile uint32_t g_counter_millis = 0;
class SYS_TIMER_peripheral
{
    public:      
        uint16_t system_tick_frequency;

        SYS_TIMER_peripheral(uint16_t system_frequency);
        
};

#endif