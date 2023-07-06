#include "SYS_TIMER_peripheral.hpp"

SYS_TIMER_peripheral::SYS_TIMER_peripheral(uint16_t system_frequency)
{
    system_tick_frequency = system_frequency;

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);     
    systick_clear();                                    
    systick_set_reload(rcc_ahb_frequency / system_tick_frequency - 1);   
    systick_interrupt_enable();                         
    systick_counter_enable();   

}
