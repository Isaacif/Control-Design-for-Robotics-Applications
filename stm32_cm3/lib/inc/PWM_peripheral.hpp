#ifndef PWM_peripheral_HPP
#define PWM_peripheral_HPP

#define DAC(voltage)    5461*12


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include <stdint.h>

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