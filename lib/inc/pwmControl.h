#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include <stdint.h>

class pwmControl
{
    public:
        tim_oc_id TIMER_SELECT;

        rcc_periph_clken RCC_TIMER_SELECT;
        rcc_periph_clken RCC_GPIO_PORT_SELECT;

        uint32_t GPIO_PORT_SELECT;
        uint32_t GPIO_PIN_SELECT;
        uint32_t GPIO_OUTPUT_MODE;
        uint32_t GPIO_SPEED;

        uint32_t TIMER_REG;
        uint32_t TIMER_COUNTER_REG;
        uint32_t GPIO_REG;

        pwmControl(tim_oc_id T_Select,  uint32_t TC_REG, uint32_t T_REG, rcc_periph_clken RCC_TSelect, 
        uint32_t GPIN_Select, uint32_t GPORT_Select, uint32_t GOUTPUT_Mode,
        uint32_t GSpeed, rcc_periph_clken RCC_GPort_Select);
        
        void pwmStart(uint8_t duty_cycle);
        void pwmStop();
        
    
    private:
       const uint32_t PRESCALER = 72; 
       const uint32_t COUNT_UP_TO = 50000;

       void pwmTimer_reset();
};