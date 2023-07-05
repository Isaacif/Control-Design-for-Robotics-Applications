#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>

class TIMER_peripheral
{
    public: 
        tim_oc_id TIMER_SELECT;

        rcc_periph_clken RCC_TIMER_SELECT;

        uint32_t timer_register;
        uint32_t timer_counter_register;

        TIMER_peripheral(uint16_t TC_REG, uint32_t T_REG, 
                         rcc_periph_clken RCC_TSelect, float resolution);

        uint16_t timerGetElapsed();
        void timerEnable();
        void timerDisable();
        void timerReset();

    private:
        uint16_t frequency;

};