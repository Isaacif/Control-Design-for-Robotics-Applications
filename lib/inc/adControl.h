#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include <stdint.h>

class adControl
{
    public:

        rcc_periph_clken RCC_GPIO_PORT_SELECT;

        uint32_t GPIO_PORT_SELECT;
        uint16_t GPIO_PIN_SELECT;

        uint8_t ADC_CHANNEL_SELECT;

        adControl(uint8_t ADC_CHANNEL_SELECT, rcc_periph_clken RCC_GPSelect, 
                  uint32_t GPIO_PortSelect, uint32_t GPIO_PinSelect);
                  
        uint16_t adc_read();

    private:
       uint32_t rcc_apb2_frequency = 72000000;
};