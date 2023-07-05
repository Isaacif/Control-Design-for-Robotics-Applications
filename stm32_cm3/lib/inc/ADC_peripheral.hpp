#ifndef ADC_peripheral_HPP
#define ADC_peripheral_HPP

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include <stdint.h>

class ADC_peripheral
{
    public:
        rcc_periph_clken RCC_GPIO_PORT_SELECT;
        rcc_periph_clken RCC_ADC_SELECT;

        uint32_t GPIO_PORT_SELECT;
        uint8_t ADC_PERIPHERAL_SELECT;

        ADC_peripheral(uint32_t ADC_PSelect, rcc_periph_clken RCC_ADSelect,
                       rcc_periph_clken RCC_GPSelect, uint32_t GPIO_PortSelect);

        void gpioSetup(uint16_t GPIO_PIN_SELECT);
        uint16_t adc_read(uint8_t ADC_CHANNEL_SELECT);

    private:
       uint32_t rcc_apb2_frequency = 72000000;
};

#endif 