#ifndef DMA_peripheral_HPP
#define DMA_peripheral_HPP

#define MEMORY_LENGTH   2

#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>


#include <stdint.h>         // Using std number format

/**
 * @brief ADC_peripheral class
 * Joins all ADC configurations 
 */
class DMA_peripheral
{
    public:
        rcc_periph_clken RCC_GPIO_PORT_SELECT;
        rcc_periph_clken RCC_ADC_SELECT;

        uint32_t DMA_PERIPHERAL_SELECT;
        uint32_t DMA_CHANNEL;
        uint32_t TARGET_REGISTER;
        volatile uint16_t memory_buffer[MEMORY_LENGTH];

        DMA_peripheral(){}
        DMA_peripheral(uint32_t DMA_SELECT, uint32_t DMA_MODE, uint32_t T_REGISTER, rcc_periph_clken DMA_CLOCK);

        void DMA_initialize(uint32_t DMA_SELECT, uint32_t DMA_MODE, uint32_t T_REGISTER, rcc_periph_clken DMA_CLOCK);

    private:
       uint32_t rcc_apb2_frequency = 72000000;
};

#endif
