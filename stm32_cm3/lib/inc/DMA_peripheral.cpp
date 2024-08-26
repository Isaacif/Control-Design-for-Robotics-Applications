#include "DMA_peripheral.hpp"

DMA_peripheral::DMA_peripheral(uint32_t DMA_SELECT, uint32_t DMA_CHANNEL_SELECT, uint32_t T_REGISTER, rcc_periph_clken DMA_CLOCK)
{
    DMA_PERIPHERAL_SELECT = DMA_SELECT, DMA_CHANNEL = DMA_CHANNEL_SELECT, TARGET_REGISTER = T_REGISTER;

    rcc_periph_clock_enable(DMA_CLOCK);

    /* Reset DMA stream for ADC */
    dma_stream_reset(DMA_PERIPHERAL_SELECT, DMA_CHANNEL);

    /* Set up DMA for ADC */
    dma_set_peripheral_address(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, (uint32_t)&TARGET_REGISTER);
    //dma_set_memory_address(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, (uint32_t)this->memory_buffer);
    dma_set_number_of_data(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, MEMORY_LENGTH); // Transfer 2 half-words (16-bit) from ADC DR to buffer
    dma_set_priority(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, DMA_SxCR_PL_LOW); // Low priority
    dma_set_memory_size(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, DMA_SxCR_MSIZE_16BIT); // Memory size 16-bit
    dma_set_peripheral_size(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, DMA_SxCR_PSIZE_16BIT); // Peripheral size 16-bit
    dma_enable_memory_increment_mode(DMA_PERIPHERAL_SELECT, DMA_CHANNEL); // Increment memory address
    dma_disable_peripheral_increment_mode(DMA_PERIPHERAL_SELECT, DMA_CHANNEL); // Do not increment peripheral address
    dma_enable_circular_mode(DMA_PERIPHERAL_SELECT, DMA_CHANNEL); // Circular mode
    //dma_enable_transfer_complete_interrupt(DMA_PERIPHERAL_SELECT, DMA_CHANNEL); // Enable transfer complete interrupt

    /* Enable DMA stream */
    //dma_enable_stream(DMA_PERIPHERAL_SELECT, DMA_CHANNEL);
    //dma_enable_channel(DMA_PERIPHERAL_SELECT, DMA_PERIPHERAL_SELECT);
}

void DMA_peripheral::DMA_initialize(uint32_t DMA_SELECT, uint32_t DMA_MODE, uint32_t T_REGISTER, rcc_periph_clken DMA_CLOCK)
{
    DMA_PERIPHERAL_SELECT = DMA_SELECT, DMA_CHANNEL = DMA_MODE, TARGET_REGISTER = T_REGISTER;

    rcc_periph_clock_enable(DMA_CLOCK);
    //nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);

    /* Reset DMA stream for ADC */
    dma_stream_reset(DMA_PERIPHERAL_SELECT, DMA_CHANNEL);

    /* Set up DMA for ADC */
    dma_set_priority(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, DMA_SxCR_PL_LOW); // Low priority
    dma_set_memory_size(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, DMA_SxCR_MSIZE_16BIT); // Memory size 16-bit
    dma_set_peripheral_size(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, DMA_SxCR_PSIZE_16BIT); // Peripheral size 16-bit
    dma_set_memory_address(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, (uint32_t) memory_buffer);
    dma_set_number_of_data(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, MEMORY_LENGTH); // Transfer 2 half-words (16-bit) from ADC DR to buffer
    dma_enable_memory_increment_mode(DMA_PERIPHERAL_SELECT, DMA_CHANNEL); // Increment memory address
    dma_disable_peripheral_increment_mode(DMA_PERIPHERAL_SELECT, DMA_CHANNEL); // Do not increment peripheral address
    
    dma_enable_circular_mode(DMA_PERIPHERAL_SELECT, DMA_CHANNEL); // Circular mode
    dma_set_transfer_mode(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_address(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, (uint32_t)&ADC1_DR);

    //dma_enable_transfer_complete_interrupt(DMA_PERIPHERAL_SELECT, DMA_CHANNEL); // Enable transfer complete interrupt
    dma_channel_select(DMA_PERIPHERAL_SELECT, DMA_CHANNEL, DMA_SxCR_CHSEL_0);
	dma_enable_stream(DMA_PERIPHERAL_SELECT, DMA_CHANNEL);
}