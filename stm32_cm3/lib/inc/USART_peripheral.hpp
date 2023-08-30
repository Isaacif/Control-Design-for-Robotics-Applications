#ifndef USART_peripheral_HPP
#define USART_peripheral_HPP

#define MAX_RX_BUFFER_SIZE         32

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <string.h>
#include <cstdio>
class USART_peripheral
{
    public:
        rcc_periph_clken RCC_USART_SELECT;
        rcc_periph_clken RCC_GPIO_PORT_SELECT;
        uint32_t GPIO_PORT_SELECT;

        uint32_t usart_register;
        uint32_t usart_baudrate;
        uint8_t usart_pin_tx;
        uint8_t usart_pin_rx;

        USART_peripheral(uint16_t USART_TX, uint16_t USART_RX, uint32_t GPORT_SELECT, 
                         rcc_periph_clken RCC_USART, uint32_t USART_REG, 
                         rcc_periph_clken RCC_GPORT_SELECT, uint32_t baudrate);
        
        
        void usartSend_char(const char *characters);
        void usartSend_integer(int value); 

    private:
        uint32_t data_bits = 8;

};


#endif