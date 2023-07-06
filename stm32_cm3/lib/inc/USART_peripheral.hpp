#ifndef USART_peripheral_HPP
#define USART_peripheral_HPP

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

class USART_peripheral
{
    public:
        rcc_periph_clken RCC_USART_SELECT;
        rcc_periph_clken RCC_GPIO_PORT_SELECT;

        uint16_t usart_register;
        uint16_t usart_baudrate;
        uint8_t usart_pin_tx;
        uint8_t usart_pin_rx;

        USART_peripheral(uint8_t USART_TX, uint8_t USART_RX, uint32_t GPORT_SELECT, 
                         rcc_periph_clken RCC_USART, uint16_t USART_REG, 
                         rcc_periph_clken RCC_GPORT_SELECT, uint16_t baudrate);

        inline void usartSend(char character);

    private:
        uint8_t data_bits = 8;

};


#endif