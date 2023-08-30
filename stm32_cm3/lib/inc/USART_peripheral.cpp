#include "USART_peripheral.hpp"

USART_peripheral::USART_peripheral(uint16_t USART_TX, uint16_t USART_RX, uint32_t GPORT_SELECT, 
                                   rcc_periph_clken RCC_USART, uint32_t USART_REG, 
                                   rcc_periph_clken RCC_GPORT_SELECT, uint32_t baudrate)
{
    usart_pin_tx = USART_TX, usart_pin_rx = USART_RX, RCC_GPIO_PORT_SELECT = RCC_GPORT_SELECT;
    RCC_USART_SELECT = RCC_USART, usart_register = USART_REG, usart_baudrate = baudrate;
    GPIO_PORT_SELECT = GPORT_SELECT;

    rcc_periph_clock_enable(RCC_USART);
    rcc_periph_clock_enable(RCC_GPORT_SELECT);

    gpio_set_mode(GPIO_PORT_SELECT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USART_TX | USART_RX);

    usart_set_baudrate(usart_register, usart_baudrate);
    usart_set_databits(usart_register, data_bits);
    usart_set_stopbits(usart_register, USART_STOPBITS_1);
    usart_set_parity(usart_register, USART_PARITY_NONE);
    
    nvic_enable_irq(NVIC_USART1_IRQ);
    usart_enable_rx_interrupt(usart_register);

    usart_set_mode(usart_register, USART_MODE_TX_RX);
    usart_enable(usart_register);

}

void USART_peripheral::usartSend_char(const char *characters)
{
    while(*characters != '\0')
    {
        usart_send_blocking(usart_register, (uint16_t)(*characters));
        ++characters;
    }
    usart_send_blocking(usart_register, '\n');
}

void USART_peripheral::usartSend_integer(int value) 
{
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%d\n", value);

    int i = 0;
    while (buffer[i] != '\0') 
    {
        usart_send_blocking(usart_register, buffer[i]);
        i++;
    }
}
                            