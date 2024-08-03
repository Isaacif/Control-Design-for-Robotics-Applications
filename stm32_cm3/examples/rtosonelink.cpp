#define MICROSECONDS_PER_SECOND 1000000UL

#define MICROSECONDS_TO_TICKS(us) \
    ((TickType_t)(((us) * (TickType_t)configTICK_RATE_HZ) / MICROSECONDS_PER_SECOND))


#include "PWM_peripheral.hpp"
#include "ADC_peripheral.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include "USART_peripheral.hpp"
#include "DMA_peripheral.hpp"

ADC_peripheral  adc_port_a;
PWM_peripheral  pwm_timer_4;
USART_peripheral serial_interface;
DMA_peripheral dma1_interface;

int16_t input_function_stage = 0;
int8_t voltage;

extern "C" void input_function(void *pvParameters) 
{
    (void)pvParameters;

    while (1) 
    {
        if(input_function_stage <= 10)
        {
            voltage = 0;
        }
        else if(10 < input_function_stage && input_function_stage  <= 20)
        {
            voltage = 3.3;
        }
        else if(20 < input_function_stage && input_function_stage  <= 30)
        {
            voltage = 3.3 + 0.17*(input_function_stage - 10);
        }
        else if(30 < input_function_stage && input_function_stage  <= 50)
        {
            voltage = 6;
        }
        else if(50 < input_function_stage && input_function_stage  <= 60)
        {
            voltage = 5 - 0.17*(input_function_stage - 50);
        }
        else if(60 < input_function_stage && input_function_stage  <= 70)
        {
            voltage = 3.3;
        }
        else if(70 < input_function_stage && input_function_stage  <= 80)
        {
            voltage = 0;
        }
        else if(80 < input_function_stage && input_function_stage <= 90)
        {
            voltage = 0;
        }
        else if(90 < input_function_stage && input_function_stage  <= 100)
        {
            voltage = -3.3;
        }
        else if(100 < input_function_stage && input_function_stage  <= 110)
        {
            voltage = -(3.3 + 0.17*(input_function_stage - 100));
        }
        else if(110 < input_function_stage && input_function_stage  <= 130)
        {
            voltage = -6;
        }
        else if(130 < input_function_stage && input_function_stage  <= 140)
        {
            voltage = -(5 - 0.17*(input_function_stage - 130));
        }
        else if(140 < input_function_stage && input_function_stage  <= 150)
        {
            voltage = -3.3;
        }
        else if(150 < input_function_stage && input_function_stage  <= 160)
        {
            voltage = 0;
        }

        if(input_function_stage >= 160)
        {
            input_function_stage = 0;
        }

        input_function_stage++;
        vTaskDelay(MICROSECONDS_TO_TICKS(125));
    }
}


extern "C" void joint_read(void *pvParameters) 
{
    (void)pvParameters;

    while (1) 
    {
        serial_interface.usartSend_char("Joint one: ");
        serial_interface.usartSend_integer(dma1_interface.memory_buffer[0]);
        serial_interface.usartSend_char("Joint two: ");
        serial_interface.usartSend_integer(dma1_interface.memory_buffer[1]);
        vTaskDelay(MICROSECONDS_TO_TICKS(25));
    }
}

int main(void) 
{
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    adc_port_a.ADC_initialization(ADC1, RCC_ADC1, RCC_GPIOA, GPIOA, 1);
    adc_port_a.gpioSetup(GPIO1);
    adc_port_a.gpioSetup(GPIO0);
    uint8_t channels[] = {ADC_CHANNEL0, ADC_CHANNEL1};
    adc_set_regular_sequence(ADC1, 2 , channels);
    dma1_interface.DMA_initialize(DMA1, DMA_STREAM0, ADC1_DR, RCC_DMA1);

    pwm_timer_4.PWM_initialization(TIM4, TIM4_CNT, RCC_TIM4);
    pwm_timer_4.gpioSetup(TIM_OC1, GPIOB, GPIO6, RCC_GPIOB);
    pwm_timer_4.gpioSetup(TIM_OC4, GPIOB, GPIO8, RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);

    serial_interface.USART_initialization(GPIO2, GPIO3, GPIOA, 
                                  RCC_USART2, USART2, RCC_GPIOA, 115200); 
        
    xTaskCreate(joint_read, "joint", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    //xTaskCreate(input_function, "input", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    // Should not reach here
    while (1);

    return 0;
}