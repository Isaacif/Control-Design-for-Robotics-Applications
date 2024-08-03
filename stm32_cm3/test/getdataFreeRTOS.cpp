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
uint16_t joint_one = 0;
uint16_t joint_two = 0;
uint16_t pwm_value;
int8_t voltage;
uint16_t reading;
uint32_t samplesnumb = 0;

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) 
{
    // Handle or log the stack overflow as needed (or leave it empty).
}

extern "C" void input_function(void *pvParameters) 
{
    (void)pvParameters;

    while (1) 
    {
        if(input_function_stage <= 10 || (input_function_stage > 20 && input_function_stage <= 30))
        {
            voltage = 0;
            pwm_timer_4.pwmWrite(0, TIM_OC1);
            gpio_clear(GPIOB, GPIO8);
            pwm_timer_4.pwmWrite(0, TIM_OC2);
            gpio_clear(GPIOB, GPIO9);
        }
        else if(input_function_stage > 10 && input_function_stage <= 20)
        {
            voltage = 5;
            pwm_value = 41;
            pwm_timer_4.pwmWrite(pwm_value, TIM_OC1);
            gpio_clear(GPIOB, GPIO8);
            pwm_timer_4.pwmWrite(pwm_value, TIM_OC2);
            gpio_clear(GPIOB, GPIO9);
        }
        else if(input_function_stage > 30 && input_function_stage <= 40)
        {
            voltage = -5;
            pwm_value = 100 - 41;
            pwm_timer_4.pwmWrite(pwm_value, TIM_OC1);
            gpio_set(GPIOB, GPIO8);
            pwm_timer_4.pwmWrite(pwm_value, TIM_OC2);
            gpio_set(GPIOB, GPIO9);
        }


        if(input_function_stage >= 40)
        {
            input_function_stage = 0;
        }

        input_function_stage = input_function_stage + 1;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}


extern "C" void joint_read(void *pvParameters) 
{
    (void)pvParameters;

    while (1) 
    {
        if(samplesnumb < 60000)
        {
            joint_one=dma1_interface.memory_buffer[0];
            joint_two=dma1_interface.memory_buffer[1];
                

            joint_one = (joint_one - 960);
            joint_two = (joint_two - 1015);

            serial_interface.usartSend_char("Joint one: ");
            serial_interface.usartSend_integer(joint_one);
            serial_interface.usartSend_char("Joint two: ");
            serial_interface.usartSend_integer(joint_two);
            serial_interface.usartSend_char("voltage: ");
            serial_interface.usartSend_integer(voltage);
            vTaskDelay(MICROSECONDS_TO_TICKS(50));
            samplesnumb++;
        }
        else
        {
            serial_interface.usartSend_char("ENDED");
            break;
        }
    }
}

int main(void) 
{
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    dma1_interface.DMA_initialize(DMA2, DMA_STREAM0, ADC1_DR, RCC_DMA2);
    //dma_setup();
    adc_port_a.ADC_initialization(ADC1, RCC_ADC1, RCC_GPIOA, GPIOA, 1);
    adc_port_a.gpioSetup(GPIO1);
    adc_port_a.gpioSetup(GPIO0);

    pwm_timer_4.PWM_initialization(TIM4, TIM4_CNT, RCC_TIM4);
    rcc_periph_clock_enable(RCC_GPIOB);
    pwm_timer_4.gpioSetup(TIM_OC1, GPIOB, GPIO6, RCC_GPIOB);
    pwm_timer_4.gpioSetup(TIM_OC2, GPIOB, GPIO7, RCC_GPIOB);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP,
                    GPIO_OSPEED_25MHZ,  GPIO8);
    serial_interface.USART_initialization(GPIO2, GPIO3, GPIOA, 
                                  RCC_USART2, USART2, RCC_GPIOA, 115200); 

    gpio_clear(GPIOB, GPIO8);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP,
                    GPIO_OSPEED_25MHZ,  GPIO9);
    gpio_clear(GPIOB, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO10);

    while (true)
    {
        reading = gpio_get(GPIOA, GPIO10);
        if(reading == 0)
        {
            break;
        }
    }
    xTaskCreate(joint_read, "joint", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(input_function, "input", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    vTaskStartScheduler();

    // Should not reach here
    while (1);

    return 0;
}