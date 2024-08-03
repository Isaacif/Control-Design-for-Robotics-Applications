#define MICROSECONDS_PER_SECOND 1000000UL

#define MICROSECONDS_TO_TICKS(us) \
    ((TickType_t)(((us) * (TickType_t)configTICK_RATE_HZ) / MICROSECONDS_PER_SECOND))

#define SENSOR_K    0.08

#include "PWM_peripheral.hpp"
#include "ADC_peripheral.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include "USART_peripheral.hpp"
#include "DMA_peripheral.hpp"
#include "controller.hpp"
#include "algorithms.hpp"

ADC_peripheral  adc_port_a;
PWM_peripheral  pwm_timer_4;
USART_peripheral serial_interface;
DMA_peripheral dma1_interface;
ADPI_Controller link1;
ADPI_Controller link2;
controller motor_link1;
controller motor_link2;
ISubject system_manager;

int16_t input_function_stage = 0;
uint16_t joint_one = 0;
uint16_t joint_two = 0;
uint16_t joint_one_counter = 0;
uint16_t joint_two_counter = 0;
uint16_t pwm_value;
int8_t voltage;

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) 
{
    // Handle or log the stack overflow as needed (or leave it empty).
}

extern "C" void logger(void *pvParameters) 
{
    (void)pvParameters;

    while (1) 
    {
        serial_interface.usartSend_char("Link 1: ");
        serial_interface.usartSend_char("P action: ");
        serial_interface.usartSend_integer(link1.log_P_control_action());
        serial_interface.usartSend_char("Error: ");
        serial_interface.usartSend_integer(link1.log_error());

        serial_interface.usartSend_char("Link 2: ");
        serial_interface.usartSend_char("P action: ");
        serial_interface.usartSend_integer(link2.log_P_control_action());
        serial_interface.usartSend_char("Error: ");
        serial_interface.usartSend_integer(link2.log_error());

        vTaskDelay(pdMS_TO_TICKS(350));
    }
}

extern "C" void control(void *pvParameters) 
{
    (void)pvParameters;

    while (1) 
    {
        motor_link1.loopDMA(joint_one);
        motor_link2.loopDMA(joint_two);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

extern "C" void joint_read(void *pvParameters) 
{
    (void)pvParameters;

    while (1) 
    {
        joint_one_counter = 0, joint_two_counter = 0;

        for(uint8_t i = 0; i < 10; i++)
        {
            joint_one_counter+=dma1_interface.memory_buffer[0];
            joint_two_counter+=dma1_interface.memory_buffer[1];
            vTaskDelay(MICROSECONDS_TO_TICKS(5));
        }

        joint_one = joint_one_counter/10;
        joint_two = joint_two_counter/10;
        
        joint_one = (joint_one - 960)*0.0878;
        
        joint_two = (joint_two - 1500)*0.089;

        vTaskDelay(MICROSECONDS_TO_TICKS(250));
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
    link1.ADPI_initialize(10, 6, 160);
    link2.ADPI_initialize(10, 6, 90);
    //gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);
    //gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);

    serial_interface.USART_initialization(GPIO2, GPIO3, GPIOA, 
                                  RCC_USART2, USART2, RCC_GPIOA, 115200); 
    motor_link1.controller_initialize(0, GPIOB, GPIO8, &adc_port_a, &link1, 
                &pwm_timer_4, RCC_GPIOB);
                
    motor_link2.controller_initialize(1, GPIOB, GPIO9, &adc_port_a, &link2, 
                &pwm_timer_4, RCC_GPIOB);
    
    motor_link1.time_period = 0.002;
    motor_link2.time_period = 0.002;
    system_manager.Attach(&motor_link1);
    system_manager.Attach(&motor_link2);

    xTaskCreate(joint_read, "joint", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(control, "input", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(logger, "log", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    // Should not reach here
    while (1);

    return 0;
}