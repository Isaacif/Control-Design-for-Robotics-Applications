#define StateGain   0.2
#define SetPoint    430
#define dataSize    10000

#include "PWM_peripheral.hpp"
#include "ADC_peripheral.hpp"
#include "SYS_TIMER_peripheral.hpp"
#include "USART_peripheral.hpp"
#include "DMA_peripheral.hpp"

/* Timer 4 and Timer 3 are reserved for PWM applications
   Timer 2 and other timers can be used for general tasks
   Sys_Tick timer is being used for general counter control
*/

#define LED_PIN GPIO13
#define LED_PORT GPIOC

#define system_frequency_1Mhz    1e6

SYS_TIMER_peripheral system_counter;
ADC_peripheral  adc_port_a;
PWM_peripheral  pwm_timer_4;
USART_peripheral serial_interface;
DMA_peripheral dma2_interface;

int16_t input_function_stage = 0;
uint16_t joint_one = 0;
uint16_t joint_two = 0;
uint16_t joint_one_h = 0;
uint16_t joint_two_h = 0;
uint16_t joint_one_s = 0;
uint16_t joint_two_s = 0;

uint16_t pwm_value1;
uint16_t pwm_value2;
int8_t voltage1;
int8_t voltage2;
uint16_t reading;
int16_t on = 0;
uint32_t samplesnumb = 0;
uint32_t time_reference = 0;

int8_t change_state = 1;
int8_t ended = 0;
int8_t side = 1;
int8_t stop = 0;
int8_t voltages1[dataSize];
int8_t voltages2[dataSize];
uint16_t joint_one_data[dataSize];
uint16_t joint_two_data[dataSize];

void sys_tick_handler(void)
{
    g_counter_millis++;
}

void input_function() 
{
    if(input_function_stage <= 10)
    {
        voltage1 = 3;
        pwm_value1 = 25;
        voltage2 = -10;
        pwm_value2 = 100 - 83;
        pwm_timer_4.pwmWrite(pwm_value1, TIM_OC1);
        gpio_clear(GPIOB, GPIO8);
        pwm_timer_4.pwmWrite(pwm_value2, TIM_OC2);
        gpio_set(GPIOB, GPIO9);
    }
    else if(input_function_stage <= 20)
    {
        voltage1 = -3;
        pwm_value1 = 100 - 25;
        voltage2 = 10;
        pwm_value2 = 83;
        pwm_timer_4.pwmWrite(pwm_value1, TIM_OC1);
        gpio_set(GPIOB, GPIO8);
        pwm_timer_4.pwmWrite(pwm_value2, TIM_OC2);
        gpio_clear(GPIOB, GPIO9);
    }

    if(input_function_stage >= 20)
    {
        input_function_stage = 0;
    }

    input_function_stage = input_function_stage + 1;
}

void side_one()
{
    if(samplesnumb < 5000)
    {
        voltage1 = 2;
        pwm_value1 = 16.7;
        voltage2 = -4;
        pwm_value2 = 100 - 33;
        pwm_timer_4.pwmWrite(pwm_value1, TIM_OC1);
        gpio_clear(GPIOB, GPIO8);
        pwm_timer_4.pwmWrite(pwm_value2, TIM_OC2);
        gpio_set(GPIOB, GPIO9);
    }
    else if(samplesnumb > 5000)
    {
        voltage1 = 3;
        pwm_value1 = 27.5;
        voltage2 = -3;
        pwm_value2 = 100 - 25;
        pwm_timer_4.pwmWrite(pwm_value1, TIM_OC1);
        gpio_clear(GPIOB, GPIO8);
        pwm_timer_4.pwmWrite(pwm_value2, TIM_OC2);
        gpio_set(GPIOB, GPIO9);
    }
}

void side_two()
{
    if(samplesnumb < 5000)
    {
        voltage1 = -3;
        pwm_value1 = 100 - 27.5;
        voltage2 = 4;
        pwm_value2 = 33;
        pwm_timer_4.pwmWrite(pwm_value1, TIM_OC1);
        gpio_set(GPIOB, GPIO8);
        pwm_timer_4.pwmWrite(pwm_value2, TIM_OC2);
        gpio_clear(GPIOB, GPIO9);
    }
    else if(samplesnumb > 5000 && change_state)
    {
        voltage1 = -7;
        pwm_value1 = 100 - 58;
        voltage2 = 3;
        pwm_value2 = 25;
        pwm_timer_4.pwmWrite(pwm_value1, TIM_OC1);
        gpio_set(GPIOB, GPIO8);
        pwm_timer_4.pwmWrite(pwm_value2, TIM_OC2);
        gpio_clear(GPIOB, GPIO9);
        change_state = 0;
    }
    else
    {
        voltage1 = -2;
        pwm_value1 = 100 - 16.7;
        voltage2 = 4;
        pwm_value2 = 33;
        pwm_timer_4.pwmWrite(pwm_value1, TIM_OC1);
        gpio_set(GPIOB, GPIO8);
        pwm_timer_4.pwmWrite(pwm_value2, TIM_OC2);
        gpio_clear(GPIOB, GPIO9);
    }
}

void joint_read() 
{
    joint_one_h = 0;
    joint_two_h = 0;
    joint_one_s = 0;
    joint_two_s = 0;

    for(uint8_t k = 0; k < 10; k++)
    {
        joint_one_h=dma2_interface.memory_buffer[0];
        joint_two_h=dma2_interface.memory_buffer[1];   
        joint_one_h-=800;
        joint_two_h-=665;
        if(joint_one_h > 6000)
        {
            joint_one_h = 0;
        }
        if(joint_two_h > 6000)
        {
            joint_two_h = 0;
        }
        joint_one_s+=joint_one_h;
        joint_two_s+=joint_two_h;
    }   

    joint_one = joint_one_s*0.0073;
    joint_two = joint_two_s*0.0094;
    
    voltages1[samplesnumb] = voltage1;
    voltages2[samplesnumb] = voltage2;
    joint_one_data[samplesnumb] = joint_one;
    joint_two_data[samplesnumb] = joint_two;
    samplesnumb++;

    if(samplesnumb >= dataSize)
    {
        ended = 1;
    }

}

void send_data() 
{
    for(int j = 0; j < dataSize; j++)
    {
        serial_interface.usartSend_char("1: ");
        serial_interface.usartSend_integer(joint_one_data[j]);
        serial_interface.usartSend_char("2: ");
        serial_interface.usartSend_integer(joint_two_data[j]);
        serial_interface.usartSend_char("v1: ");
        serial_interface.usartSend_integer(voltages1[j]);
        serial_interface.usartSend_char("v2: ");
        serial_interface.usartSend_integer(voltages2[j]);
    }
}

int main(void)
{
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    system_counter.SYS_TIMER_initialize(system_frequency_1Mhz);
    dma2_interface.DMA_initialize(DMA2, DMA_STREAM0, ADC1_DR, RCC_DMA2);

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

    while(true)
    {

        if(ended == 0)
        {   
            if(g_counter_millis - time_reference > SYSTEM_TICK_MS(5) && samplesnumb < dataSize)
            {
                joint_read();
                time_reference = g_counter_millis;
            }
            
            if(g_counter_millis > SYSTEM_TICK_MS(500))
            {
                if(side == 1)
                {
                    side_one();
                    side = -1;
                }
                else if(side == -1)
                {
                    side_two();
                    side = 1;
                }
                g_counter_millis = 0;
                time_reference = 0;
            }
        }

        if(ended && !stop)
        {
            pwm_timer_4.pwmWrite(0, TIM_OC1);
            gpio_clear(GPIOB, GPIO8);
            pwm_timer_4.pwmWrite(0, TIM_OC2);
            gpio_clear(GPIOB, GPIO9);
            send_data();
            stop = 1;
        }
    }
}