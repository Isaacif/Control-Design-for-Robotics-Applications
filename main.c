#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define GPIO_PWM0A_OUT 15   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 16   //Set GPIO 16 as PWM0B
#define Ganho_Proporcional  0.25

#define GPIO_INTERNAL_PIN 4

#define Period 20

float voltage;
int step;
int start = false;
int ia;

static const char *TAG = "ADC: ";

float angle;
float voltage;
float erro;
float time = 0;
int cicle_positive = true;

float setpoint = 250;
static esp_adc_cal_characteristics_t adc1_chars;

static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

static void generateOutput(void *arg)
{
    while(true)
    {
        if(cicle_positive)
        {
            for(int i= 0; i < 100; i++)
            {
                step = i;
                if(i < 10)
                {
                    voltage = 0;
                    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, voltage);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 20)
                {
                    voltage = 12*(2.5*i)/100;
                    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 2.5*i);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 40)
                {   
                    voltage = 6;
                    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 50)
                {
                    ia = i - 40;
                    voltage = 6 - 12*(2.5*ia)/100;
                    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50-2.5*ia);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 60)
                {
                    voltage = 0;
                    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 0);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 70)
                {
                    ia = i - 50;
                    voltage = -12*(2.5*(i-50))/100;
                    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 2.5*ia);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 90)
                {
                    voltage = -6;
                    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50);
                    vTaskDelay(pdMS_TO_TICKS(Period));   
                }
                else if(i < 100)
                {
                    ia = i - 90;
                    voltage = -(6 - 12*(2.5*ia)/100);
                    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50-2.5*ia);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
            }            
        }
    }
}

static void readSensor(void *arg)
{   
    while(true)
    {
        angle = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_6), &adc1_chars);
        angle = angle/9.15;
        printf("Voltage Value: %f ", voltage);
        printf("; Angle Value: %f ", angle);
        printf("; Time Value Ms: %f", time);
        printf("; Step: %d \n", step);
        
        time+=5;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void app_main(void)
{

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_INTERNAL_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Enable the internal pull-up resistor
    gpio_config(&io_conf);


    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); 
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11));

    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 0);
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("starting \n");
    while(true)
    {
        int internal_gpio_value = gpio_get_level(GPIO_INTERNAL_PIN);
        vTaskDelay(pdMS_TO_TICKS(100));
        if(internal_gpio_value)
        {
            start = true;
            break;
        }
    }
    
    if(start)
    {
      //xTaskCreate(generateOutput, "Gera_Entrada", 4096, NULL, 3, NULL);
      xTaskCreate(readSensor, "Ler_Saida", 4096, NULL, tskIDLE_PRIORITY, NULL);
    }
}