#include "controller.hpp"

controller::controller(uint8_t id, uint32_t GPORT_INB, uint32_t GPIN_INB, 
                       ADC_peripheral *sensor, ADPI_Controller *jcontroller, 
                       PWM_peripheral *output, rcc_periph_clken RCC_GPIOP)
{
    this->id = id, GPIO_PORT_INB = GPORT_INB, GPIO_PIN_INB = GPIN_INB;
    RCC_GPIO_INB = RCC_GPIOP;
    a_sensor = sensor;
    Ji_controller = jcontroller;
    u_output = output;  

    rcc_periph_clock_enable(RCC_GPIO_INB);
    gpio_set_mode(GPIO_PORT_INB, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO_PIN_INB);
    gpio_clear(GPIO_PORT_INB, GPIO_PIN_INB);
}

void controller::Update(float set_point)
{
    this->set_point = set_point;
}

void controller::loop()
{
    sensor_k = a_sensor->adc_read(JOINT_ANGLE_OBSERVER[id]);
    pwm_value_k = Ji_controller->computeControlAction(sensor_k, this->time_period);
    u_output->pwmWrite(pwm_value_k, JOINT_PWM_INPUT[id]);
}

float controller::pwm_mapping(float pwm_value)
{   
    if(pwm_value > 0)
    {
        u_output->pwmWrite(pwm_value_k, JOINT_PWM_INPUT[id]);
        gpio_clear(GPIO_PORT_INB, GPIO_PIN_INB);
    }
    else if(pwm_value < 0)
    {
        negative_duty_cycle = CONTROL_ACTION_THRESHOLD - pwm_value;
        u_output->pwmWrite(negative_duty_cycle, JOINT_PWM_INPUT[id]);
        gpio_set(GPIO_PORT_INB, GPIO_PIN_INB);
    }
    else 
    {
        u_output->pwmWrite(pwm_value, JOINT_PWM_INPUT[id]);
    }
}

void controller::attach_parameters(loop_parameters_t c_parameters)
{
    this->control_parameters = c_parameters;
}