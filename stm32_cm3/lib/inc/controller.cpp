#include "controller.hpp"

controller::controller(int id, ADC_peripheral a_sensor, 
                       ADPI_Controller Ji_controller, PWM_peripheral u_output)
{
    this->id = id;
    this->a_sensor = a_sensor;
    this->Ji_controller = Ji_controller;
}

void controller::Update(float set_point)
{

}

void controller::loop()
{
    sensor_k = a_sensor.adc_read(ADC_CHANNEL1);
    pwm_value_k = Ji_controller.computeControlAction(sensor_k);

    u_output.pwmWrite(pwm_value_k, TIM_OC1);
}