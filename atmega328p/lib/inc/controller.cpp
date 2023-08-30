#include "controller.hpp"

controller::controller(uint8_t id, uint32_t gpio_pwm, uint32_t gpio_pin, ADPI_Controller *jcontroller, float time)
{
    Ji_controller = jcontroller;
    GPIO_P1 = gpio_pwm, GPIO_P2 = gpio_pin;
    time_period = time;

    pinMode(GPIO_P1, OUTPUT);
    pinMode(GPIO_P2, OUTPUT);
}

void controller::Update(float spoint)
{
    this->set_point = spoint;
    Ji_controller->configureSP(spoint);
}

void controller::loop()
{
    sensor_k = analogRead(JOINT_ANGLE_OBSERVER[id]);
    pwm_value_k = round(Ji_controller->computeControlAction(sensor_k, this->time_period));
    this->pwm_mapping(pwm_value_k);
}
void controller::pwm_mapping(float pwm_value)
{   
    if(pwm_value > 0)
    {
        analogWrite(GPIO_P1, pwm_value);
        digitalWrite(GPIO_P2, LOW);
    }
    else if(pwm_value < 0)
    {
        negative_duty_cycle = CONTROL_ACTION_THRESHOLD - pwm_value;
        analogWrite(GPIO_P1, negative_duty_cycle);
        digitalWrite(GPIO_P2, HIGH);
    }
    else 
    {
        analogWrite(GPIO_P1, pwm_value);
    }
}

void controller::attach_parameters(loop_parameters_t c_parameters)
{
    this->control_parameters = c_parameters;
}