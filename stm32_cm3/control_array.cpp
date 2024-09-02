/*
#include "controller.hpp"

controller::controller(uint8_t id_1, uint8_t id_2,uint32_t GPORT_INB1s, uint32_t GPIN_INB1s, uint32_t GPORT_INB2s, uint32_t GPIN_INB2s, 
                   ADC_peripheral *a_sensors, servoIn_Controller *Ji_controllers, 
                   PWM_peripheral *u_outputs, rcc_periph_clken RCC_GPIOPs)
{
    this->id1 = id_1, this->id2 = id_2;
    GPIO_PORT_INB1 = GPORT_INB1s, GPIO_PIN_INB1 = GPIN_INB1s;
    GPIO_PORT_INB2 = GPORT_INB2s, GPIO_PIN_INB2 = GPIN_INB2s;
    RCC_GPIO_INB = RCC_GPIOPs;
    a_sensor = a_sensors;
    Ji_controller = Ji_controllers;
    u_output = u_outputs;  

    //rcc_periph_clock_enable(RCC_GPIO_INB);
    gpio_mode_setup(GPIO_PORT_INB1, GPIO_MODE_OUTPUT,
                  GPIO_PUPD_PULLUP, GPIO_PIN_INB1);
    gpio_set_output_options(GPIO_PORT_INB1, GPIO_OTYPE_PP,
                    GPIO_OSPEED_25MHZ,  GPIO_PIN_INB1);
    gpio_clear(GPIO_PORT_INB1, GPIO_PIN_INB1);

    gpio_mode_setup(GPIO_PORT_INB2, GPIO_MODE_OUTPUT,
                  GPIO_PUPD_PULLUP, GPIO_PIN_INB2);
    gpio_set_output_options(GPIO_PORT_INB2, GPIO_OTYPE_PP,
                    GPIO_OSPEED_25MHZ,  GPIO_PIN_INB1);
    gpio_clear(GPIO_PORT_INB2, GPIO_PIN_INB2);

}

void controller::controller_initialize(uint8_t id_1, uint8_t id_2, uint32_t GPORT_INB1s, uint32_t GPIN_INB1s, uint32_t GPORT_INB2s, uint32_t GPIN_INB2s, 
                   ADC_peripheral *a_sensors, servoIn_Controller *Ji_controllers, 
                   PWM_peripheral *u_outputs, rcc_periph_clken RCC_GPIOPs)
{
    this->id1 = id_1, this->id2 = id_2;
    GPIO_PORT_INB1 = GPORT_INB1s, GPIO_PIN_INB1 = GPIN_INB1s;
    GPIO_PORT_INB2 = GPORT_INB2s, GPIO_PIN_INB2 = GPIN_INB2s;
    RCC_GPIO_INB = RCC_GPIOPs;
    a_sensor = a_sensors;
    Ji_controller = Ji_controllers;
    u_output = u_outputs;  

    //rcc_periph_clock_enable(RCC_GPIO_INB);
    gpio_mode_setup(GPIO_PORT_INB1, GPIO_MODE_OUTPUT,
                  GPIO_PUPD_PULLUP, GPIO_PIN_INB1);
    gpio_set_output_options(GPIO_PORT_INB1, GPIO_OTYPE_PP,
                    GPIO_OSPEED_25MHZ,  GPIO_PIN_INB1);
    gpio_clear(GPIO_PORT_INB1, GPIO_PIN_INB1);

    gpio_mode_setup(GPIO_PORT_INB2, GPIO_MODE_OUTPUT,
                  GPIO_PUPD_PULLUP, GPIO_PIN_INB2);
    gpio_set_output_options(GPIO_PORT_INB2, GPIO_OTYPE_PP,
                    GPIO_OSPEED_25MHZ,  GPIO_PIN_INB1);
    gpio_clear(GPIO_PORT_INB2, GPIO_PIN_INB2);
}

void controller::Update(int8_t setpoint, int16_t sensor1_k, int16_t sensor2_k, int8_t ID, int8_t K_iter)
{
    if(ID == 1)
    {
        this->set_point_target1 = setpoint;
    }
    if(ID == 2)
    {
        this->set_point_target2 = setpoint;
    }
    this->K_iteration = K_iter;
    this->trajectoryPlanner(ID, sensor1_k, sensor2_k);
    //Ji_controller->configureSP(setpoint, ID);
}

bool controller::iteractPath(int8_t ID, int16_t sensor_kIter)
{
    if(ID == 1)
    {
        if(this->hasReached(ID, this->currentTargetOne, sensor_kIter))
        {
            if(this->currentStep_one == this->K_iteration || this->currentStep_one == this->K_iteration - 1)
            {
                return true;
            }
            this->currentStep_one+=1;
            currentTargetOne = this->trajectory_one[this->currentStep_one];
            Ji_controller->configureSP(currentTargetOne, ID);
        }
        return false;
    }
    if(ID == 2)
    {
        if(this->hasReached(ID, this->currentTargetTwo, sensor_kIter))
        {
            if(this->hasReached(ID, this->set_point_target2, sensor_kIter) || this->currentStep_two == this->K_iteration - 1)
            {
                return true;
            }
            this->currentStep_two+=1;
            currentTargetTwo = this->trajectory_two[this->currentStep_two];
            Ji_controller->configureSP(currentTargetTwo, ID);
        }
        return false;
    }
    return false;
}
/*
void controller::clearQueue(int8_t ID)
{
    if(ID == 1)
    {
        while(!this->trajectory_one.empty())
        {
            this->trajectory_one.pop();
        }
    }
    if(ID == 2)
    {
        while(!this->trajectory_two.empty())
        {
            this->trajectory_two.pop();
        }
    }
}
*/
void controller::trajectoryPlanner(int8_t ID, float sensor1_k, float sensor2_k)
{   
    float reach_target, target, step;
    if(ID == 1)
    {
        step = (this->set_point_target1 - sensor1_k)/this->K_iteration;
        reach_target = sensor1_k;
        target = this->set_point_target1;
        for(int interation = 0; interation < this->K_iteration; interation++)
        {
            reach_target+=step;
            this->trajectory_one[interation] = reach_target;
        }
        currentTargetOne = this->trajectory_one[0];
        Ji_controller->configureSP(currentTargetOne, ID);
        this->currentStep_one = 0;
    }
    if(ID == 2)
    {
        step = (this->set_point_target2 - sensor2_k)/this->K_iteration;
        reach_target = sensor2_k;
        target = this->set_point_target2;
        for(int interation = 0; interation < this->K_iteration; interation++)
        {
            reach_target+=step;
            this->trajectory_two[interation] = reach_target;
        }
        currentTargetTwo = this->trajectory_two[0];
        Ji_controller->configureSP(currentTargetTwo, ID);
        this->currentStep_two = 0;
    }
}

void controller::loop()
{
    sensor_k = a_sensor->adc_read(JOINT_ANGLE_OBSERVER[id]);
    pwm_value_k1 = round(Ji_controller->computeControlAction(sensor_k, 0, this->time_period));
    pwm_value_k1 = pwm_value_k1*0.0015;
    if(pwm_value_k1 > 100)
    {
        pwm_value_k1 = 25;
    }
    this->pwm_mapping(pwm_value_k1, 0);
}

bool controller::hasReached(int8_t ID, int8_t target, int16_t sensor_kIter)
{
    if(ID == 1)
    {
        return std::abs(target - sensor_kIter) < 4;
    }
    if(ID == 2)
    {
        return std::abs(target - sensor_kIter) < 4;
    }
}

void controller::loopDMA(int16_t reading1, int16_t reading2)
{
    
    Ji_controller->computeControlAction(reading1, reading2, this->time_period);
    pwm_value_k1 = Ji_controller->u1_k;
    pwm_value_k2 = Ji_controller->u2_k;

    this->pwm_mapping(pwm_value_k1, 1);
    this->pwm_mapping(pwm_value_k2, 2);
}

void controller::pwm_mapping(int32_t pwm_value, uint8_t joint_id)
{   
    int this_instance_id;
    uint32_t GPIO_PORT, GPIO_PIN;
    if(joint_id == 1)
    {   
        this_instance_id = 0;
        GPIO_PORT = GPIO_PORT_INB1;
        GPIO_PIN = GPIO_PIN_INB1;
    }
    
    if(joint_id == 2)
    {
        this_instance_id = 1;
        GPIO_PORT = GPIO_PORT_INB2;
        GPIO_PIN = GPIO_PIN_INB2;
    }

    if(pwm_value > 0)
    {
        u_output->pwmWrite(pwm_value, JOINT_PWM_INPUT[this_instance_id]);
        gpio_clear(GPIO_PORT, GPIO_PIN);
    }
    else if(pwm_value < 0)
    {
        //negative_duty_cycle = CONTROL_ACTION_THRESHOLD + pwm_value;
        negative_duty_cycle = 100 + pwm_value;
        u_output->pwmWrite(negative_duty_cycle, JOINT_PWM_INPUT[this_instance_id]);
        gpio_set(GPIO_PORT, GPIO_PIN);
    }
    else 
    {
        u_output->pwmWrite(pwm_value, JOINT_PWM_INPUT[this_instance_id]);
        gpio_clear(GPIO_PORT, GPIO_PIN);
    }
}

void controller::attach_parameters(loop_parameters_t c_parameters)
{
    this->control_parameters = c_parameters;
}

*/