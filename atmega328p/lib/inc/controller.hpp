#ifndef controller_HPP
#define controller_HPP

#define CONTROL_ACTION_THRESHOLD          65535      
#define NUMBER_OF_JOINTS                  5

#include "algorithms.hpp"

static const uint8_t JOINT_ANGLE_OBSERVER[NUMBER_OF_JOINTS] = 
{
    A1, A2, A3, A4, A5
};

typedef struct loop_parameters
{
    uint32_t timer_id_period;
    float set_point_id;

} loop_parameters_t;

class controller
{
    public:

        int id;
        uint16_t set_point = 0;    
        uint32_t GPIO_P1, GPIO_P2;

        uint16_t sensor_k;
        int16_t pwm_value_k;
        float time_period;

        loop_parameters_t control_parameters;
        ADPI_Controller *Ji_controller;

        controller(uint8_t id, uint32_t gpio_pwm, uint32_t gpio_pin, ADPI_Controller *jcontroller, float time);

        void attach_parameters(loop_parameters_t c_parameters);
        virtual void loop();
        virtual void Update(float spoint);
        void pwm_mapping(float pwm_value);

    private:
        int32_t negative_duty_cycle;
};  


#endif
