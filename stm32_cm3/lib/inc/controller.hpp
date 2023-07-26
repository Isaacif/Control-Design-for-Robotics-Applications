#ifndef controller_HPP
#define controller_HPP

#define CONTROL_ACTION_THRESHOLD          65535      
#define NUMBER_OF_JOINTS                  5

#include "algorithms.hpp"

static const tim_oc_id JOINT_PWM_INPUT[NUMBER_OF_JOINTS] = 
{
    TIM_OC1, TIM_OC2, TIM_OC3, TIM_OC4, TIM_OC1
};

static const uint8_t JOINT_ANGLE_OBSERVER[NUMBER_OF_JOINTS] = 
{
    ADC_CHANNEL1, ADC_CHANNEL3, ADC_CHANNEL5, ADC_CHANNEL7, ADC_CHANNEL9
};

static const uint8_t JOINT_CURRENT_OBSERVER[NUMBER_OF_JOINTS] = 
{
    ADC_CHANNEL2, ADC_CHANNEL4, ADC_CHANNEL6, ADC_CHANNEL8, ADC_CHANNEL10
};

typedef struct loop_parameters
{
    uint32_t timer_id_period;
    float set_point_id;

} loop_parameters_t;

class controller : public IObserver
{
    public:
        std::vector<IObserver *> observers;

        int id;
        float set_point = 0;    

        float sensor_k;
        float pwm_value_k;

        uint32_t GPIO_PORT_INB;
        uint32_t GPIO_PIN_INB;
        rcc_periph_clken RCC_GPIO_INB;

        ADC_peripheral *a_sensor;
        ADPI_Controller *Ji_controller;
        PWM_peripheral *u_output;
        loop_parameters_t control_parameters;

        controller(uint8_t id, uint32_t GPORT_INB, uint32_t GPIN_INB, 
                   ADC_peripheral *a_sensor, ADPI_Controller *Ji_controller, 
                   PWM_peripheral *u_output, rcc_periph_clken RCC_GPIOP);

        void attach_parameters(loop_parameters_t c_parameters);
        virtual void loop();
        virtual void Update(float set_point) override;
        float pwm_mapping(float pwm_value);

    private:
        int32_t negative_duty_cycle;
};  


#endif
