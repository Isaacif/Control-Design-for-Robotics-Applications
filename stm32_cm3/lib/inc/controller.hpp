#ifndef controller_HPP
#define controller_HPP

#define CONTROL_ACTION_THRESHOLD          65535      
#define NUMBER_OF_JOINTS                  4

#include "algorithms.hpp"

static const tim_oc_id JOINT_PWM_INPUT[NUMBER_OF_JOINTS] = 
{
    TIM_OC1, TIM_OC2, TIM_OC3, TIM_OC4
};

static const uint8_t JOINT_ANGLE_OBSERVER[NUMBER_OF_JOINTS] = 
{
    ADC_CHANNEL0, ADC_CHANNEL1, ADC_CHANNEL5, ADC_CHANNEL7
};

static const uint8_t JOINT_CURRENT_OBSERVER[NUMBER_OF_JOINTS] = 
{
    ADC_CHANNEL2, ADC_CHANNEL4, ADC_CHANNEL6, ADC_CHANNEL8
};

typedef struct loop_parameters
{
    uint32_t timer_id_period;
    uint16_t set_point_id;

} loop_parameters_t;

class controller : public IObserver
{
    public:
        std::vector<IObserver *> observers;

        int id1;
        int id2;
        int8_t set_point1 = 0;   
        int8_t set_point2 = 0;    

        uint16_t sensor_k;
        int32_t pwm_value_k1;
        int32_t pwm_value_k2;


        uint16_t time_period;
        uint32_t GPIO_PORT_INB1;
        uint32_t GPIO_PIN_INB1;
        uint32_t GPIO_PORT_INB2;
        uint32_t GPIO_PIN_INB2;
        rcc_periph_clken RCC_GPIO_INB;

        ADC_peripheral *a_sensor;
        servoIn_Controller *Ji_controller;
        PWM_peripheral *u_output;
        loop_parameters_t control_parameters;

        controller(){}
        controller(uint8_t id_1, uint8_t id_2,uint32_t GPORT_INB1, uint32_t GPIN_INB1, uint32_t GPORT_INB2, uint32_t GPIN_INB2, 
                   ADC_peripheral *a_sensor, servoIn_Controller *Ji_controller, 
                   PWM_peripheral *u_output, rcc_periph_clken RCC_GPIOP);

        void controller_initialize(uint8_t id_1, uint8_t id_2,uint32_t GPORT_INB1, uint32_t GPIN_INB1, uint32_t GPORT_INB2, uint32_t GPIN_INB2, 
                   ADC_peripheral *a_sensor, servoIn_Controller *Ji_controller, 
                   PWM_peripheral *u_output, rcc_periph_clken RCC_GPIOP);

        void attach_parameters(loop_parameters_t c_parameters);
        virtual void loop();
        virtual void loopDMA(int16_t reading1, int16_t reading2);
        virtual void Update(int8_t setpoint, int8_t ID) override;
        void pwm_mapping(int32_t pwm_value, uint8_t joint_id);

    private:
        int32_t negative_duty_cycle;
};  

#endif
