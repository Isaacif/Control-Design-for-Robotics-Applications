#ifndef controller_HPP
#define controller_HPP

#include "algorithms.hpp"

class controller : public IObserver
{
    public:
        std::vector<IObserver *> observers;

        int id;
        float set_point = 0;    

        float sensor_k;
        float pwm_value_k;

        ADC_peripheral a_sensor;
        ADPI_Controller Ji_controller;
        PWM_peripheral u_output;

        controller(int id, ADC_peripheral a_sensor, 
                   ADPI_Controller Ji_controller, PWM_peripheral u_output);
        virtual void loop();
        virtual void Update(float set_point) override;
};  


#endif
