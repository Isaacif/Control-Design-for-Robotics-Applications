/**
 * @file algorithms.hpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief Head file for general algorithms.
 * @version 0.1
 * @date 2023-07-02
 * 
 */

#define SYSTEM_PERIOD       0.0001
#define RAMP_TS_ONE         0.2
#define RAMP_TS_TWO         0.8 
#define STACK_SIZE          100

#include <vector>
#include <cstdlib>
#include <stdint.h>   
#include <algorithm>
#include <string>

#include "ADC_peripheral.hpp"
#include "PWM_peripheral.hpp"


#ifndef algorithms_HPP
#define algorithms_HPP

class ADPI_Controller
{
    public:
        float c_state_Kp;
        float c_Ki;
        float control_action;

        bool setpointchanged;

        std::vector<float> u = {0};
        std::vector<float> u_i = {0};
        std::vector<float> e = {};

        ADPI_Controller(float Kp, float Ki, float setpoint);

        void configureSP(float setpoint);
        float computeADKp();
        float computeControlAction(float error);

    private:
        float r_k;
        float error_k;
        float max_error;
        float ADKp;

        float p_action;
        float i_action;

        float Kp_min = 0.5;
};  

class IObserver 
{
    public:
        int id;

        virtual ~IObserver(){};
        virtual void Update(float set_point);
};

class ISubject 
{
    public:
        std::vector<IObserver *> observers;

        uint8_t Joint;
        float set_point;   

        virtual ~ISubject(){};
        virtual void Attach(IObserver *observer) = 0;
        virtual void Detach(IObserver *observer) = 0;
        virtual void Notify() = 0;
        void setState(uint8_t Joint, float set_point);
};

#endif
