/**
 * @file algorithms.hpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief algorithms file for general algorithms.
 * @version 0.1
 * @date 2023-07-02
 * 
 */

#include "algorithms.hpp"

ADPI_Controller::ADPI_Controller(float Kp, float Ki, float rCoef, float setpoint)
{
    c_state_Kp = Kp; c_Ki = Ki, ramp_Coef = ramp_Coef;
    r_k = setpoint;
    setpointchanged = true;
}

void ADPI_Controller::configureSP(float setpoint)
{
    r_k = setpoint;
    setpointchanged = true;
}

float ADPI_Controller::computeADKp()
{   
    uint16_t x = 1/e[-1];
    float Kp, ramp_thresold;

    if(setpointchanged)
    {
        max_error = e[-1];
        setpointchanged = false;
    }

    if(x > max_error*RAMP_TS_ONE)
    { 
        Kp = Kp_min + c_state_Kp*x*max_error*RAMP_TS_ONE;
    }

    else if(x >= RAMP_TS_TWO)
    {
        Kp = c_state_Kp+Kp_min;
    }

    else
    {
        Kp = Kp_min + (c_state_Kp/x)/(max_error*RAMP_TS_TWO);
    }   

    return Kp;
}

float ADPI_Controller::computeControlAction(float sensor_k)
{
    error_k = r_k - sensor_k;
    e.push_back(error_k);
    if(e.size() > STACK_SIZE)
    {
        e.erase(e.begin());
    }

    ADKp = computeADKp();
    control_action = ADKp*e[-1] + (c_Ki*(SYSTEM_PERIOD*e[-1]) + u[-1]);
    u.push_back(control_action);
    
    return control_action;
}