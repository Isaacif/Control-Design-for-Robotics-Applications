/**
 * @file algorithms.hpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief algorithms file for general algorithms.
 * @version 0.1
 * @date 2023-07-02
 * 
 */

#include "algorithms.hpp"

ADPI_Controller::ADPI_Controller(float Kp, float Ki, float setpoint)
{
    c_state_Kp = Kp; c_Ki = Ki, r_k = setpoint;
    setpointchanged = true;
}

void ADPI_Controller::configureSP(float setpoint)
{
    r_k = setpoint;
    setpointchanged = true;
}

float ADPI_Controller::computeADKp()
{   
    uint16_t x = 1/e.back();
    float Kp;
    
    if(setpointchanged)
    {
        max_error = e.back();
        setpointchanged = false;
    }

    if(e.back() > max_error*RAMP_TS_ONE)
    { 
        Kp = std::abs(Kp_min + c_state_Kp*x*max_error*RAMP_TS_ONE);
    }

    else if(e.back() >= RAMP_TS_TWO)
    {
        Kp = std::abs(c_state_Kp+Kp_min);
    }

    else
    {
        Kp = std::abs(Kp_min + (c_state_Kp/x)/(max_error*RAMP_TS_TWO));
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
    p_action = ADKp*e.back();
    i_action = (c_Ki*(SYSTEM_PERIOD*e.back()) + u_i.back());
    control_action = p_action + i_action;
    u_i.push_back(i_action);
    u.push_back(control_action);
    
    return control_action;
}