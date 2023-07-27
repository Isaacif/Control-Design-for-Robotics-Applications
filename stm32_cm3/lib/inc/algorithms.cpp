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
    float x = 1/e.back();
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

float ADPI_Controller::computeControlAction(float sensor_k, float time_period)
{
    error_k = r_k - sensor_k;
    if (error_k < 0)
    {
        p_action = (c_state_Kp+Kp_min)*e.back();
        i_action = (c_Ki*(SYSTEM_COUNT_100uS*time_period*e.back()) + u_i.back());
        control_action = p_action + i_action;
        u_i.push_back(i_action);
        u.push_back(control_action);
        return control_action;
    }
    
    e.push_back(error_k);
    if(e.size() > STACK_SIZE)
    {
        e.erase(e.begin());
    }

    ADKp = computeADKp();
    p_action = ADKp*e.back();
    i_action = (c_Ki*(SYSTEM_COUNT_100uS*time_period*e.back()) + u_i.back());
    control_action = p_action + i_action;
    u_i.push_back(i_action);
    u.push_back(control_action);
    
    return control_action;
}

void ISubject::Attach(IObserver *observer)
{
    observers.push_back(observer);
}

void ISubject::Detach(IObserver *observer)
{
    auto iterator = std::find(observers.begin(), observers.end(), observer);

    if (iterator != observers.end()) 
    { 
        observers.erase(iterator); 
    }
}

void ISubject::Notify()
{
    observers[Joint]->Update(set_point); 
}

void ISubject::setState(uint8_t Joint, float set_point)
{
    this->set_point = set_point;
    this->Joint = Joint;
}