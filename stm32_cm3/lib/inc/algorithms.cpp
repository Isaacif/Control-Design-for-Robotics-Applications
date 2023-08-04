/**
 * @file algorithms.hpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief algorithms file for general algorithms.
 * @version 0.1
 * @date 2023-07-02
 * 
 */

#include "algorithms.hpp"

P_Controller::P_Controller(float Kp, float setpoint)
{
    c_state_Kp = Kp, r_k = setpoint;
    setpointchanged = true;
}

void P_Controller::configureSP(float setpoint)
{
    r_k = setpoint;
    setpointchanged = true;
}

float P_Controller::computeControlAction(float sensor_k, float time_period)
{
    error_k = r_k - sensor_k;  
    control_action = c_state_Kp*error_k;
    
    return control_action;
}

ADPI_Controller::ADPI_Controller(float Kp, float Ki, float setpoint)
{
    c_state_Kp = Kp; c_Ki = Ki, r_k = setpoint;
    setpointchanged = true;

    circularBufferPush(&u, 0);
    circularBufferPush(&u_i, 0);
}

void ADPI_Controller::configureSP(float setpoint)
{
    r_k = setpoint;
    setpointchanged = true;
}

float ADPI_Controller::computeADKp()
{   
    float x = 1/circularBufferGetRelativeElement(&e, -1);
    float Kp;
    
    if(setpointchanged)
    {
        max_error = 1/circularBufferGetRelativeElement(&e, -1);
        setpointchanged = false;
    }

    if(circularBufferGetRelativeElement(&e, -1) > max_error*RAMP_TS_ONE)
    { 
        Kp = std::abs(Kp_min + c_state_Kp*x*max_error*RAMP_TS_ONE);
    }

    else if(circularBufferGetRelativeElement(&e, -1) >= RAMP_TS_TWO)
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
    circularBufferPush(&e, error_k);

    if (error_k < 0)
    {
        p_action = (c_state_Kp)*error_k;
        i_action = (c_Ki*(SYSTEM_PERIOD*time_period*error_k) + circularBufferGetRelativeElement(&u_i, -1));
        control_action = p_action;
        circularBufferPush(&u_i, i_action);
        circularBufferPush(&u, control_action);
        return control_action;
    }
    

    ADKp = computeADKp();
    p_action = ADKp*error_k;
    i_action = (c_Ki*(SYSTEM_PERIOD*time_period*error_k) + circularBufferGetRelativeElement(&u_i, -1));
    control_action = p_action;
    circularBufferPush(&u_i, i_action);
    circularBufferPush(&u, control_action);
    
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

void ISubject::setState(uint8_t sJoint, float spoint)
{
    this->set_point = spoint;
    this->Joint = sJoint;
}