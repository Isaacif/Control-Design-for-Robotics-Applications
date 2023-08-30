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

ADPI_Controller::ADPI_Controller(float Kp, float Ki, int16_t setpoint)
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
    circularBufferClean(&e);
    circularBufferClean(&u);
    circularBufferClean(&u_i);
}

int16_t ADPI_Controller::log_setpoint()
{
    return this->r_k;
}

float ADPI_Controller::log_P_control_action()
{
    return circularBufferGetRelativeElement(&u, -1);
}

float ADPI_Controller::log_I_control_action()
{
    return circularBufferGetRelativeElement(&u_i, -1);
}

int16_t ADPI_Controller::log_error()
{
    return circularBufferGetRelativeElement(&e, -1);
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
        Kp = abs(Kp_min + c_state_Kp*x*max_error*RAMP_TS_ONE);
    }

    else if(circularBufferGetRelativeElement(&e, -1) >= RAMP_TS_TWO)
    {
        Kp = abs(c_state_Kp+Kp_min);
    }

    else
    {
        Kp = abs(Kp_min + (c_state_Kp/x)/(max_error*RAMP_TS_TWO));
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
        i_action = c_Ki*(time_period*error_k) + circularBufferGetRelativeElement(&u_i, -1);
        i_action = this->threshold(i_action, error_k);

        control_action = p_action+i_action;
        circularBufferPush(&u_i, i_action);
        circularBufferPush(&u, p_action);
        return control_action;
    }
    

    ADKp = computeADKp();
    p_action = ADKp*error_k;
    i_action = c_Ki*(time_period*error_k) + circularBufferGetRelativeElement(&u_i, -1);
    i_action = this->threshold(i_action, error_k);
    control_action = p_action+i_action;
    circularBufferPush(&u_i, i_action);
    circularBufferPush(&u, p_action);
    
    return control_action;
}

float ADPI_Controller::threshold(float output_value, float limit_error)
{
    if(output_value > 100)
    {
        output_value = 100;
    }
    else if(output_value < -100)
    {
        output_value = -100;
    }

    if(limit_error < 15)
    {
        output_value = 0;
    }
    
    return output_value;
}