/**
 * @file algorithms.hpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief algorithms file for general algorithms.
 * @version 0.1
 * @date 2023-07-02
 * 
 */

#include "algorithms.hpp"

P_Controller::P_Controller(uint16_t Kp, uint16_t setpoint)
{
    c_state_Kp = Kp, r_k = setpoint;
    setpointchanged = true;
}

void P_Controller::configureSP(uint16_t setpoint)
{
    r_k = setpoint;
    setpointchanged = true;
}

uint16_t P_Controller::computeControlAction(uint16_t sensor_k, uint16_t time_period)
{
    error_k = r_k - sensor_k;  
    control_action = c_state_Kp*error_k;
    
    return control_action;
}

ADPI_Controller::ADPI_Controller(uint16_t Kp, uint16_t Ki, uint16_t setpoint)
{
    c_state_Kp = Kp; c_Ki = Ki, r_k = setpoint;
    setpointchanged = true;

    circularBufferPush(&u, 0);
    circularBufferPush(&u_i, 0);
}

int ADPI_Controller::ADPI_initialize(uint16_t Kp, uint16_t Ki, uint16_t setpoint)
{
    c_state_Kp = Kp; c_Ki = Ki, r_k = setpoint;
    setpointchanged = true;

    circularBufferPush(&u, 0);
    circularBufferPush(&u_i, 0);
}

void ADPI_Controller::configureSP(uint16_t setpoint)
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

uint16_t ADPI_Controller::log_P_control_action()
{
    return circularBufferGetRelativeElement(&u, -1);
}

uint16_t ADPI_Controller::log_I_control_action()
{
    return circularBufferGetRelativeElement(&u_i, -1);
}

int16_t ADPI_Controller::log_error()
{
    return circularBufferGetRelativeElement(&e, -1);
}

void ADPI_Controller::computeADKp()
{   
    uint16_t x = 1/(circularBufferGetRelativeElement(&e, -1) + 0.01);
    
    if(setpointchanged)
    {
        max_error = 1/(circularBufferGetRelativeElement(&e, -1) + 0.01);
        setpointchanged = false;
    }

    if(circularBufferGetRelativeElement(&e, -1) > max_error*RAMP_TS_ONE)
    { 
        ADKp = std::abs(Kp_min + c_state_Kp*x*max_error*RAMP_TS_ONE);
    }

    else if(circularBufferGetRelativeElement(&e, -1) >= max_error*RAMP_TS_TWO)
    {
        ADKp = std::abs(c_state_Kp+Kp_min);
    }

    else
    {
        ADKp = std::abs(Kp_min + (c_state_Kp/x)/(max_error*RAMP_TS_TWO));
    }   

}

int16_t ADPI_Controller::integrator_threshold(int16_t output_value, int16_t  limit_error)
{
    if(output_value > 32000)
    {
        output_value = 32000;
    }
    else if(output_value < -32000)
    {
        output_value = -32000;
    }

    if(limit_error < 25 && limit_error > 0)
    {
        output_value = 0;
    }
    else if(limit_error > -25 && limit_error < 0)
    {
        output_value = 0;
    }

    return output_value;
}

int16_t ADPI_Controller::control_action_threshold(int16_t output_value)
{
    if(output_value > 65535)
    {
        output_value = 65535;
    }
    else if(output_value < -65535)
    {
        output_value = -65535;
    }

    return output_value;
}

int16_t ADPI_Controller::computeControlAction(uint16_t sensor_k, uint16_t time_period)
{
    error_k = r_k - sensor_k;
    circularBufferPush(&e, error_k);

    if (error_k < 0)
    {
        p_action = c_state_Kp*error_k;
        i_action = c_Ki*(time_period*error_k) + circularBufferGetRelativeElement(&u_i, -1);
        i_action = this->integrator_threshold(i_action, error_k);
        
        control_action = p_action + i_action;
        control_action = this->control_action_threshold(control_action);
        circularBufferPush(&u_i, i_action);
        circularBufferPush(&u, p_action);
        return control_action;
    }
    

    p_action = c_state_Kp*error_k;
    i_action = c_Ki*(time_period*error_k) + circularBufferGetRelativeElement(&u_i, -1);
    i_action = this->integrator_threshold(i_action, error_k);

    control_action = p_action + i_action;
    control_action = this->control_action_threshold(control_action);
    circularBufferPush(&u_i, i_action);
    circularBufferPush(&u, p_action);
    
    return control_action;
}

servoIn_Controller::servoIn_Controller(int16_t setpoint1, int16_t setpoint2)
{
    r1_k = setpoint1;
    r2_k = setpoint2;
    setpointOnechanged = true;
    setpointTwochanged = true;
}

int servoIn_Controller::servoIn_initialize(int16_t setpoint1, int16_t setpoint2)
{
    r1_k = setpoint1;
    r2_k = setpoint2;
    setpointOnechanged = true;
    setpointTwochanged = true;

    for(int8_t nb = 0; nb < 5; nb++)
    {
        circularBufferPush(&e, 0);
    }
}

void servoIn_Controller::configureSP(int16_t setpoint, int8_t ID)
{
    if(ID == 1)
    {
        r1_k = setpoint;
        setpointOnechanged = true;
    }
    if(ID == 2)
    {
        r2_k = setpoint;
        setpointTwochanged = true;
    }
}

float servoIn_Controller::integralMax(float integral, int16_t max_value)
{
    if(integral > max_value)
    {
        integral = max_value;
    }
    else if(integral < -max_value)
    {
        integral = -max_value;
    }
    return integral;
}

float servoIn_Controller::error_thresold(int16_t error, int16_t max_value)
{
    if(error > 0 && error < max_value)
    {
        error = 0;
    }
    else if(error < 0 && error > -max_value)
    {
        error = 0;
    }
    return error;
}

float servoIn_Controller::computeStateFeedBack(uint8_t joint, uint8_t state, int16_t setpoint, int16_t state_value, int16_t error, int16_t error_max, float GAIN)
{
    if(joint == 1 && state == 1)
    {
        if(error_max > 0)
        {
            return GAIN*state_value;
        }
        else 
        {
            return GAIN*setpoint/(error_max)*error - GAIN*setpoint;
        }
    }
    if(joint == 1 && state == 2)
    {
        if(error_max > 0)
        {
            return -0.67*state_value;
        }
        else 
        {
            return -0.67*setpoint/(error_max)*error + 0.67*setpoint;
        }
    }
    if(joint == 2 && state == 1)
    {
        if(error_max > 0)
        {
            return -0.3862*state_value;
        }
        else 
        {
            return 0.3862*setpoint/(error_max)*error - 0.3862*setpoint;
        }
    }
    if(joint == 2 && state == 2)
    {
        if(error_max > 0)
        {
            return -6.31*state_value;
        }
        else 
        {
            return -6.31*setpoint/(error_max)*error + 6.31*setpoint;
        }
    }
}

int16_t servoIn_Controller::check_integral_limit(int16_t ri_k)
{
    if(ri_k > 85)
    {
        return 625;
    }
    else
    {
        return 350;
    }
}

float servoIn_Controller::computeControlAction(int16_t sensor1_k, int16_t sensor2_k, int16_t time_period)
{
    if(r1_k == 0)
    {
        r1_k = -10;
    }
    if(r1_k < - 85)
    {
        r1_k = -85;
    }
    e1_k = r1_k - sensor1_k;
    e2_k = r2_k - sensor2_k;

    e1_k = error_thresold(e1_k, 4);
    e2_k = error_thresold(e2_k, 4);
    if(setpointOnechanged)
    {
        x1_l1_est_k = sensor1_k;
        x1_l1_int = 0;
        setpointOnechanged = false;
    }
    if(setpointTwochanged)
    {
        x1_l2_est_k = sensor2_k;
        x1_l2_int = 0;
        setpointTwochanged = false;
    }
    x1_l1_int += e1_k;
    x1_l2_int += e2_k; 
    x1_l1_est_k_1 = -1.15*x1_l1_est_k + 0.005*x2_l1_est_k + 2.15*sensor1_k;
    x2_l1_est_k_1 = 0.76*x2_l1_est_k + 5.28*u1_k + 265.5*(sensor1_k - x1_l1_est_k);
    x1_l2_est_k_1 = -1.14*x1_l2_est_k + 0.005*x2_l2_est_k + 2.14*sensor2_k;
    x2_l2_est_k_1 =  0.812*x2_l2_est_k + 2.82*u2_k + 265*(sensor2_k - x1_l2_est_k);

    x2_l1_est_k = integralMax(x1_l1_int, 1000);
    x2_l2_est_k = integralMax(x1_l1_int, 500);
    x1_l1_int = integralMax(x1_l1_int, 75);
    x1_l2_int = integralMax(x1_l2_int, 75);

    u1_k = 1.505*e1_k -0.02*x2_l1_est_k_1 + 0.0563*x1_l1_int;
    u2_k = 2.836*e2_k -0.0574*x2_l2_est_k_1 + 0.106*x1_l2_int;
    u1_k = integralMax(u1_k, 60);
    u2_k = integralMax(u2_k, 60);

    if(std::abs(u2_k) < 8)
    {
        u2_k = 0;
    }
    x1_l1_est_k = x1_l1_est_k_1; x2_l1_est_k = x2_l1_est_k_1;
    x1_l2_est_k = x1_l2_est_k_1; x2_l2_est_k = x2_l2_est_k_1;
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
    observers[Joint]->Update(setpoint, sensor1_k, sensor2_k, ID, K_iter); 
}

void ISubject::setState(uint8_t sJoint, int16_t spoint)
{
    this->set_point1 = spoint;
    this->Joint = sJoint;
}