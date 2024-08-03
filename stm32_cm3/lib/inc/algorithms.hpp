/**
 * @file algorithms.hpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief Head file for general algorithms.
 * @version 0.1
 * @date 2023-07-02
 * 
 */

#ifndef algorithms_HPP
#define algorithms_HPP

#define SYSTEM_PERIOD_FOR_CALCULAT       0.001
#define RAMP_TS_ONE         0.2
#define RAMP_TS_TWO         0.8 

#include <vector>
#include <cstdlib>
#include <stdint.h>   
#include <algorithm>
#include <string>
#include "math.h"

#include "utils.h"
#include "ADC_peripheral.hpp"
#include "PWM_peripheral.hpp"

#define variable_k1(variable, index)        circularBufferGetRelativeElement(&variable, index)

class P_Controller
{
    public:
        uint16_t c_state_Kp;
        uint16_t control_action;

        bool setpointchanged;

        P_Controller(uint16_t Kp, uint16_t setpoint);

        void configureSP(uint16_t setpoint);
        uint16_t computeControlAction(uint16_t sensor_k, uint16_t time_period);

    private:
        uint16_t r_k;
        uint16_t error_k;
};  

class ADPI_Controller
{
    public:
        uint16_t c_state_Kp;
        uint16_t c_Ki;
        float control_action;
        uint16_t r_k;


        float p_action;
        float i_action;

        bool setpointchanged;

        sensors_circular_buffer_t u;
        sensors_circular_buffer_t u_i;
        sensors_circular_buffer_t e;

        ADPI_Controller(){}
        ADPI_Controller(uint16_t Kp, uint16_t Ki, uint16_t setpoint);


        int ADPI_initialize(uint16_t Kp, uint16_t Ki, uint16_t setpoint);
        void configureSP(uint16_t setpoint);
        void computeADKp();
        int16_t computeControlAction(uint16_t sensor_k, uint16_t time_period);
        int16_t control_action_threshold(int16_t output_value);
        int16_t integrator_threshold(int16_t output_value, int16_t limit_error);

        int16_t log_setpoint();
        int16_t log_error();
        uint16_t log_P_control_action();
        uint16_t log_I_control_action();

    private:
        int16_t error_k;
        int16_t max_error;
        float ADKp;
        uint16_t Kp_min = 0.5;
};  

class servoIn_Controller
{
    public:
        float k1;
        int16_t state_one;
        int16_t state_one_pass = 0;
        int16_t state_two;
        int16_t r1_k;
        int16_t e1_k = 0;
        int16_t r2_k;
        int16_t e2_k = 0;
        int16_t e1_k_1 = 0;
        int16_t e2_k_1 = 0;
        float u1_k = 0;
        float u2_k = 0;
        bool setpointchanged;
        float x1_l1_est_k_1;
        float x2_l1_est_k_1;
        float x1_l1_est_k = 0;
        float x2_l1_est_k = 0;
        float x1_l2_est_k_1;
        float x2_l2_est_k_1;
        float x3_l2_est_k_1;
        float x1_l2_est_k = 0;
        float x2_l2_est_k = 0;
        float x3_l2_est_k;
        float x1_l1_int;
        float x1_l2_int;

        sensors_circular_buffer_t e;
        servoIn_Controller(){}
        servoIn_Controller(int16_t setpoint1, int16_t setpoint2);


        int servoIn_initialize(int16_t setpoint1, int16_t setpoint2);
        void configureSP(int16_t setpoint1, int16_t setpoint2);
        float computeControlAction(int16_t sensor1_k, int16_t sensor2_k, int16_t time_period);
        float integralMax(float integral, int16_t max_value);
        float error_thresold(int16_t error, int16_t max_value);
        float computeStateFeedBack(uint8_t joint, uint8_t state, int16_t setpoint, int16_t state_value, int16_t error, int16_t error_max, float GAIN);
        int16_t check_integral_limit(int16_t ri_k);


    private:
        int16_t error_k;
        int16_t max_error;
};  

class IObserver 
{
    public:
        int id;

        virtual ~IObserver(){};
        virtual void Update(int8_t setpoint1, int8_t setpoint2);
};

class ISubject 
{
    public:
        std::vector<IObserver *> observers;

        uint8_t Joint;
        int16_t set_point1;  
        int16_t set_point2;   

        void Attach(IObserver *observer);
        void Detach(IObserver *observer);
        void Notify();
        void setState(uint8_t sJoint, int16_t spoint);
};

#endif
