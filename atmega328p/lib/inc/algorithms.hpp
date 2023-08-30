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

#include <stdint.h>   
#include "utils.h"

#ifndef algorithms_HPP
#define algorithms_HPP

class P_Controller
{
    public:
        float c_state_Kp;
        float control_action;

        bool setpointchanged;

        P_Controller(float Kp, float setpoint);

        void configureSP(float setpoint);
        float computeControlAction(float sensor_k, float time_period);

    private:
        float r_k;
        float error_k;
};  

class ADPI_Controller
{
    public:
        float c_state_Kp;
        float c_Ki;
        float control_action;
        uint16_t r_k;


        float p_action;
        float i_action;

        bool setpointchanged;

        sensors_circular_buffer_t u;
        sensors_circular_buffer_t u_i;
        sensors_circular_buffer_t e;

        ADPI_Controller(float Kp, float Ki, int16_t setpoint);

        void configureSP(float setpoint);
        float computeADKp();
        float computeControlAction(float sensor_k, float time_period);
        float threshold(float output_value, float  limit_error);
        int16_t log_setpoint();
        int16_t log_error();
        float log_P_control_action();
        float log_I_control_action();

    private:
        float error_k;
        float max_error;
        float ADKp;
        float Kp_min = 0.5;
};  

#endif
