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
#include <stdint.h>   

#ifndef utills_HPP
#define utills_HPP

class ADPI_Controller
{
    public:
        float ramp_Coef;
        float c_state_Kp;
        float c_Ki;

        bool setpointchanged;

        std::vector<float> u = {0};
        std::vector<float> e = {};

        ADPI_Controller(float Kp, float Ki, float rCoef, float setpoint);

        void configureSP(float setpoint);
        float computeADKp();
        float computeControlAction(float error);

    private:
        float r_k;
        float error_k;
        float max_error;
        float ADKp;
        float control_action;

        float Kp_min = 0.02;
};  

#endif
