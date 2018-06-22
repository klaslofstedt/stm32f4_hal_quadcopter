#include "pid.h"
#include "uart_print.h"
#include "filter.h"
#include "functions.h"

// Might wanna have a better error handling when using delta input as rate:
// https://github.com/Lauszus/LaunchPadFlightController/blob/fddbe4eb9303ea4301a714585d7383a2de275d80/src/PID.c

void PID_Calc(PID_t *pid)
{
    // These are used for ease to read
    float p_term, i_term, d_term, error, output;
    float dt = pid->dt;
    // Calculate input rate with derivation of position instead from EKF output
    // TODO: Does it need low pass-filter?                        
    //pid->rate_calc = filter_lowpass((float)((pid->input - pid->last_input) / (float)dt), pid->rate_calc, 0.75);
    //pid->rate_calc = ((pid->input - pid->last_input) / (float)dt);
    pid->rate_calc = (pid->input - pid->last_input) / dt;
    
    // Calculate error between current and desired position
    error = pid->setpoint - pid->input;
    
    // Calculate the P contribution
    p_term = pid->k_p * error;
    
    // Calculate the I contribution
    pid->i_term += (float)(pid->k_i * dt * ((error + pid->last_error) / 2.0f));
    
    // Constrain i_term
    i_term = constrain(pid->i_term, pid->boundary_min, pid->boundary_max);
    
    // Calculate the D contribution
    d_term = pid->k_d * pid->rate;
    
    //Calculate output
    output = p_term + i_term - d_term;
    
    // Set data for output and next loop and constrain output
    pid->output = constrain(output, pid->boundary_min, pid->boundary_max);
    pid->last_input = pid->input;
    pid->last_error = error;
}