#ifndef PID_h
#define PID_h

typedef struct{
    // Set these in init
    float k_p;
    float k_i;
    float k_d;
	float input;
    float setpoint;
    float boundary_max;
    float boundary_min;
    // Temp variables not to be set
    float last_input;
    float rate;
    float rate_calc;
	float i_term;
    float last_error;
	float output;
}PID_t;

void PID_Calc(PID_t *pid, unsigned long dt);

#endif
