#include "filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

float h;
float v;
float Q_accel = 0.1;
float R_altitude = 1;

float P[2][2] =
{
    { 1.0f,     0.0f },
    { 0.0f,     1.0f }
};



float Altitude_KF(float acceleration, float altitude, float dt)
{
    
	// Repeated arithmetics
	float _dtdt = dt * dt;
    
	// The state vector is defined as x = [h v]' where  'h' is altitude above ground and 'v' velocity, both
	// aligned with the vertical direction of the Earth NED frame, but positive direction being upwards to zenith.
    
	// State-space system model 'x_k = A*x_k-1 + B*u_k is given by:
	//
	//	x_k = [ h_k ] = [ 1 dT ] * [ h_k-1 ] + [ 1/2*dT^2 ] * u_k
  	//  	      [ v_k ]   [ 0  1 ]   [ v_k-1 ]   [ dT       ]
	//
	//			   A			     B
	//
	// where 'u_k' is our acceleration input signal.
    
	// Propagation of the state (equation of motion) by Euler integration
	h = h + v*dt + 0.5f*acceleration*_dtdt;
	v = v + acceleration*dt;
    
	// The "a priori" state estimate error covariance 'P_k|k-1 = A * P_k-1 * A' + Q_k' is calculated as follows:
	//
	// P_k|k-1 = [ 1 dT ] * P_k-1 * [  1 0 ] + Q_k
	//	     [ 0  1 ]	        [ dT 1 ]
    
	// The process noise covariance matrix 'Q' is a bit trickier to derive, but consider some additive noise 'w_k' perturbing the
	// true acceleration 'a_k', thus the input signal is 'u_k = a_k + w_k'. The affect of 'w_k' on the state estimate is by linearity
	// described by [1/2*dT^2 dT]' i.e. the input matrix 'B'. We call this new matrix 'G'.
	//
	// Then, by definition* 'Q' equals 'G * G' * s^2', which in our case translates into:
	//
	// Q_k = G_k * G'_k * s_accelerometer^2 = [(dT^4)/4 (dT^3)/2] * s_accelerometer^2
	//					  [(dT^3)/2     dT^2]
	//
	// * I only get half of the math showing 'Q = G * G' * s^2', so I hide myself behind 'by definition'.
    
	// Calculate the state estimate covariance
	//
	// Repeated arithmetics
	float _Q_accel_dtdt = Q_accel * _dtdt;
	//
	P[0][0] = P[0][0] + (P[1][0] + P[0][1] + (P[1][1] + 0.25f*_Q_accel_dtdt) * dt) * dt;
	P[0][1] = P[0][1] + (P[1][1] + 0.5f*_Q_accel_dtdt) * dt;
	P[1][0] = P[1][0] + (P[1][1] + 0.5f*_Q_accel_dtdt) * dt;
	P[1][1] = P[1][1] + _Q_accel_dtdt;
    
    
        
	// Observation vector 'zhat' from the current state estimate:
	//
	// zhat_k = [ 1 0 ] * [ h_k ]
	//                    [ v_k ]
	//             H
    
	// 'H' is constant, so its time instance I'm using below is a bit ambitious.
    
	// The innovation (or residual) is given by 'y = z - zhat', where 'z' is the actual observation i.e. measured state.
    
	// Calculate innovation, in this particular case we observe the altitude state directly by an altitude measurement
	float y = altitude - h;
    
	// The innovation covariance is defined as 'S_k = H_k * P_k|k-1 * H'_k + R_k', for this particular case
	// 'H_k * P_k|k-1 * H'_k' is equal to the first row first column element of 'P_k|k-1' i.e. P_00.
    
	// The Kalman gain equals 'K_k = P_k|k-1 * H'_k * S_k^-1', where
	//
	// P_k|k-1 * H'_k = [ P_00 ]
	//                  [ P_10 ]
	//
	// and 'S_k^-1' equals '1/S_k' since 'S_k^-1' is being a scalar (that is a good thing!).
    
	// Calculate the inverse of the innovation covariance
	float Sinv = 1.0f / (P[0][0] + R_altitude);
    
	// Calculate the Kalman gain
	float K[2] = { P[0][0] * Sinv, P[1][0] * Sinv };
    
	// Update the state estimate
	h += K[0] * y;
	v += K[1] * y;
    
	// The "a posteriori" state estimate error covariance equals 'P_k|k = (I - K_k * H_k) * P_k|k-1', where
	//
	//  (I - K_k * H_k) = ( [ 1 0 ] - [ K_0 ] * [ 1 0 ] ) = [ (1-K_0) 0  ] , thus
	//                    ( [ 0 1 ]   [ K_1 ]           )   [ -K_1    1  ]
	//
	//  P_k|k = (I - K_k * H_k) * P_k+1|k = [ (1-K_0) 0 ] * [ P_00 P_01 ] = [ (1-K_0)*P_00       (1-K_0)*P_01       ]
	//					[ -K_1    1 ]   [ P_10 P_11 ]   [ (-K_1*P_00 + P_10) (-K_1*P_01 + P_11) ]
    
	// Calculate the state estimate covariance
	P[0][0] = P[0][0] - K[0] * P[0][0];
	P[0][1] = P[0][1] - K[0] * P[0][1];
	P[1][0] = P[1][0] - (K[1] * P[0][0]);
	P[1][1] = P[1][1] - (K[1] * P[0][1]);
    
    return h;
}



void Filter_Kalman(FilterKalman_t *k)
{
    // x = F*x + B*u
    float x_temp[2];
    x_temp[0] = k->F[0][0] * k->x[0] + k->F[0][1] * k->x[1] + k->B[0]*k->u; // Position
    x_temp[1] = k->F[1][0] * k->x[0] + k->F[1][1] * k->x[1] + k->B[1]*k->u; // Velocity
    k->x[0] = x_temp[0];
    k->x[1] = x_temp[1];
    //rate = u - x[1];
    
    // P = F * P * Ft + Q
    float P_temp[2][2]; // Why is Q multiplied with dt?
    P_temp[0][0] = k->F[0][0] * (k->F[0][0]*k->P[0][0]+k->F[0][1]*k->P[1][0]) + k->F[0][1] * (k->F[0][0]*k->P[0][1]+k->F[0][1]*k->P[1][1]) + k->Q[0]*k->dt;
    P_temp[0][1] = k->F[1][0] * (k->F[0][0]*k->P[0][0]+k->F[0][1]*k->P[1][0]) + k->F[1][1] * (k->F[0][0]*k->P[0][1]+k->F[0][1]*k->P[1][1]);
    P_temp[1][0] = k->F[0][0] * (k->F[1][0]*k->P[0][0]+k->F[1][1]*k->P[1][0]) + k->F[0][1] * (k->F[1][0]*k->P[0][1]+k->F[1][1]*k->P[1][1]);
    P_temp[1][1] = k->F[1][0] * (k->F[1][0]*k->P[0][0]+k->F[1][1]*k->P[1][0]) + k->F[1][1] * (k->F[1][0]*k->P[0][1]+k->F[1][1]*k->P[1][1]) + k->Q[1]*k->dt;
    
    k->P[0][0] = P_temp[0][0];
    k->P[0][1] = P_temp[0][1];
    k->P[1][0] = P_temp[1][0];
    k->P[1][1] = P_temp[1][1];
    
    // S = H*P * Ht + R
    k->S[0][0] = k->H[0][0] * k->H[0][0] * k->P[0][0] + k->R[0];
    k->S[0][1] = k->H[0][1] * k->H[1][0] * k->P[0][1];
    k->S[1][0] = k->H[0][1] * k->H[1][0] * k->P[1][0];
    k->S[1][1] = k->H[1][1] * k->H[1][1] * k->P[1][1] + k->R[1];
    
    // K = P * Ht * S^-1
    float S_inv = k->S[0][0]*k->S[1][1] - k->S[0][1]*k->S[1][0];
    
    k->K[0][0] = ((k->H[0][0]*k->P[0][0] + k->H[0][1]*k->P[0][1])*k->S[1][1] - (k->H[1][0]*k->P[0][0] + k->H[1][1]*k->P[0][1])*k->S[1][0]) / S_inv;
    k->K[0][1] = ((k->H[1][0]*k->P[0][0] + k->H[1][1]*k->P[0][1])*k->S[0][0] - (k->H[0][0]*k->P[0][0] + k->H[0][1]*k->P[0][1])*k->S[0][1]) / S_inv;
    k->K[1][0] = ((k->H[0][0]*k->P[1][0] + k->H[0][1]*k->P[1][1])*k->S[1][1] - (k->H[1][0]*k->P[1][0] + k->H[1][1]*k->P[1][1])*k->S[1][0]) / S_inv;
    k->K[1][1] = ((k->H[1][0]*k->P[1][0] + k->H[1][1]*k->P[1][1])*k->S[0][0] - (k->H[0][0]*k->P[1][0] + k->H[0][1]*k->P[1][1])*k->S[0][1]) / S_inv;
    
    // y = z - (H*x)
    k->y[0] = k->z[0] - (k->H[0][0]*k->x[0] + k->H[0][1]*k->x[1]);
    k->y[1] = k->z[1] - (k->H[1][0]*k->x[0] + k->H[1][1]*k->x[1]);
    
    // x = x + (K*y)
    k->x[0] = k->x[0] + k->K[0][0]*k->y[0] + k->K[0][1]*k->y[1];
    k->x[1] = k->x[1] + k->K[1][0]*k->y[0] + k->K[1][1]*k->y[1];
    
    // P = (I - K*H)*P
    P_temp[0][0] = k->P[0][0]*(1 - k->H[0][0]*k->K[0][0] - k->H[1][0]*k->K[0][1]);
    P_temp[0][1] = k->P[0][1]*(0 - k->H[0][1]*k->K[0][0] - k->H[1][1]*k->K[0][1]);
    P_temp[1][0] = k->P[1][0]*(0 - k->H[0][0]*k->K[1][0] - k->H[1][0]*k->K[1][1]);
    P_temp[1][1] = k->P[1][1]*(1 - k->H[0][1]*k->K[1][0] - k->H[1][1]*k->K[1][1]);
    
    k->P[0][0] = P_temp[0][0];
    k->P[0][1] = P_temp[0][1];
    k->P[1][0] = P_temp[1][0];
    k->P[1][1] = P_temp[1][1];
}



void Filter_Average(FilterAverage_t *avg)
{
    if(avg->counter <= avg->sample_start){
        // do nothing until samples has stabilized
        avg->counter++;
    }
    if((avg->counter > avg->sample_start) && (avg->counter < avg->sample_stop)){
        avg->total += (avg->sample);
        avg->counter++;
        avg->average = avg->total /(avg->counter - avg->sample_start);
        //Serial.print(offset);
        //Serial.print(", ");
    }
    if(avg->counter >= avg->sample_stop){
        avg->ready = true; 
    }
}

void Filter_Lowpass(FilterLowpass_t *lp)
{
    const float tau = 1.0f/(2.0f*M_PI*lp->cf); // Time constant
    const float alpha = lp->dt/(tau + lp->dt); // See (10) in http://techteach.no/simview/lowpass_filter/doc/filter_algorithm.pdf
    
    // y(n) = y(n-1) + alpha*(u(n) - y(n-1))
    lp->output = lp->output_last + alpha*(lp->input - lp->output_last);
    lp->output_last = lp->output;
}

float Filter_Transition(float data1, float data2, float damping)
{
    return (data1 * (1.0f - damping) + data2 * damping);
}