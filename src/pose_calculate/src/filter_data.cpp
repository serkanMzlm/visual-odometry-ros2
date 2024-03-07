#include "filter_data.hpp"

float constrain(float value, float min_value, float max_value){
    return fminf(max_value, fmaxf(value, min_value));
}

float mapValues(float value, float in_min, float in_max, 
                            float  out_min, float out_max){
    float in_step_size = in_max - in_min;
    float out_step_size = out_max -  out_min;
    return ((((value - in_min) * out_step_size) / in_step_size) + out_min);
}

LinearKalman::LinearKalman(float Q, float R) : 
              process_noise(Q), measurement_noise(R) { 
}

float LinearKalman::filter(float measurement){
	float new_covariance = previous_covariance + process_noise;

	// Calculate Kalman gain
	float gain = new_covariance / (new_covariance + measurement_noise);

	// Update step
	previous_estimate = previous_estimate + gain * (measurement - previous_estimate);
	previous_covariance = (1 - gain) * previous_covariance;

	return previous_estimate;
}