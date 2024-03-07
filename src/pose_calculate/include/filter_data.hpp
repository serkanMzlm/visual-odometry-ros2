#ifndef __FILTER_DATA_HPP__ 
#define __FILTER_DATA_HPP__

#include <math.h>

float constrain(float value, float min_value, float max_value);
float mapValues(float value, float in_min, float in_max, 
                            float  out_min, float out_max);


class LinearKalman{
private:
    float process_noise       = 0.0f; // Q
    float measurement_noise   = 0.0f; // R
    float previous_covariance = 0.0f;
    float previous_estimate   = 0.0f;
public:
    LinearKalman(float Q, float R);
    float filter(float measurement);
};

#endif