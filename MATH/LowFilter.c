#include <math.h>
#include "LowFilter.h"

#ifndef M_PI_F
#define M_PI_F 3.14159f
#endif

void LowPassFilter2p_set_cutoff_frequency(float sample_freq, float cutoff_freq)
{
		float fr;
    float ohm;
    float c;
	
    _cutoff_freq = cutoff_freq;
  
  if (_cutoff_freq <= 0.0f) {
        // no filtering
        return;
    }

		fr = sample_freq/_cutoff_freq;
		ohm = tanf(M_PI_F/fr);
		c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
		
    _b0 = ohm*ohm/c;
    _b1 = 2.0f*_b0;
    _b2 = _b0;
    _a1 = 2.0f*(ohm*ohm-1.0f)/c;
    _a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
}

float LowPassFilter2p_apply(float sample)
{
		float delay_element_0;
		float output;
    if (_cutoff_freq <= 0.0f) {
        // no filtering
        return sample;
    }

    // do the filtering
    delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
    output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;
    
    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}




