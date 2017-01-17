#ifndef __LOWFILTER_H
#define __LOWFILTER_H


float _cutoff_freq = 0.0f;
float _a1 = 0.0f;
float _a2 = 0.0f;
float _b0 = 0.0f;
float _b1 = 0.0f;
float _b2 = 0.0f;
float _delay_element_1 = 0.0f;
float _delay_element_2 = 0.0f;

/*
param:
	sample :������
	cutoff_freq	:������

*/
void LowPassFilter2p_set_cutoff_frequency(float sample_freq, float cutoff_freq);
		
/*
param:
	sample:�����趨ֵ
*/		
float LowPassFilter2p_apply(float sample);








#endif
