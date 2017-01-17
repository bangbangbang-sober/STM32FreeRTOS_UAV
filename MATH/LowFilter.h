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
	sample :采样率
	cutoff_freq	:采样数

*/
void LowPassFilter2p_set_cutoff_frequency(float sample_freq, float cutoff_freq);
		
/*
param:
	sample:过滤设定值
*/		
float LowPassFilter2p_apply(float sample);








#endif
