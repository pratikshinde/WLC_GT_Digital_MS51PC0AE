#ifndef __ADC_INPUTS__
#define __ADC_INPUTS__
#include "MS51_32K.h"

#define ADC_DELAY   200
#define SW1_ADC_MIN 3100
#define SW2_ADC_MIN 2600
#define SW2_ADC_MAX 3000
#define SW3_ADC_MIN 2300
#define SW3_ADC_MAX 2500
unsigned int readButton(void);
float readMotorCurrent(unsigned char motor_channel);
unsigned int readACVoltage(void);
void readVdd(void);
void readBandgapCalibratedValue(void);
unsigned char checkLineVoltage(unsigned int minACVolt, unsigned int maxACVolt, unsigned char motorState);
unsigned int getLineVoltage(void);
#endif