#ifndef __ADC_INPUTS__
#define __ADC_INPUTS__
#include "MS51_32K.h"

#define ADC_DELAY   200
#define SW1_ADC_MIN 3100
#define SW2_ADC_MIN 2600
#define SW2_ADC_MAX 3000
#define SW3_ADC_MIN 2300
#define SW3_ADC_MAX 2500

/* CT Coil 1:2500 with 100R burden calibration multiplier:
 * Scaled so that 7.0A measured reading matches 10.0A panel meter */
#define CT_CALIBRATION_FACTOR   (10.0f / 7.0f)

unsigned int readButton(void);
float readMotorCurrent(void);
void resetMotorCurrentFilter(void);
unsigned int readACVoltage(void);
void readVdd(void);
void readBandgapCalibratedValue(void);
unsigned char checkLineVoltage(unsigned int minACVolt, unsigned int maxACVolt, unsigned char motorState, unsigned char transformerType);
unsigned int getLineVoltage(void);
#endif