#include "ADC_inputs.h"
#include "GPIO_Config.h"
#include "UART2_functions.h"
#include "delay.h"

float Bandgap_Voltage, VDD_Voltage, lineVoltage;
unsigned int corrected_volatge;
unsigned int getLineVoltage(void)
{
    return corrected_volatge;
}
unsigned char checkLineVoltage(unsigned int minACVolt, unsigned int maxACVolt, unsigned char motorState, unsigned char transformerType)
{
    unsigned char ret_val;
    corrected_volatge = 0;
    // readACVoltage();
    if (motorState && transformerType == 0)
    {
        /* 12-0-12 center tap: compensate for voltage drop when relay is ON */
        corrected_volatge = lineVoltage + (0.14 * lineVoltage);
        if ((minACVolt > corrected_volatge) || (maxACVolt < corrected_volatge))
        {
            printf("FLT Volt: %d", corrected_volatge);
            ret_val = 0;
            // L_VOLT_FLT = 1;
        }
        else
        {
            printf("CRTVolt: %d", corrected_volatge);
            ret_val = 1;
            // L_VOLT_FLT = 0;
        }
    }
    else
    {
        if (((minACVolt > lineVoltage) || (maxACVolt < lineVoltage)))
        {
            printf("FLTVolt: %d", (int)lineVoltage);
            ret_val = 0;

            // L_VOLT_FLT = 1;
        }
        else
        {
            printf("CRTVolt: %d", (int)lineVoltage);
            ret_val = 1;
            // L_VOLT_FLT = 0;
        }
        corrected_volatge = (int)lineVoltage;
    }

    return ret_val;
}
void readBandgapCalibratedValue(void)
{
    Bandgap_Voltage = READ_BANDGAP() * 3 / 4;
    printf("\r\nBG V:%0.2f", Bandgap_Voltage);
}
unsigned int readButton(void)
{
    unsigned int temp;
    unsigned int ret = 0;
    ENABLE_ADC_CH2;
    ADCCON1 |= 0X30; /* clock divider */
    ADCCON2 |= 0X0E; /* AQT time */
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;
    while (!(ADCCON0 & SET_BIT7))
        ; /* wait ADCF */
    // printf(" after");
    temp = (ADCRH << 4) + (ADCRL & 0x0F);
    DISABLE_ADC;
    Timer0_Delay(24000000, 10, 1000);
    if (temp > SW2_ADC_MIN && temp < SW2_ADC_MAX)
    {
        printf(" SW2");
        ret = 2;
    }
    else if (temp > SW1_ADC_MIN)
    {
        printf(" SW1");
        ret = 1;
    }
    else if (temp > SW3_ADC_MIN && temp < SW3_ADC_MAX)
    {
        printf(" SW3");
        ret = 3;
    }
    return ret;
}

unsigned int readACVoltage(void)
{
    unsigned int temp;
    unsigned int max_val = 0;
    unsigned int min_val = 4096;
    unsigned int i;
    unsigned int dc_adc;
    float volt = 0;
    static float filtered_voltage = 0.0f;

    ENABLE_ADC_CH6;
    ADCCON1 |= 0X30; /* clock divider */
    ADCCON2 |= 0X0E; /* AQT time */

    /* Peak-to-peak midpoint detection over ~20ms (2 full 100Hz ripple cycles).
     * 400 samples x 50us spacing = 20ms.
     * True DC value = (max + min) / 2, immune to ripple phase. */
    for (i = 0; i < 400; i++)
    {
        clr_ADCCON0_ADCF;
        set_ADCCON0_ADCS;
        while (!(ADCCON0 & SET_BIT7))
            ; /* wait ADCF */
        temp = (ADCRH << 4) + (ADCRL & 0x0F);
        if (temp > max_val)
            max_val = temp;
        if (temp < min_val)
            min_val = temp;
        Timer0_Delay(24000000, 1, 50); /* 50us between samples */
    }

    DISABLE_ADC;
    SFRS = 0; /* Ensure Page 0 for UART SFRs */

    dc_adc = (max_val + min_val) / 2;
    volt = ((dc_adc * VDD_Voltage) / 4096);
    volt = 0.076 * volt + 20;

    /* Faster IIR low-pass filter for response within 2-3s (alpha = 0.5) */
    if (filtered_voltage == 0.0f)
        filtered_voltage = volt;
    filtered_voltage = (filtered_voltage * 0.5f) + (volt * 0.5f);

    lineVoltage = filtered_voltage;
    printf("ADC DC:%u (Max:%u Min:%u) Vdd:%0.2f AC Voltage:%0.1f",
           dc_adc, max_val, min_val, VDD_Voltage / 1000, lineVoltage);
    return dc_adc;
}
void readVdd(void)
{
    float bgvalue;
    unsigned int temp = 0;
    ENABLE_ADC_BANDGAP;
    ADCCON1 |= 0X30; /* clock divider */
    ADCCON2 |= 0X0E; /* AQT time */
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;
    while (!(ADCCON0 & SET_BIT7))
        ;
    ;
    bgvalue = (ADCRH << 4) + (ADCRL & 0x0F);
    DISABLE_ADC;
    Timer0_Delay(24000000, 10, 1000);
    VDD_Voltage = (0x1000 / bgvalue) * Bandgap_Voltage;
}

float readMotorCurrent(uint8_t motor_channel)
{
    unsigned int temp;
    unsigned int max_val = 0;
    unsigned int min_val = 4096;
    unsigned int i;
    float current_rms_mA = 0.0;
    unsigned int peak_adc = 0;
    static float filtered_rms = 0.0;

    printf("\r\nCT Reading.");

    // Config ADC Channel
    if (motor_channel == 1)
    {
        ENABLE_ADC_CH9;
    }
    else
    {
        ENABLE_ADC_CH1;
    }

    ADCCON1 |= 0X30; /* clock divider */
    ADCCON2 |= 0X0E; /* AQT time */

    // Sample for ~100ms (2000 samples with 50us gap) to cover 5 cycles of 50Hz
    for (i = 0; i < 2000; i++)
    {
        clr_ADCCON0_ADCF;
        set_ADCCON0_ADCS;
        while (!(ADCCON0 & SET_BIT7))
            ; /* wait ADCF */
        temp = (ADCRH << 4) + (ADCRL & 0x0F);
        if (temp > max_val)
            max_val = temp;
        if (temp < min_val)
            min_val = temp;

        // Small delay to stretch sampling across multiple AC cycles
        Timer0_Delay(24000000, 1, 50);

        if (i % 500 == 0)
            printf(".");
    }

    DISABLE_ADC;
    SFRS = 0; // Ensure we are back in Page 0 for other SFRs like UART

    // Peak-to-Peak detection handles bias shift (max - min)
    if (max_val > min_val)
    {
        unsigned int diff = max_val - min_val;
        if (diff > 12)
        {
            peak_adc = diff / 2;
        }
        else
        {
            peak_adc = 0;
        }
    }

    // Calculate Voltage and Current
    if (VDD_Voltage > 0)
    {
        float volt_mV = (peak_adc * (float)VDD_Voltage) / 4096.0f;
        float current_peak_mA_sec = volt_mV / 10.0f;              // 10 Ohm Burden
        float current_rms_mA_sec = current_peak_mA_sec * 0.7071f; // Peak to RMS

        // Convert to primary current (Assuming 1000:1 CT ratio)
        // Calibrating: Multimeter 2.5A -> Firmware 6.0A.
        // New Factor = 0.70 * (2.5 / 6.0) = 0.2916
        current_rms_mA = (current_rms_mA_sec * 1000.0f) * 0.19f;

        // Low-pass Filter for display stability
        if (filtered_rms == 0.0f)
            filtered_rms = current_rms_mA;
        filtered_rms = (filtered_rms * 0.8f) + (current_rms_mA * 0.2f);

        printf("\r\n Max: %u, Min: %u, P-P: %u, mA: %d",
               (unsigned int)max_val,
               (unsigned int)min_val,
               (unsigned int)(max_val - min_val),
               (int)filtered_rms);

        current_rms_mA = filtered_rms;
    }
    else
    {
        printf("\r\n Raw: %u-%u (No VDD)", (unsigned int)max_val, (unsigned int)min_val);
    }

    printf("..Done\r\n");
    return current_rms_mA;
}