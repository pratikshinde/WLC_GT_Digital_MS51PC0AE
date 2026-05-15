#include "ADC_inputs.h"
#include "GPIO_config.h"
#include "MS51_32K.h"
#include "TM1637.h"
#include "Timer3_Int.h"
#include "WLC_functions.h"

void main(void)
{

    unsigned int voltageCheckCount = 0, n = 0;
    uint32_t prev_ticks_100ms, start_ticks_100ms, display_ticks_100ms;
    char configLEDTimeOut = 0;
    unsigned char configRequested = 0;
    Timer3_INT_Initial(DIV128, 0xb6, 0xc2);
    MODIFY_HIRC(HIRC_24);
    Enable_UART0_VCOM_printf_24M_115200();
    init_GPIO();
    clr_IE_ES;
    printf("\r\nBoot...");
    prev_ticks_100ms = getTicks100ms();
    start_ticks_100ms = prev_ticks_100ms;
    while ((getTicks100ms() - prev_ticks_100ms) < 20)
        ;
    prev_ticks_100ms = getTicks100ms();
    // printf("\r\ninit Display...");
    TM1637_init(1 /*enable*/, 5 /*brightness*/);
    TM1637_WriteString(1, "POD");

    while ((getTicks100ms() - prev_ticks_100ms) < 60)
    {
        // Blink LEDs and check for config button SW2 state
        if ((getTicks100ms() - start_ticks_100ms) > 10)
        {
            configLEDTimeOut = !configLEDTimeOut;
            start_ticks_100ms = getTicks100ms();
            TM1637_WriteString(1, "SET");
            TM1637_WriteNumber(2, n);
            n++;
        }
        if (configLEDTimeOut)
        {
            allLEDsON();
        }
        else
        {
            allLEDsOFF();
        }
        if (readButton() == 2)
        {
            configRequested = 1;
            break;
        }
    }
    printf("\r\nRead Settings...");
    allLEDsOFF();

    readSettingsEEPROM();
    readSettingsMotorOffRemainingEEPROM();
    readBandgapCalibratedValue();
    // updateSensorLED();
    if (configRequested)
    {
        // readSettingsEEPROM();
        configSettings();
    }
    prev_ticks_100ms = getTicks100ms();
    start_ticks_100ms = getTicks100ms();
    powerOnDelayCounter();
    printf("\r\nwlc loop...");
    prev_ticks_100ms = getTicks100ms();
    display_ticks_100ms = getTicks100ms();

    while (1)
    {
        if (getTicks100ms() - prev_ticks_100ms > 1) // execute every 200ms
        {
            updateSensorLED();
            prev_ticks_100ms = getTicks100ms();
            wlcLoop();
            buzzerLoop();
            refreshCurrentDisplay();
            voltageCheckCount++;
        }

        if (getTicks100ms() - display_ticks_100ms >= 30) // execute every 3 seconds
        {
            display_ticks_100ms = getTicks100ms();
            handleDisplayRolling();
        }

        readVdd();
        readACVoltage();
        voltageCheckCount = 0;
    }
}