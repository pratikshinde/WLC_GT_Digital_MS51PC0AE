#include <stdio.h>
#include "WLC_functions.h"
#include "ADC_inputs.h"
#include "GPIO_config.h"
#include "TM1637.h"
#include "Timer3_Int.h"
#include "debugStrings.h"

struct
{
    unsigned int state;                   // Integrity Check
    unsigned int dryRunTimeOut;           // in Seconds 0 to 999
    long dryRunReset;                     // in minutes 0 to 999
#ifdef MINIMUM_FUNCTIONALITY
    unsigned int powerOnDelaySeconds;     // POS: in Seconds 0 to 999
    unsigned int powerOnDelayMinutes;     // POM: in Minutes 0 to 999
    unsigned int powerOnDelayHours;       // POH: in Hours 0 to 999
#else
    unsigned int powerOnDelay;            // POD: in Seconds 0 to 999
#endif
    unsigned int secondaryDryRunDuration; // in seconds 0 to 999
    unsigned int tankOnLevel;
    unsigned int tankOffLevel;
    unsigned int motorOnTime1;
    unsigned int motorOffTime1;
    unsigned int motorOnTime2;
    unsigned int motorOffTime2;

    unsigned int offTimeTankOffLevel;
    unsigned char dualTankEnable;
    unsigned int sumpType;   // 0 : Bore motor, do not check sump levels, 1: Single Sump, 2: Dual Sump
    unsigned int minACVolt;  // 120 to 180V
    unsigned int maxACVolt;  // 250 to 280V
    unsigned int relayPulse; //
    unsigned int starterSelection;
    unsigned char workMode;
    unsigned char dualMotorEnable; // Enable dual sump motors
    unsigned char motorMode;       // Choose between ACO, Sump Alternate, Sump Only, Bore Only, Combo (bore to sump and sump to tank)
    unsigned int motor1Cycles;     // ACC mode: how many times Motor1 runs per alternation
    unsigned int motor2Cycles;     // ACC mode: how many times Motor2 runs per alternation
    unsigned char ctEnable;        // 0: Disable, 1: Enable
    float referenceCurrent1;
    float referenceCurrent2;
    unsigned char transformerType; // 0: 12-0-12 (center tap), 1: 12-0-12-0 (dual output)
    unsigned char voltageLimitEnable; // 0: Disable, 1: Enable
    unsigned int state2;
} GlobalSettings;

struct
{
    char state; // Integrity Check
    char motorOffTimeEnabled;
    long motorOffTimeRemainingTime; // In seconds
    char state2;
} GlobalMotorOffTimeStruct;

struct
{
    unsigned char stateS25 : 1;
    unsigned char stateS50 : 1;
    unsigned char stateS75 : 1;
    unsigned char stateS100 : 1;

    unsigned char stateT25 : 1;
    unsigned char stateT50 : 1;
    unsigned char stateT75 : 1;
    unsigned char stateT100 : 1;

    unsigned char stateDRY_RUN : 1;
} GlobalSensorCount;

char sw_val = 0;
unsigned int count_val = 0, resetMinutes, dispCount = 0, displayTankFlag;
unsigned long setting_ticks_100ms, state_ticks_100ms, pod_ticks_100ms;
unsigned long option_select_ticks_100ms;

unsigned int sumpLevel, tankLevel, motorRunningCount, prevSumpLevel = 50, prevTankLevel = 50, voltageFaultCount, motorRunTimeMinutes, sensorErrorCount, motorOffTime_val;
unsigned char motorState, dryRunState, motorOnIndicationFlag, motorOffInitFlag, dryRunResetInitFlag, sumpEmptyDetectFlag, tankEmptyDetectFlag, sumpFullTankNotFufllFlag;
unsigned char sump1EmptyDetectFlag, sump1FullDetectFlag, tankFullDetectFlag, motorOnInitFlag, displayRunTimeFlag, sensorErrorFlag, motorOffBySensorFlag;
unsigned char motorAlternateFlag, motor1FaultFlag, motor2FaultFlag;

float currentRMS = 0;
unsigned char runningMotor = 0;
unsigned int motor1CycleCounter = 0;
unsigned int motor2CycleCounter = 0;
unsigned char overCurrentFlag = 0;
unsigned char underCurrentFlag = 0;
unsigned long underCurrentStartTicks = 0, overCurrentStartTicks = 0;

// ALT Mode: Per-motor independent rest tracking
unsigned char motor1InRest = 0, motor2InRest = 0;
unsigned long motor1RestStartTicks = 0, motor2RestStartTicks = 0;

unsigned long buzzerTank_ticks_100ms, buzzerSump_ticks_100ms, motorElapsedTicks, motorResetTicks, voltageFaultTicks;
static unsigned char dryRunBuzzerActive = 0;
static unsigned long dryRunBuzzerStartTicks = 0;
static unsigned char lastDryRunMotor = 0;
extern float lineVoltage;

unsigned char switch_val = 0;
unsigned char auto_press_count = 0;
unsigned char reset_press_count = 0;
SYSTEM_STATES state = BOOT_START;
MOTOR_STATES motor_state = MOTOR_STATE_OFF;
WORKING_MODE work_mode = WORKING_MODE_AUTO;
void powerOnDelayCounter(void)
{
#ifdef MINIMUM_FUNCTIONALITY
    unsigned long total_pod_seconds = (unsigned long)GlobalSettings.powerOnDelaySeconds + 
                                     ((unsigned long)GlobalSettings.powerOnDelayMinutes * 60) + 
                                     ((unsigned long)GlobalSettings.powerOnDelayHours * 3600);
#else
    unsigned long total_pod_seconds = (unsigned long)GlobalSettings.powerOnDelay;
#endif
    
    // Power On Delay CountDown
    state_ticks_100ms = pod_ticks_100ms = getTicks100ms();
    while (getTicks100ms() - pod_ticks_100ms < (total_pod_seconds * 10))
    {
        if ((getTicks100ms() - state_ticks_100ms) > 10)
        {
            unsigned long elapsed_seconds = (getTicks100ms() - pod_ticks_100ms) / 10;
            unsigned long remaining_seconds = 0;
            if (total_pod_seconds > elapsed_seconds)
            {
                remaining_seconds = total_pod_seconds - elapsed_seconds;
            }
            
            state_ticks_100ms = getTicks100ms();
            
#ifdef MINIMUM_FUNCTIONALITY
            {
                unsigned int disp_val;
                const char *label;
                
                if (remaining_seconds >= 3600)
                {
                    label = "POH";
                    disp_val = (unsigned int)(remaining_seconds / 3600);
                }
                else if (remaining_seconds >= 60)
                {
                    label = "POM";
                    disp_val = (unsigned int)(remaining_seconds / 60);
                }
                else
                {
                    label = "POS";
                    disp_val = (unsigned int)remaining_seconds;
                }
                
                TM1637_WriteString(1, (char *)label);
                TM1637_WriteNumber(2, disp_val);
                printf("\r\nPOD Timeout: %s %d", label, disp_val);
            }
#else
            TM1637_WriteString(1, "POD");
            TM1637_WriteNumber(2, (unsigned int)remaining_seconds);
            printf("\r\nPOD Timeout: %ld", remaining_seconds);
#endif
        }
    }
    readVdd();
    readACVoltage();
}
#ifndef MINIMUM_FUNCTIONALITY
void displayDualSumpRolling(void)
{
    if (displayTankFlag == 0)
    {
        TM1637_WriteString(2, "SM1");
    }
    else if (displayTankFlag == 1)
    {
        if (GlobalSensorCount.stateS50)
            TM1637_WriteNumber(2, 100);
        else if (GlobalSensorCount.stateS25)
            TM1637_WriteNumber(2, 50);
        else
            TM1637_WriteNumber(2, 0);
    }
    else if (displayTankFlag == 2)
    {
        TM1637_WriteString(2, "SM2");
    }
    else if (displayTankFlag == 3)
    {
        if (GlobalSensorCount.stateS100)
            TM1637_WriteNumber(2, 100);
        else if (GlobalSensorCount.stateS75)
            TM1637_WriteNumber(2, 50);
        else
            TM1637_WriteNumber(2, 0);
    }
}
#endif

void refreshCurrentDisplay(void)
{
    unsigned char showAltDisplay = 0;
#ifdef MINIMUM_FUNCTIONALITY
    if (GlobalSettings.dualTankEnable == 1)
#else
    if (GlobalSettings.sumpType == 2 || GlobalSettings.dualTankEnable == 1)
#endif
    {
        showAltDisplay = (displayTankFlag == 4) ? 1 : ((displayTankFlag == 5) ? 2 : 0);
    }
    else
    {
        showAltDisplay = (displayTankFlag == 1) ? 1 : ((displayTankFlag == 2) ? 2 : 0);
    }

    if (state == MOTOR_OFF_OVERCURRENT_FAULT || state == MOTOR_OFF_DRY_RUN_FAULT)
    {
        if (showAltDisplay == 1) // Toggle between Tank and Reset display when in fault
        {
            if (state == MOTOR_OFF_DRY_RUN_FAULT)
            {
                if (dryRunBuzzerActive && lastDryRunMotor != 0)
                {
                    if (lastDryRunMotor == 1)
                        TM1637_WriteString(1, "RS1");
                    else if (lastDryRunMotor == 2)
                        TM1637_WriteString(1, "RS2");
                }
                else
                {
                    if (motor1FaultFlag && motor2FaultFlag)
                        TM1637_WriteString(1, "RST");
                    else if (motor1FaultFlag)
                        TM1637_WriteString(1, "RS1");
                    else if (motor2FaultFlag)
                        TM1637_WriteString(1, "RS2");
                    else
                        TM1637_WriteString(1, "RST");
                }
            }
            else // MOTOR_OFF_OVERCURRENT_FAULT
            {
#ifndef MINIMUM_FUNCTIONALITY
                if (GlobalSettings.dualMotorEnable && GlobalSettings.sumpType == 2)
                {
                    if (motor1FaultFlag && motor2FaultFlag)
                        TM1637_WriteString(1, "ALF"); // All Fault
                    else if (motor2FaultFlag)         // If motor 2 faulted
                        TM1637_WriteString(1, "M2F");
                    else if (motor1FaultFlag) // If motor 1 faulted
                        TM1637_WriteString(1, "M1F");
                    else
                        TM1637_WriteString(1, "RST");
                }
                else
#endif
                {
                    TM1637_WriteString(1, "RST");
                }
            }
            TM1637_WriteNumber(2, resetMinutes);
        }
        else
        {
            TM1637_WriteNumber(1, tankLevel);
#ifndef MINIMUM_FUNCTIONALITY
            if (GlobalSettings.sumpType == 0)
            {
                TM1637_WriteString(2, "BOR");
            }
            else if (GlobalSettings.sumpType == 2)
            {
                displayDualSumpRolling();
            }
            else
#endif
            {
#ifndef MINIMUM_FUNCTIONALITY
                if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ACO || GlobalSettings.motorMode == MOTOR_MODE_BOR) && runningMotor == 2)
                {
                    TM1637_WriteString(2, "BOR");
                }
                else
#endif
                {
                    TM1637_WriteNumber(2, sumpLevel);
                }
            }
        }
    }
    else if (state == MOTOR_OFF_VOLTAGE_FAULT)
    {
        TM1637_WriteString(1, "VOL");
        TM1637_WriteNumber(2, getLineVoltage());
    }
    else
    {
        if (sensorErrorCount > 50)
        {
            TM1637_WriteString(1, "SEN");
            TM1637_WriteString(2, "ERR");
        }
        else
        {
            if (GlobalSettings.dualTankEnable == 0)
            {
                if (state != MOTOR_OFF_RESET_DELAY)
                {
                    if (showAltDisplay == 0) // Only display tank level in normal mode to prevent flickering
                        TM1637_WriteNumber(1, tankLevel);
                }

                if (motorState)
                {
                    if (showAltDisplay == 2)
                    {
                        unsigned int val = (unsigned int)(currentRMS / 100.0);
                        if (val > 999)
                            val = 999;
                        TM1637_WriteString(1, "AMP");
                        TM1637_WriteNumberCurrent(2, val);
                    }
                    else if (showAltDisplay == 1)
                    {
                        if (dryRunBuzzerActive && lastDryRunMotor != 0)
                        {
                            if (lastDryRunMotor == 1)
                                TM1637_WriteString(1, "RS1");
                            else if (lastDryRunMotor == 2)
                                TM1637_WriteString(1, "RS2");
                        }
                        else
                        {
#ifndef MINIMUM_FUNCTIONALITY
                            if (GlobalSettings.dualMotorEnable)
                            {
                                if (GlobalSettings.motorMode == MOTOR_MODE_COM)
                                {
                                    if (runningMotor == 2)
                                        TM1637_WriteString(1, "BTS");
                                    else
                                        TM1637_WriteString(1, "STT");
                                }
                                else if (GlobalSettings.motorMode == MOTOR_MODE_ALT) // ALT
                                {
                                    if (runningMotor == 2)
                                        TM1637_WriteString(1, "M2T");
                                    else
                                        TM1637_WriteString(1, "M1T");
                                }
                                else // ACO, SUM, BOR
                                {
                                    if (runningMotor == 2)
                                        TM1637_WriteString(1, "MO2");
                                    else
                                        TM1637_WriteString(1, "MO1");
                                }
                            }
                            else
#endif
                            {
                                // Clear top line if single motor to make it look clean if needed or show tank level
                                TM1637_WriteNumber(1, tankLevel);
                            }
                        }
                        TM1637_WriteNumberDP(2, motorRunTimeMinutes);
                    }
                    else
                    {
#ifndef MINIMUM_FUNCTIONALITY
                        if (GlobalSettings.sumpType == 0)
                        {
                            TM1637_WriteString(2, "BOR");
                        }
                        else if (GlobalSettings.sumpType == 2)
                        {
                            displayDualSumpRolling();
                        }
                        else
#endif
                        {
#ifndef MINIMUM_FUNCTIONALITY
                            if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ACO || GlobalSettings.motorMode == MOTOR_MODE_BOR) && runningMotor == 2)
                            {
                                TM1637_WriteString(2, "BOR");
                            }
                            else
#endif
                            {
                                TM1637_WriteNumber(2, sumpLevel);
                            }
                        }
                    }
                }
                else
                {
                    if (state == MOTOR_OFF_RESET_DELAY)
                    {
                        TM1637_WriteString(1, "OFF");
                        TM1637_WriteNumberDP(2, getRemainingOffTime());
                    }
                    else if (showAltDisplay == 1)
                    {
                        TM1637_WriteString(1, "MOT");
                        TM1637_WriteNumberDP(2, motorRunTimeMinutes);
                    }
                    else
                    {
#ifndef MINIMUM_FUNCTIONALITY
                        if (GlobalSettings.sumpType == 0)
                        {
                            TM1637_WriteString(2, "BOR");
                        }
                        else if (GlobalSettings.sumpType == 2)
                        {
                            displayDualSumpRolling();
                        }
                        else
#endif
                        {
#ifndef MINIMUM_FUNCTIONALITY
                            if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ACO || GlobalSettings.motorMode == MOTOR_MODE_BOR) && runningMotor == 2)
                            {
                                TM1637_WriteString(2, "BOR");
                            }
                            else
#endif
                            {
                                TM1637_WriteNumber(2, sumpLevel);
                            }
                        }
                    }
                }
            }
            else
            {
                // Dual Tank Logic
                if (showAltDisplay == 0)
                {
                    if (displayTankFlag == 0) // Display Tank1 Level
                    {
                        TM1637_WriteString(1, "TN1");
                    }
                    else if (displayTankFlag == 1) // Display Tank1 Level
                    {
                        if (GlobalSensorCount.stateT50)
                            TM1637_WriteNumber(1, 100);
                        else if (GlobalSensorCount.stateT25)
                            TM1637_WriteNumber(1, 50);
                        else
                            TM1637_WriteNumber(1, 0);
                    }
                    else if (displayTankFlag == 2) // Display Tank2 Level
                    {
                        TM1637_WriteString(1, "TN2");
                    }
                    else if (displayTankFlag == 3) // Display Tank2 Level
                    {
                        if (GlobalSensorCount.stateT100)
                            TM1637_WriteNumber(1, 100);
                        else if (GlobalSensorCount.stateT75)
                            TM1637_WriteNumber(1, 50);
                        else
                            TM1637_WriteNumber(1, 0);
                    }

#ifndef MINIMUM_FUNCTIONALITY
                    if (GlobalSettings.sumpType == 0)
                    {
                        TM1637_WriteString(2, "BOR");
                    }
                    else if (GlobalSettings.sumpType == 2)
                    {
                        displayDualSumpRolling();
                    }
                    else
#endif
                    {
#ifndef MINIMUM_FUNCTIONALITY
                        if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ACO || GlobalSettings.motorMode == MOTOR_MODE_BOR) && runningMotor == 2)
                        {
                            TM1637_WriteString(2, "BOR");
                        }
                        else
#endif
                        {
                            TM1637_WriteNumber(2, sumpLevel);
                        }
                    }
                }
                else if (showAltDisplay == 1)
                {
                    if (motorState)
                    {
                        if (dryRunBuzzerActive && lastDryRunMotor != 0)
                        {
                            if (lastDryRunMotor == 1)
                                TM1637_WriteString(1, "RS1");
                            else if (lastDryRunMotor == 2)
                                TM1637_WriteString(1, "RS2");
                        }
                        else
                        {
#ifndef MINIMUM_FUNCTIONALITY
                            if (GlobalSettings.dualMotorEnable)
                            {
                                if (GlobalSettings.motorMode == MOTOR_MODE_COM)
                                {
                                    if (runningMotor == 2)
                                        TM1637_WriteString(1, "BTS");
                                    else
                                        TM1637_WriteString(1, "STT");
                                }
                                else if (GlobalSettings.motorMode == MOTOR_MODE_ALT) // ALT
                                {
                                    if (runningMotor == 2)
                                        TM1637_WriteString(1, "M2T");
                                    else
                                        TM1637_WriteString(1, "M1T");
                                }
                                else // ACO, SUM, BOR
                                {
                                    if (runningMotor == 2)
                                        TM1637_WriteString(1, "MO2");
                                    else
                                        TM1637_WriteString(1, "MO1");
                                }
                            }
                            else
#endif
                            {
                                TM1637_WriteString(1, "MOT");
                            }
                        }
                        TM1637_WriteNumberDP(2, motorRunTimeMinutes);
                    }
                    else
                    {
                        if (state == MOTOR_OFF_RESET_DELAY)
                        {
                            TM1637_WriteString(1, "OFF");
                            TM1637_WriteNumberDP(2, getRemainingOffTime());
                        }
                        else
                        {
                            TM1637_WriteString(1, "MOT");
                            TM1637_WriteNumberDP(2, motorRunTimeMinutes);
                        }
                    }
                }
                else if (showAltDisplay == 2)
                {
                    unsigned int val = (unsigned int)(currentRMS / 100.0);
                    if (val > 999)
                        val = 999;
                    TM1637_WriteString(1, "AMP");
                    TM1637_WriteNumberCurrent(2, val);
                }
            }
        }
    }
}

void handleDisplayRolling(void)
{
#ifdef MINIMUM_FUNCTIONALITY
    if (GlobalSettings.dualTankEnable == 1)
#else
    if (GlobalSettings.sumpType == 2 || GlobalSettings.dualTankEnable == 1)
#endif
    {
        unsigned char max_flag = (GlobalSettings.ctEnable && motorState) ? 5 : 4;
        displayTankFlag++;
        if (displayTankFlag > max_flag)
        {
            displayTankFlag = 0;
        }
    }
    else
    {
        unsigned char max_flag = (GlobalSettings.ctEnable && motorState) ? 2 : 1;
        displayTankFlag++;
        if (displayTankFlag > max_flag)
        {
            displayTankFlag = 0;
        }
    }
}

void updateSensorLED(void)
{
    sumpLevel = 0;
    tankLevel = 0;
    GlobalSensorCount.stateS25 = GlobalSensorCount.stateS50 = GlobalSensorCount.stateS75 = GlobalSensorCount.stateS100 = GlobalSensorCount.stateDRY_RUN = 0;
    GlobalSensorCount.stateT25 = GlobalSensorCount.stateT50 = GlobalSensorCount.stateT75 = GlobalSensorCount.stateT100 = 0;
    sensorErrorFlag = 0;

    if (!GlobalSettings.ctEnable)
    {
        dryRunState = 1;
    }
    else
    {
        dryRunState = 0;
    }
    ENABLE_TIMER0_MODE1; /* Timer 0 mode configuration */
    TIMER0_FSYS_DIV12;
    TH0 = TH0_INIT;
    TL0 = TL0_INIT;
    TF0 = 0;
    set_TCON_TR0; /* Timer0 run */
    while (TF0 == 0)
    {
        if (S_SUMP_25 == 0)
        {
            GlobalSensorCount.stateS25 = 1;
        }
        if (S_SUMP_50 == 0)
        {
            GlobalSensorCount.stateS50 = 1;
        }
        if (S_SUMP_75 == 0)
        {
            GlobalSensorCount.stateS75 = 1;
        }
        if (S_SUMP_100 == 0)
        {
            GlobalSensorCount.stateS100 = 1;
        }

        if (S_TANK_25 == 0)
        {
            GlobalSensorCount.stateT25 = 1;
        }
        if (S_TANK_50 == 0)
        {
            GlobalSensorCount.stateT50 = 1;
        }
        if (S_TANK_75 == 0)
        {
            GlobalSensorCount.stateT75 = 1;
        }
        if (S_TANK_100 == 0)
        {
            GlobalSensorCount.stateT100 = 1;
        }

        if (S_DRY_RUN == 0)
        {
            GlobalSensorCount.stateDRY_RUN = 1; // Water detected
        }
    }

    // CT Logic
    if (motorState)
    {
        unsigned char motor_channel = 1; // Default to motor 1

#ifndef MINIMUM_FUNCTIONALITY
        if (GlobalSettings.dualMotorEnable)
        {
            if (O_RLY_2 || O_RLY_3)
            {
                motor_channel = 2;
            }
        }
#endif

        currentRMS = readMotorCurrent();

        if (GlobalSettings.ctEnable)
        {
#ifdef MINIMUM_FUNCTIONALITY
            float ref = GlobalSettings.referenceCurrent1;
#else
            float ref = (motor_channel == 2) ? GlobalSettings.referenceCurrent2 : GlobalSettings.referenceCurrent1;
#endif
            if (ref > 0)
            {
                float buffer = ref * 0.20f; // 20%

                // Overcurrent Detection
                if (currentRMS > (ref + buffer))
                {
                    if (overCurrentStartTicks == 0)
                        overCurrentStartTicks = getTicks100ms();
                    else if ((getTicks100ms() - overCurrentStartTicks) >= 50) // 5 seconds
                    {
                        overCurrentFlag = 1;
                        dryRunState = 0;
                    }
                }
                else
                {
                    overCurrentStartTicks = 0;
                    overCurrentFlag = 0;
                }

                // Undercurrent (Dry Run) Detection
                if (currentRMS < (ref - buffer))
                {
                    if (underCurrentStartTicks == 0)
                        underCurrentStartTicks = getTicks100ms();
                    else if ((getTicks100ms() - underCurrentStartTicks) >= 50) // 5 seconds
                    {
                        dryRunState = 1;
                        overCurrentFlag = 0;
                    }
                }
                else
                {
                    underCurrentStartTicks = 0;
                    if (!overCurrentFlag)
                        dryRunState = 0;
                }
            }
            else
            {
                dryRunState = 0;
                overCurrentFlag = 0;
                underCurrentStartTicks = 0;
                overCurrentStartTicks = 0;
            }
        }
    }
    else
    {
        currentRMS = 0;
        overCurrentFlag = 0;
        underCurrentStartTicks = 0;
        overCurrentStartTicks = 0;
    }

    clr_TCON_TR0;

    if (!GlobalSettings.ctEnable)
    {
        if (GlobalSensorCount.stateDRY_RUN)
            dryRunState = 0; // Water found, no dry run
        else
            dryRunState = 1; // No water found, dry run
    }
    if (GlobalSensorCount.stateS25)
    {
        sumpLevel = 25;
    }
    if (GlobalSensorCount.stateS50)
    {
        sumpLevel = 50;
    }
    if (GlobalSensorCount.stateS75)
    {
        sumpLevel = 75;
    }
    if (GlobalSensorCount.stateS100)
    {
        sumpLevel = 100;
    }
    if (GlobalSensorCount.stateT25)
    {
        tankLevel = 25;
    }
    if (GlobalSensorCount.stateT50)
    {
        tankLevel = 50;
    }
    if (GlobalSensorCount.stateT75)
    {
        tankLevel = 75;
    }
    if (GlobalSensorCount.stateT100)
    {
        tankLevel = 100;
    }
    if (GlobalSettings.dualTankEnable == 1)
    {
        // Dual Tank specific error checking
        if (tankLevel != 0)
        {
            if ((GlobalSensorCount.stateT50 == 1 && GlobalSensorCount.stateT25 == 0) ||
                (GlobalSensorCount.stateT100 == 1 && GlobalSensorCount.stateT75 == 0))
            {
                sensorErrorFlag = 1;
            }
        }
    }
    else
    {
        // Existing single tank proportional error checking
        if (tankLevel >= GlobalSettings.tankOnLevel && (tankLevel != 0))
        {
            // Require the specific tankOnLevel sensor to be ON if tankLevel shows water has reached or exceeded it
            if ((GlobalSettings.tankOnLevel == 25 && GlobalSensorCount.stateT25 == 0) ||
                (GlobalSettings.tankOnLevel == 50 && GlobalSensorCount.stateT50 == 0) ||
                (GlobalSettings.tankOnLevel == 75 && GlobalSensorCount.stateT75 == 0))
            {
                sensorErrorFlag = 1;
            }
        }

        if (GlobalSettings.tankOffLevel <= tankLevel && (tankLevel != 0))
        {
            if (GlobalSensorCount.stateT25 + GlobalSensorCount.stateT50 + GlobalSensorCount.stateT75 + GlobalSensorCount.stateT100 < 2)
            {
                sensorErrorFlag = 1;
            }
        }
    }

#ifndef MINIMUM_FUNCTIONALITY
    if (GlobalSettings.sumpType == 0)
    {
        sumpLevel = 50;
    }
    else if (GlobalSettings.sumpType == 1)
    {
        // Dual motor is generally not used with single sump logic in same context, but ensure standard logic
        if (GlobalSensorCount.stateS25 + GlobalSensorCount.stateS50 + GlobalSensorCount.stateS75 + GlobalSensorCount.stateS100 < 1)
        {
            // Do not flag sensorErrorFlag here, the sump code uses dry run logic.
        }
    }
    else if (GlobalSettings.sumpType == 2)
    {
        // Dual Sump Logic:
        // Run motor if EITHER sump has water. Motor logic operates off `sumpLevel`.
        // We set sumpLevel=50 if at least one Sump has water (S25 or S75).
        // Otherwise, if both are dry, set sumpLevel=0.
        if (GlobalSensorCount.stateS25 || GlobalSensorCount.stateS75)
        {
            sumpLevel = 50;
        }
        else
        {
            sumpLevel = 0;
        }
    }
#endif

#ifdef ENABLE_SUMP_BUZZER
    if (prevSumpLevel != sumpLevel)
    {
        if (sumpLevel == 0)
        {
            sump1EmptyDetectFlag = 1;
            buzzerSump_ticks_100ms = getTicks100ms();
        }
        else if (sumpLevel == 100)
        {
            sump1FullDetectFlag = 1;
            buzzerSump_ticks_100ms = getTicks100ms();
        }
        if (sumpLevel > 0)
        {
            sump1EmptyDetectFlag = 0;
        }
        if (sumpLevel < 100)
        {
            sump1FullDetectFlag = 0;
        }
        prevSumpLevel = sumpLevel;
    }
#endif
#ifdef ENABLE_TANK_BUZZER
    if (prevTankLevel != tankLevel)
    {
        if (tankLevel == 0)
        {
            tankEmptyDetectFlag = 1;
            buzzerTank_ticks_100ms = getTicks100ms();
        }
        else if (tankLevel == 100)
        {
            tankFullDetectFlag = 1;
            buzzerTank_ticks_100ms = getTicks100ms();
        }
        if (tankLevel > 0)
        {
            tankEmptyDetectFlag = 0;
        }
        if (tankLevel < 100)
        {
            tankFullDetectFlag = 0;
        }
        prevTankLevel = tankLevel;
    }
#endif

    if (sensorErrorFlag)
    {
        sensorErrorCount++;
        if (sensorErrorCount > 50)
        {
            state = MOTOR_OFF_SENSOR_ERROR;
        }
    }

    // Sump low-level relay: Single motor + Single sump only
    // O_RLY_2 is unused by motor control in this config, repurpose as sump indicator
    if (!GlobalSettings.dualMotorEnable && GlobalSettings.sumpType == 1)
    {
        if (sumpLevel < 50)
            O_RLY_2 = 1; // Relay ON (active low) — sump low
        else
            O_RLY_2 = 0; // Relay OFF — sump OK
    }
}

void flowLEDsMotorOff(void)
{
    L_FLOW_1 = L_FLOW_2 = L_FLOW_3 = 0;
}
void flowLEDsMotorDryRunCheck(void)
{
    motorOnIndicationFlag = !motorOnIndicationFlag;

    if (motorOnIndicationFlag)
    {
        if (state == MOTOR_ON_SECONDARY_DRY_CHECK)
        {
            L_FLOW_1 = L_FLOW_2 = L_FLOW_3 = 1;
        }
        else if (state == MOTOR_ON_PRIMARY_DRY_RUN_CHECK)
        {
            motorRunningCount++;
            if (motorRunningCount > 1)
            {
                motorRunningCount = 0;
                L_FLOW_1 = L_FLOW_2 = L_FLOW_3 = 1;
            }
        }
    }
    else
    {
        L_FLOW_1 = L_FLOW_2 = L_FLOW_3 = 0;
    }
    // L_DRY_RUN = 0;
}
void flowLEDsDryRunFault(void)
{
    L_FLOW_1 = L_FLOW_2 = L_FLOW_3 = 0;
    // L_DRY_RUN = 1;
    L_MOT1_ON = !L_MOT1_ON;
}
void flowLEDsMotorOn(void)
{
    motorRunningCount++;
    if (motorRunningCount > 3)
    {
        motorRunningCount = 0;
    }
    L_FLOW_1 = L_FLOW_2 = L_FLOW_3 = 0;
    // L_DRY_RUN = 0;
    switch (motorRunningCount)
    {
    case 0:
        L_FLOW_1 = 1;
        break;
    case 1:
        L_FLOW_2 = 1;
        break;
    case 2:
        L_FLOW_3 = 1;
        break;
    }
}
void allLEDsOFF(void)
{
    L_FLOW_1 = 0;
    L_FLOW_2 = 0;
    L_FLOW_3 = 0;
}
void allLEDsON(void)
{
    L_FLOW_1 = 1;
    L_FLOW_2 = 1;
    L_FLOW_3 = 1;
}

typedef enum
{
    DISP_NUMERIC,
    DISP_ENA_DIS,
    DISP_DIS_ENA,
    DISP_OLD_ELE,
    DISP_SUMP_MODE,
    DISP_AMP,
    DISP_MOTOR_MODE,
    DISP_TRAFO
} ParameterDisplayType;

static float editFloatParameter(const char *label, float current_val, float min_val, float max_val, float up_step, float down_step, ParameterDisplayType dispType)
{
    float count_val;
    allLEDsOFF();
    setting_ticks_100ms = getTicks100ms();
    while ((getTicks100ms() - setting_ticks_100ms) < 20)
        ;

    if (GlobalSettings.state)
    {
        printf("\r\nSV %s ", label);
        TM1637_WriteString(1, (char *)label);
        if (dispType == DISP_AMP) // Changed to if, as else if without preceding else is invalid
        {
            unsigned int int_val = (unsigned int)(current_val * 10.0);
            TM1637_WriteNumberCurrent(2, int_val);
            printf("%0.1f", current_val);
        }
    }
    count_val = current_val;
    option_select_ticks_100ms = getTicks100ms();

    while ((getTicks100ms() - option_select_ticks_100ms) < SETTINGS_SELECT_TIMEOUT)
    {
        if ((getTicks100ms() - setting_ticks_100ms) > 1) // basic debounce
        {
            setting_ticks_100ms = getTicks100ms();
            sw_val = readButton();
            if (sw_val == 1) // UP
            {
                option_select_ticks_100ms = getTicks100ms();
                count_val += up_step;
                if (count_val > max_val)
                {
                    count_val = (up_step == 1 && min_val == 0) ? 0 : max_val;
                }
            }
            else if (sw_val == 2) // OK / NEXT
            {
                break;
            }
            else if (sw_val == 3) // DOWN
            {
                option_select_ticks_100ms = getTicks100ms();
                if (count_val >= down_step)
                {
                    count_val -= down_step;
                }
                else
                {
                    count_val = 0;
                }

                if (count_val < min_val)
                {
                    count_val = min_val;
                }
            }

            if (sw_val == 1 || sw_val == 3)
            {
                printf("\r\n%s ", label);
                TM1637_WriteString(1, (char *)label);
                if (dispType == DISP_AMP) // Changed to if, as else if without preceding else is invalid
                {
                    unsigned int int_val = (unsigned int)(count_val * 10.0);
                    TM1637_WriteNumberCurrent(2, int_val);
                    printf("%0.1f", count_val);
                }
            }
        }
    }

    TM1637_WriteString(2, "SAV");
    return count_val;
}

static unsigned int editParameter(const char *label, unsigned int current_val, unsigned int min_val, unsigned int max_val, unsigned int up_step, unsigned int down_step, ParameterDisplayType dispType)
{
    unsigned int count_val;
    unsigned long start_hold_ticks = 0;
    unsigned int effective_up_step;
    unsigned int effective_down_step;
    allLEDsOFF();
    setting_ticks_100ms = getTicks100ms();
    while ((getTicks100ms() - setting_ticks_100ms) < 20)
        ;

    if (GlobalSettings.state)
    {
        printf("\r\nSV %s ", label);
        TM1637_WriteString(1, (char *)label);
        if (dispType == DISP_NUMERIC)
        {
            TM1637_WriteNumber(2, current_val);
            printf("%d", current_val);
        }
        else if (dispType == DISP_ENA_DIS)
        {
            TM1637_WriteString(2, current_val ? "ENA" : "DIS");
            printf("%s", current_val ? "Enabled" : "Disabled");
        }
        else if (dispType == DISP_DIS_ENA)
        {
            TM1637_WriteString(2, current_val ? "DIS" : "ENA");
            printf("%s", current_val ? "Disabled" : "Enabled");
        }
        else if (dispType == DISP_OLD_ELE)
        {
            TM1637_WriteString(2, current_val ? "ELE" : "OLD");
            printf("%s", current_val ? "ELECTRONIC" : "OLD");
        }
        else if (dispType == DISP_SUMP_MODE)
        {
            if (current_val == 0)
            {
                TM1637_WriteString(2, "BOR");
                printf("BOR");
            }
            else
            {
                TM1637_WriteNumber(2, current_val);
                printf("%d", current_val);
            }
        }
        else if (dispType == DISP_MOTOR_MODE)
        {
            if (current_val == 0)
            {
                TM1637_WriteString(2, "ACO");
                printf("ACO");
            }
            else if (current_val == 1)
            {
                TM1637_WriteString(2, "ALT");
                printf("ALT");
            }
            else if (current_val == 2)
            {
                TM1637_WriteString(2, "SUM");
                printf("SUM");
            }
            else if (current_val == 3)
            {
                TM1637_WriteString(2, "BOR");
                printf("BO");
            }
            else if (current_val == 4)
            {
                TM1637_WriteString(2, "COB");
                printf("COB");
            }
            else if (current_val == 5)
            {
                TM1637_WriteString(2, "ACC");
                printf("ACC");
            }
            else if (current_val == 6)
            {
                TM1637_WriteString(2, "SAL");
                printf("SAL");
            }
            else
            {
                TM1637_WriteString(2, "SUM");
                printf("SUM");
            }
        }
        else if (dispType == DISP_ENA_DIS) // Added this based on the instruction
        {
            TM1637_WriteString(2, current_val ? "ENA" : "DIS");
            printf("%s", current_val ? "Enabled" : "Disabled");
        }
        else if (dispType == DISP_TRAFO)
        {
            TM1637_WriteString(2, current_val ? "DUL" : "CTP");
            printf("%s", current_val ? "12-0-12-0" : "12-0-12");
        }
    }
    count_val = current_val;
    option_select_ticks_100ms = getTicks100ms();

    while ((getTicks100ms() - option_select_ticks_100ms) < SETTINGS_SELECT_TIMEOUT)
    {
        if ((getTicks100ms() - setting_ticks_100ms) > 1) // basic debounce
        {
            setting_ticks_100ms = getTicks100ms();
            sw_val = readButton();
            if (sw_val == 1 || sw_val == 3) // UP or DOWN
            {
                if (start_hold_ticks == 0)
                    start_hold_ticks = getTicks100ms();

                if (dispType == DISP_NUMERIC && (getTicks100ms() - start_hold_ticks) >= 30) // 3 seconds (30 * 100ms)
                {
                    effective_up_step = 10;
                    effective_down_step = 10;
                }
                else
                {
                    effective_up_step = up_step;
                    effective_down_step = down_step;
                }

                option_select_ticks_100ms = getTicks100ms();
                if (sw_val == 1) // UP
                {
                    if (dispType == DISP_ENA_DIS || dispType == DISP_DIS_ENA || dispType == DISP_OLD_ELE)
                    {
                        count_val = 1;
                    }
                    else
                    {
                        count_val += effective_up_step;
                        if (count_val > max_val)
                        {
                            count_val = (up_step == 1 && min_val == 0) ? 0 : max_val;
                        }
                    }
                }
                else if (sw_val == 3) // DOWN
                {
                    if (dispType == DISP_ENA_DIS || dispType == DISP_DIS_ENA || dispType == DISP_OLD_ELE)
                    {
                        count_val = 0;
                    }
                    else
                    {
                        if (count_val >= effective_down_step)
                        {
                            count_val -= effective_down_step;
                        }
                        else
                        {
                            count_val = 0;
                        }

                        if (count_val < min_val)
                        {
                            count_val = min_val;
                        }
                    }
                }
            }
            else if (sw_val == 2) // OK / NEXT
            {
                break;
            }
            else
            {
                start_hold_ticks = 0;
            }

            if (sw_val == 1 || sw_val == 3)
            {
                printf("\r\n%s ", label);
                TM1637_WriteString(1, (char *)label);
                if (dispType == DISP_NUMERIC)
                {
                    TM1637_WriteNumber(2, count_val);
                    printf("%d", count_val);
                }
                else if (dispType == DISP_ENA_DIS)
                {
                    TM1637_WriteString(2, count_val ? "ENA" : "DIS");
                    printf("%s", count_val ? "Enabled" : "Disabled");
                }
                else if (dispType == DISP_DIS_ENA)
                {
                    TM1637_WriteString(2, count_val ? "DIS" : "ENA");
                    printf("%s", count_val ? "Disabled" : "Enabled");
                }
                else if (dispType == DISP_OLD_ELE)
                {
                    TM1637_WriteString(2, count_val ? "ELE" : "OLD");
                    printf("%s", count_val ? "ELECTRONIC" : "OLD");
                }
                else if (dispType == DISP_SUMP_MODE)
                {
                    if (count_val == 0)
                    {
                        TM1637_WriteString(2, "BOR");
                        printf("BOR");
                    }
                    else if (count_val == 1)
                    {
                        TM1637_WriteNumber(2, count_val);
                        printf("1");
                    }
                    else if (count_val == 2)
                    {
                        TM1637_WriteNumber(2, count_val);
                        printf("2");
                    }
                    else
                    {
                        TM1637_WriteNumber(2, count_val);
                        printf("%d", count_val);
                    }
                }
                else if (dispType == DISP_MOTOR_MODE)
                {
                    if (count_val == 0)
                    {
                        TM1637_WriteString(2, "ACO");
                        printf("ACO");
                    }
                    else if (count_val == 1)
                    {
                        TM1637_WriteString(2, "ALT");
                        printf("ALT");
                    }
                    else if (count_val == 2)
                    {
                        TM1637_WriteString(2, "SUM");
                        printf("SO");
                    }
                    else if (count_val == 3)
                    {
                        TM1637_WriteString(2, "BOR");
                        printf("BO");
                    }
                    else if (count_val == 4)
                    {
                        TM1637_WriteString(2, "COB");
                        printf("COB");
                    }
                    else if (count_val == 5)
                    {
                        TM1637_WriteString(2, "ACC");
                        printf("ACC");
                    }
                    else if (count_val == 6)
                    {
                        TM1637_WriteString(2, "SAL");
                        printf("SAL");
                    }
                    else
                    {
                        TM1637_WriteString(2, "SUM");
                        printf("SUM");
                    }
                }
                else if (dispType == DISP_ENA_DIS)
                {
                    TM1637_WriteString(2, count_val ? "ENA" : "DIS");
                    printf("%s", count_val ? "Enabled" : "Disabled");
                }
                else if (dispType == DISP_TRAFO)
                {
                    TM1637_WriteString(2, count_val ? "DUL" : "CTP");
                    printf("%s", count_val ? "12-0-12-0" : "12-0-12");
                }
            }
        }
    }

    TM1637_WriteString(2, "SAV");
    return count_val;
}

void configSettings(void)
{
    unsigned int min_tank_off;
    allLEDsOFF();
    setting_ticks_100ms = getTicks100ms();
    while ((getTicks100ms() - setting_ticks_100ms) < 20)
        ;

#ifdef MINIMUM_FUNCTIONALITY
    GlobalSettings.powerOnDelaySeconds = editParameter("POS", GlobalSettings.powerOnDelaySeconds, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.powerOnDelayMinutes = editParameter("POM", GlobalSettings.powerOnDelayMinutes, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.powerOnDelayHours = editParameter("POH", GlobalSettings.powerOnDelayHours, 0, 999, 1, 1, DISP_NUMERIC);
#else
    GlobalSettings.powerOnDelay = editParameter("POD", GlobalSettings.powerOnDelay, 0, 999, 1, 1, DISP_NUMERIC);
#endif
    GlobalSettings.dryRunTimeOut = editParameter("DR1", GlobalSettings.dryRunTimeOut, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.secondaryDryRunDuration = editParameter("DR2", GlobalSettings.secondaryDryRunDuration, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.dryRunReset = editParameter("RST", GlobalSettings.dryRunReset, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.tankOnLevel = editParameter("TON", GlobalSettings.tankOnLevel, 25, 75, 25, 25, DISP_NUMERIC);

    min_tank_off = GlobalSettings.tankOnLevel + 25;
    GlobalSettings.tankOffLevel = editParameter("TOF", GlobalSettings.tankOffLevel, min_tank_off, 100, 25, 25, DISP_NUMERIC);
    GlobalSettings.dualTankEnable = editParameter("DTK", GlobalSettings.dualTankEnable, 0, 1, 1, 1, DISP_ENA_DIS);
#ifndef MINIMUM_FUNCTIONALITY
    GlobalSettings.sumpType = editParameter("SUM", GlobalSettings.sumpType, 0, 2, 1, 1, DISP_SUMP_MODE);
#endif
    GlobalSettings.motorOnTime1 = editParameter("MO1", GlobalSettings.motorOnTime1, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.motorOffTime1 = editParameter("MF1", GlobalSettings.motorOffTime1, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.voltageLimitEnable = editParameter("VLT", GlobalSettings.voltageLimitEnable, 0, 1, 1, 1, DISP_ENA_DIS);
    if (GlobalSettings.voltageLimitEnable == 1)
    {
        GlobalSettings.minACVolt = editParameter("VOL", GlobalSettings.minACVolt, 110, 180, 1, 1, DISP_NUMERIC);
        GlobalSettings.maxACVolt = editParameter("VOH", GlobalSettings.maxACVolt, 270, 300, 1, 1, DISP_NUMERIC);
    }
    GlobalSettings.transformerType = editParameter("TRF", GlobalSettings.transformerType, 0, 1, 1, 1, DISP_TRAFO);
    GlobalSettings.offTimeTankOffLevel = editParameter("OFF", GlobalSettings.offTimeTankOffLevel, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.relayPulse = editParameter("RPL", GlobalSettings.relayPulse, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.starterSelection = editParameter("STA", GlobalSettings.starterSelection, 0, 1, 1, 1, DISP_OLD_ELE);
#ifndef MINIMUM_FUNCTIONALITY
    GlobalSettings.dualMotorEnable = editParameter("DM", GlobalSettings.dualMotorEnable, 0, 1, 1, 1, DISP_ENA_DIS);
    if (GlobalSettings.dualMotorEnable == 1)
    {
        GlobalSettings.motorOnTime2 = editParameter("MO2", GlobalSettings.motorOnTime2, 0, 999, 1, 1, DISP_NUMERIC);
        GlobalSettings.motorOffTime2 = editParameter("MF2", GlobalSettings.motorOffTime2, 0, 999, 1, 1, DISP_NUMERIC);
        GlobalSettings.motorMode = editParameter("MOD", GlobalSettings.motorMode, 0, 6, 1, 1, DISP_MOTOR_MODE);
        if (GlobalSettings.motorMode == MOTOR_MODE_ACC)
        {
            GlobalSettings.motor1Cycles = editParameter("CY1", GlobalSettings.motor1Cycles, 0, 999, 1, 1, DISP_NUMERIC);
            GlobalSettings.motor2Cycles = editParameter("CY2", GlobalSettings.motor2Cycles, 0, 999, 1, 1, DISP_NUMERIC);
        }
    }
#endif
    GlobalSettings.ctEnable = editParameter("CTE", GlobalSettings.ctEnable, 0, 1, 1, 1, DISP_ENA_DIS);
    if (GlobalSettings.ctEnable)
    {
#ifndef MINIMUM_FUNCTIONALITY
        if (GlobalSettings.dualMotorEnable)
        {
            GlobalSettings.referenceCurrent1 = editFloatParameter("AM1", GlobalSettings.referenceCurrent1, 0.0f, 30.0f, 0.1f, 0.1f, DISP_AMP);
            GlobalSettings.referenceCurrent2 = editFloatParameter("AM2", GlobalSettings.referenceCurrent2, 0.0f, 30.0f, 0.1f, 0.1f, DISP_AMP);
        }
        else
#endif
        {
            GlobalSettings.referenceCurrent1 = editFloatParameter("AMP", GlobalSettings.referenceCurrent1, 0.0f, 30.0f, 0.1f, 0.1f, DISP_AMP);
        }
    }

    writeSettingEEPROM();
}

void readSettingsEEPROM(void)
{
    Read_DATAFLASH_ARRAY(0x7F00, (unsigned char *)&GlobalSettings, sizeof(GlobalSettings)); // read structure

    printf("\r\nST: %d", GlobalSettings.state);
    printf("\r\nST2: %d", GlobalSettings.state2);

    if ((GlobalSettings.state == 1) && (GlobalSettings.state2 == 84))
    {
        printf("\r\nDryRunTime: %d", GlobalSettings.dryRunTimeOut);
        printf("\r\nDryRunRST: %d", GlobalSettings.dryRunReset);
        printf("\r\n2ndDryRun: %d", GlobalSettings.secondaryDryRunDuration);
#ifdef MINIMUM_FUNCTIONALITY
        printf("\r\nPOD S/M/H: %d/%d/%d", GlobalSettings.powerOnDelaySeconds, GlobalSettings.powerOnDelayMinutes, GlobalSettings.powerOnDelayHours);
#else
        printf("\r\nPower On Delay: %d", GlobalSettings.powerOnDelay);
#endif
        printf("\r\nTank On Level: %d", GlobalSettings.tankOnLevel);
        printf("\r\nTank Off Level: %d", GlobalSettings.tankOffLevel);
        printf("\r\nMotor 1 On Time: %d", GlobalSettings.motorOnTime1);
        printf("\r\nMotor 1 Off Time: %d", GlobalSettings.motorOffTime1);
        printf("\r\nMotor 2 On Time: %d", GlobalSettings.motorOnTime2);
        printf("\r\nMotor 2 Off Time: %d", GlobalSettings.motorOffTime2);
        printf("\r\nMinVoltage: %d", GlobalSettings.minACVolt);
        printf("\r\nMaxVoltage: %d", GlobalSettings.maxACVolt);
        printf("\r\noffTimeTankOffLevel: %d", GlobalSettings.offTimeTankOffLevel);
        printf("\r\nrelayPulse: %d", GlobalSettings.relayPulse);
        printf("\r\nstarterSelection: %d", (int)GlobalSettings.starterSelection);
        printf("\r\nDualMotorEnable: %d", (int)GlobalSettings.dualMotorEnable);
        printf("\r\nMotor Mode: %d", (int)GlobalSettings.motorMode);
        printf("\r\nMotor1 Cycles: %d", GlobalSettings.motor1Cycles);
        printf("\r\nMotor2 Cycles: %d", GlobalSettings.motor2Cycles);
        printf("\r\nSumpType: %d", (int)GlobalSettings.sumpType);
        printf("\r\nCT Enable: %d", (int)GlobalSettings.ctEnable);
        printf("\r\nCT Ref 1: %0.1f", GlobalSettings.referenceCurrent1);
        printf("\r\nCT Ref 2: %0.1f", GlobalSettings.referenceCurrent2);
        printf("\r\nTransformer: %s", GlobalSettings.transformerType ? "12-0-12-0" : "12-0-12");
        if (GlobalSettings.voltageLimitEnable > 1)
        {
            GlobalSettings.voltageLimitEnable = 1; // Default to Enable if uninitialized in EEPROM
        }
        printf("\r\nVoltLimitEnable: %d", (int)GlobalSettings.voltageLimitEnable);
#ifdef MINIMUM_FUNCTIONALITY
        GlobalSettings.dualMotorEnable = 0;
        GlobalSettings.sumpType = 1;
#endif
        printf("\r\nMode: ");
        if (GlobalSettings.workMode == WORKING_MODE_AUTO)
        {
            printf("AUTO");
            L_MODE_AUTO = 0;
            // L_MODE_SEL = MODE_LED_AUTO_GREEN;
        }
        else if (GlobalSettings.workMode == WORKING_MODE_MANUAL)
        {
            printf("MANUAL");
            L_MODE_AUTO = 1;
            // L_MODE_SEL = MODE_LED_MANUAL_RED;
        }
        else
        {
            printf("AUTO");
            L_MODE_AUTO = 0;
            GlobalSettings.workMode = WORKING_MODE_AUTO;
        }
    }
    else
    {
        // these are default settings
        GlobalSettings.state = 1;
        GlobalSettings.state2 = 84;

        GlobalSettings.dryRunTimeOut = 60;           // default 120S
        GlobalSettings.dryRunReset = 120;            // default 2 hrs, value stored in minutes
        GlobalSettings.secondaryDryRunDuration = 20; // default 20s
#ifdef MINIMUM_FUNCTIONALITY
        GlobalSettings.powerOnDelaySeconds = 10;
        GlobalSettings.powerOnDelayMinutes = 0;
        GlobalSettings.powerOnDelayHours = 0;
#else
        GlobalSettings.powerOnDelay = 10;            // default 10s
#endif
        GlobalSettings.tankOnLevel = 50;
        GlobalSettings.tankOffLevel = 100;
        GlobalSettings.motorOnTime1 = 30;
        GlobalSettings.motorOffTime1 = 0;
        GlobalSettings.motorOnTime2 = 30;
        GlobalSettings.motorOffTime2 = 0;
        GlobalSettings.minACVolt = 140;
        GlobalSettings.maxACVolt = 270;
        GlobalSettings.dualTankEnable = 0;
        GlobalSettings.sumpType = 1;
        GlobalSettings.offTimeTankOffLevel = 10;
        GlobalSettings.relayPulse = 2;
        GlobalSettings.starterSelection = 0;         // Old starter, value 1 for electronic starter
        GlobalSettings.workMode = WORKING_MODE_AUTO; // default Auto
        GlobalSettings.dualMotorEnable = 0;
        GlobalSettings.motorMode = 0; // ACO
        GlobalSettings.motor1Cycles = 0;
        GlobalSettings.motor2Cycles = 0;
        GlobalSettings.ctEnable = 0;
        GlobalSettings.referenceCurrent1 = 0.0f;
        GlobalSettings.referenceCurrent2 = 0.0f;
        GlobalSettings.transformerType = 0;      // Default: 12-0-12 center tap
        GlobalSettings.voltageLimitEnable = 0;   // Default: Volt Limit Enabled

#ifdef MINIMUM_FUNCTIONALITY
        GlobalSettings.dualMotorEnable = 0;
        GlobalSettings.sumpType = 1;
#endif

        L_MODE_AUTO = 0;
        printf("\r\nDefault..");
        writeSettingEEPROM();
        // SendStringUART2_LN("\r\nCompleted");
    }
}

void writeSettingEEPROM(void)
{
    Write_DATAFLASH_ARRAY(0x7F00, (unsigned char *)&GlobalSettings, sizeof(GlobalSettings)); // write structure
    printf("\r\nSaved...");
}
void readSettingsMotorOffRemainingEEPROM(void)
{
    Read_DATAFLASH_ARRAY(0x7F80, (unsigned char *)&GlobalMotorOffTimeStruct, sizeof(GlobalMotorOffTimeStruct)); // read structure

    printf("\r\nST: %d ", GlobalMotorOffTimeStruct.state);
    printf("\r\nST2: %d", GlobalMotorOffTimeStruct.state2);
    if ((GlobalMotorOffTimeStruct.state == 1) && (GlobalMotorOffTimeStruct.state2 == 39))
    {
        printf("\r\nEnabled: %d", GlobalMotorOffTimeStruct.motorOffTimeEnabled);
        printf("\r\nRemaining: %d", GlobalMotorOffTimeStruct.motorOffTimeRemainingTime);
        motorOffTime_val = GlobalMotorOffTimeStruct.motorOffTimeRemainingTime;
    }
    else
    {

        GlobalMotorOffTimeStruct.state = 1;
        GlobalMotorOffTimeStruct.state2 = 39;

        GlobalMotorOffTimeStruct.motorOffTimeEnabled = 0;
        GlobalMotorOffTimeStruct.motorOffTimeRemainingTime = 0;

        printf("\r\nDefault.. motorOffTime");
        writeSettingMotorOffRemainingEEPROM();
    }
}
void writeSettingMotorOffRemainingEEPROM(void)
{
    Write_DATAFLASH_ARRAY(0x7F80, (unsigned char *)&GlobalMotorOffTimeStruct, sizeof(GlobalMotorOffTimeStruct)); // write structure
    printf("\r\nSaved...");
}
void updateMotorOffRemainingRemainingTime(long remainingTime)
{
    if (GlobalMotorOffTimeStruct.motorOffTimeRemainingTime != remainingTime)
    {
        GlobalMotorOffTimeStruct.motorOffTimeRemainingTime = remainingTime;
        writeSettingMotorOffRemainingEEPROM();
        printf("\r\nMotor Off Time Updated");
    }
    else
    {
        printf("\r\nMotor Off Time, Update Same Value");
    }
}
int getRemainingOffTime(void)
{
    if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ALT || GlobalSettings.motorMode == MOTOR_MODE_ACC || GlobalSettings.motorMode == MOTOR_MODE_SAL))
    {
        long rem1 = 999999, rem2 = 999999;
        if (motor1InRest)
        {
            long elapsed = (long)((getTicks100ms() - motor1RestStartTicks) / 10);
            rem1 = ((long)GlobalSettings.motorOffTime1 * 60) - elapsed;
            if (rem1 < 0) rem1 = 0;
        }
        if (motor2InRest)
        {
            long elapsed = (long)((getTicks100ms() - motor2RestStartTicks) / 10);
            rem2 = ((long)GlobalSettings.motorOffTime2 * 60) - elapsed;
            if (rem2 < 0) rem2 = 0;
        }
        if (rem1 == 999999 && rem2 == 999999) return 0;
        if (rem1 < rem2) return (int)(rem1 / 60);
        return (int)(rem2 / 60);
    }
    return (motorOffTime_val - ((getTicks100ms() - motorResetTicks) / 10)) / 60;
}
void enableMotorOffTimer(unsigned int offTime)
{
    GlobalMotorOffTimeStruct.motorOffTimeEnabled = 1;
    GlobalMotorOffTimeStruct.state = 1;
    GlobalMotorOffTimeStruct.state2 = 39;
    GlobalMotorOffTimeStruct.motorOffTimeRemainingTime = offTime;
    writeSettingMotorOffRemainingEEPROM();
    motorOffTime_val = offTime * 60;
    printf("\r\nVal:%d \r\n", motorOffTime_val);
}
void disableMotorOffTimer(void)
{
    GlobalMotorOffTimeStruct.motorOffTimeEnabled = 0;
    GlobalMotorOffTimeStruct.state = 1;
    GlobalMotorOffTimeStruct.state2 = 39;
    GlobalMotorOffTimeStruct.motorOffTimeRemainingTime = 0;
    writeSettingMotorOffRemainingEEPROM();
    motorOffTime_val = 0;
    printf("\r\nVal:%d \r\n", motorOffTime_val);
}

void Motor1_On()
{
    unsigned long motor_ticks_100ms = getTicks100ms();
    while ((getTicks100ms() - motor_ticks_100ms) < 20)
        ;
    motor_ticks_100ms = getTicks100ms();
    L_MOT1_ON = 1;
    /************************* For Electronic Starters *******************************/
    if (GlobalSettings.starterSelection == 1)
    {
        O_RLY_0 = 0;
        O_RLY_1 = 0;

        if (GlobalSettings.relayPulse > 0)
        {
            O_RLY_0 = 1; // Always On Relay
            while ((getTicks100ms() - motor_ticks_100ms) < 50)
                ;
            O_RLY_1 = 1; // Pulse relay On
            motor_ticks_100ms = getTicks100ms();
            // pulse = (GlobalSettings.relayPulseDuration*10);
            while ((getTicks100ms() - motor_ticks_100ms) < ((GlobalSettings.relayPulse * 10)))
                ;
            O_RLY_1 = 0; // Pulse Relay Off
        }
        else
        {
            O_RLY_1 = 1;
            while ((getTicks100ms() - motor_ticks_100ms) < 50)
                ;
            O_RLY_0 = 1;
        }
    }
    /************************* For Electronic Starters Ends Here ********************************/

    /************************* For Old Starters ********************************/
    else
    {

        O_RLY_0 = 0; // Always On Relay
        O_RLY_1 = 0; // Pulse Relay
        while ((getTicks100ms() - motor_ticks_100ms) < 10)
            ;
        O_RLY_0 = 0; // Always On Relay
        O_RLY_1 = 1; // Pulse Relay
        motor_ticks_100ms = getTicks100ms();
        while ((getTicks100ms() - motor_ticks_100ms) < 20)
            ;
        O_RLY_0 = 1; // Always On Relay
        O_RLY_1 = 1; // Pulse Relay
        motor_ticks_100ms = getTicks100ms();
        if (GlobalSettings.relayPulse > 0)
        {
            motor_ticks_100ms = getTicks100ms();
            while ((getTicks100ms() - motor_ticks_100ms) < ((GlobalSettings.relayPulse * 10)))
                ;
            O_RLY_1 = 0; // Pulse Relay Off
        }
        else
        {
            O_RLY_1 = 1;
            O_RLY_0 = 1;
        }
        /************************* For Old Starters Ends Here ********************************/
    }
    motorState = 1;
    runningMotor = 1;

    motorOnIndicationFlag = motorRunningCount = motorOffInitFlag = dryRunResetInitFlag = 0;
    motorOffBySensorFlag = 0;
    motorElapsedTicks = getTicks100ms();

    /* Relay switching causes inductive spikes / EMI that can corrupt the
       bit-banged TM1637 protocol.  Wait for the noise to settle, then
       re-initialise the display communication and push cached data. */
    {   unsigned long settle = getTicks100ms();
        while ((getTicks100ms() - settle) < 20) ; /* 200 ms settling delay */
    }
    TM1637_init(1, 5);
    TM1637_Write6Digits();
}

#ifdef MINIMUM_FUNCTIONALITY
void Motor2_On() {}
void Motor2_Off() {}
#else
void Motor2_On()
{
    unsigned long motor_ticks_100ms = getTicks100ms();
    L_MOT1_ON = 1;
    while ((getTicks100ms() - motor_ticks_100ms) < 20)
        ;
    motor_ticks_100ms = getTicks100ms();

    /************************* For Electronic Starters *******************************/
    if (GlobalSettings.starterSelection == 1)
    {
        O_RLY_2 = 0;
        O_RLY_3 = 0;

        if (GlobalSettings.relayPulse > 0)
        {
            O_RLY_2 = 1; // Always On Relay
            while ((getTicks100ms() - motor_ticks_100ms) < 50)
                ;
            O_RLY_3 = 1; // Pulse relay On
            motor_ticks_100ms = getTicks100ms();
            while ((getTicks100ms() - motor_ticks_100ms) < ((GlobalSettings.relayPulse * 10)))
                ;
            O_RLY_3 = 0; // Pulse Relay Off
        }
        else
        {
            O_RLY_3 = 1;
            while ((getTicks100ms() - motor_ticks_100ms) < 50)
                ;
            O_RLY_2 = 1;
        }
    }
    /************************* For Electronic Starters Ends Here ********************************/

    /************************* For Old Starters ********************************/
    else
    {

        O_RLY_2 = 0; // Always On Relay
        O_RLY_3 = 0; // Pulse Relay
        while ((getTicks100ms() - motor_ticks_100ms) < 10)
            ;
        O_RLY_2 = 0; // Always On Relay
        O_RLY_3 = 1; // Pulse Relay
        motor_ticks_100ms = getTicks100ms();
        while ((getTicks100ms() - motor_ticks_100ms) < 20)
            ;
        O_RLY_2 = 1; // Always On Relay
        O_RLY_3 = 1; // Pulse Relay
        motor_ticks_100ms = getTicks100ms();
        if (GlobalSettings.relayPulse > 0)
        {
            motor_ticks_100ms = getTicks100ms();
            while ((getTicks100ms() - motor_ticks_100ms) < ((GlobalSettings.relayPulse * 10)))
                ;
            O_RLY_3 = 0; // Pulse Relay Off
        }
        else
        {
            O_RLY_3 = 1;
            O_RLY_2 = 1;
        }
        /************************* For Old Starters Ends Here ********************************/
    }
    motorState = 1; // This tracks if any motor is ON essentially
    runningMotor = 2;

    motorOnIndicationFlag = motorRunningCount = motorOffInitFlag = dryRunResetInitFlag = 0;
    motorOffBySensorFlag = 0;
    motorElapsedTicks = getTicks100ms();

    /* Relay switching causes inductive spikes / EMI that can corrupt the
       bit-banged TM1637 protocol.  Wait for the noise to settle, then
       re-initialise the display communication and push cached data. */
    {   unsigned long settle = getTicks100ms();
        while ((getTicks100ms() - settle) < 20) ; /* 200 ms settling delay */
    }
    TM1637_init(1, 5);
    TM1637_Write6Digits();
}

void Motor2_Off()
{
    // In single motor + single sump, O_RLY_2 is repurposed as sump level relay
    // Only reset relay pins when actually used for motor control
    if (GlobalSettings.dualMotorEnable == 1)
    {
        O_RLY_2 = 0;
    }
    O_RLY_3 = 0;
    motorState = 0;
    runningMotor = 0;
    L_MOT1_ON = 0;
    // L_MOT2_ON = 0; // Not used/mapped
    motorResetTicks = getTicks100ms();

    /* Relay switching causes inductive spikes / EMI that can corrupt the
       bit-banged TM1637 protocol.  Wait for the noise to settle, then
       re-initialise the display communication and push cached data. */
    {   unsigned long settle = getTicks100ms();
        while ((getTicks100ms() - settle) < 20) ; /* 200 ms settling delay */
    }
    TM1637_init(1, 5);
    TM1637_Write6Digits();
}
#endif

void Motor1_Off()
{
    O_RLY_0 = 0;
    O_RLY_1 = 0;
    motorState = 0;
    runningMotor = 0;
    L_MOT1_ON = 0;
    motorResetTicks = getTicks100ms();

    /* Relay switching causes inductive spikes / EMI that can corrupt the
       bit-banged TM1637 protocol.  Wait for the noise to settle, then
       re-initialise the display communication and push cached data. */
    {   unsigned long settle = getTicks100ms();
        while ((getTicks100ms() - settle) < 20) ; /* 200 ms settling delay */
    }
    TM1637_init(1, 5);
    TM1637_Write6Digits();
}

static void Handle_ButtonPresses(void)
{
    printf("\r\nReading Button");
    switch_val = readButton();
    if (switch_val != 0 && dryRunBuzzerActive)
    {
        dryRunBuzzerActive = 0;
        BUZZ = 0;
        printf("\r\nDry Run Buzzer Muted by Button");
    }
    if (sumpLevel < 25)
    {
        sumpEmptyDetectFlag = 1;
    }
    if (switch_val == 1) // Motor On Button Pressed, It will also clear dry run fault state
    {
        if ((GlobalSettings.workMode == WORKING_MODE_AUTO) && (state != MOTOR_ON_SECONDARY_DRY_CHECK))
        {
            if ((tankLevel != 100) && (motorState == 0))
            {
                if (state != MOTOR_OFF_SENSOR_ERROR)
                {
                    motor1FaultFlag = 0;
                    motor2FaultFlag = 0;
                    lastDryRunMotor = 0;
                    state = MOTOR_ON;
                    state_ticks_100ms = getTicks100ms();
                    motorOnInitFlag = 0;
                }
            }
            else
            {
                printf("\r\nTank Full");
            }
        }
        else if (GlobalSettings.workMode == WORKING_MODE_MANUAL)
        {
            if (state != MOTOR_RUNNING_MANUAL)
            {
                state = MOTOR_ON_MANUAL;
            }
        }
    }
    else if (switch_val == 3) // Motor Off Button Pressed
    {
        if (tankEmptyDetectFlag || tankFullDetectFlag)
        {
            tankEmptyDetectFlag = tankFullDetectFlag = 0;
            BUZZ = 0;
            //_delay_ms(2000);
        }
        else if (sump1EmptyDetectFlag || sump1FullDetectFlag)
        {
            sump1EmptyDetectFlag = sump1FullDetectFlag = 0;
            BUZZ = 0;
            //_delay_ms(2000);
        }
        else
        {
            if (GlobalSettings.workMode == WORKING_MODE_MANUAL)
            {
                Motor1_Off();
                Motor2_Off();
                state = MOTOR_OFF_SYSTEM_IDLE;
            }
        }
    }
    else if (switch_val == 2) // Mode Change/Middle Button Pressed
    {
        if (GlobalSettings.workMode == WORKING_MODE_AUTO)
        {
            GlobalSettings.workMode = WORKING_MODE_MANUAL;
            printf("\r\nMAN");
            // L_MODE_SEL = MODE_LED_MANUAL_RED;
            state = MOTOR_OFF_MANUAL;
            motor1InRest = motor2InRest = 0; // Clear ALT rest flags on mode switch
            TM1637_WriteString(1, "MOD");
            TM1637_WriteString(2, "MAN");
            L_MODE_AUTO = 1;
            // L_MAN_MODE = 1;L_AUTO_MODE = 0;
        }
        else if (GlobalSettings.workMode == WORKING_MODE_MANUAL)
        {
            GlobalSettings.workMode = WORKING_MODE_AUTO;
            printf("\r\nAUT");
            // L_MODE_SEL = MODE_LED_AUTO_GREEN;
            TM1637_WriteString(1, "MOD");
            TM1637_WriteString(2, "ATO");
            L_MODE_AUTO = 0;
            // L_MAN_MODE = 0;L_AUTO_MODE = 1;
        }
        writeSettingEEPROM();
        _delay_ms(5000);
    }
}

static void StateHandler_BootStart(void)
{
    if (GlobalSettings.workMode == WORKING_MODE_AUTO)
    {
        printf("\r\nState:BOOT_START");
        motor1InRest = motor2InRest = 0; // Clear ALT rest flags on boot
        if (motorOffTime_val)
        {
            printf("\r\nPending Motor Off Time: %ld", motorOffTime_val);
            state = MOTOR_OFF_RESET_DELAY;
        }
        else
        {
            if (checkMotorOnConditions())
            {
                printf("\r\nMotor On");
            }
            if (checkMotorOffConditions())
            {
                printf("\r\nMotor Off");
            }
        }
    }
    else if (GlobalSettings.workMode == WORKING_MODE_MANUAL)
    {
        printf("\r\nState:BOOT_START_MANUAL");
        state = MOTOR_OFF_SYSTEM_IDLE;
    }
}

static void StateHandler_MotorOnDelay(void)
{
    printf("\r\nState:MOTOR_ON_DELAY");
    if (!motorOnInitFlag)
    {
        motorOnInitFlag = 1;
        state_ticks_100ms = getTicks100ms();
    }
    else
    {
        if ((getTicks100ms() - state_ticks_100ms) / 10 < MOTOR_ON_DELAY)
        {
            printf("\r\nMOT On Delay:%d", MOTOR_ON_DELAY - (int)((getTicks100ms() - state_ticks_100ms) / 10));
        }
        else
        {
            startSelectedMotor();
            if (state == MOTOR_ON)
            {
                state = MOTOR_ON_PRIMARY_DRY_RUN_CHECK;
                state_ticks_100ms = getTicks100ms();
            }
            motorOnInitFlag = 0;
        }
    }
}

static void StateHandler_PrimaryDryRunCheck(void)
{
    printf("\r\nST:PRI_DRY");
    if (GlobalSettings.dryRunTimeOut)
    {
        if (checkMotorOffConditions())
        {
            printf("\r\nMotor Off");
        }
        if ((getTicks100ms() - state_ticks_100ms) / 10 < GlobalSettings.dryRunTimeOut)
        {
            if (!dryRunState)
            {
                printf("\r\nMotor Running..");
                flowLEDsMotorOn();
            }
            else
            {
                flowLEDsMotorDryRunCheck();
                printf("\r\nPrimary Dry Run Timeout:%ld", (GlobalSettings.dryRunTimeOut) - (getTicks100ms() - state_ticks_100ms) / 10);
            }
        }
        else
        {
            if (dryRunState)
            {
                if (GlobalSettings.secondaryDryRunDuration)
                {
                    state = MOTOR_ON_SECONDARY_DRY_CHECK;
                    state_ticks_100ms = getTicks100ms();
                }
                else
                {
                    if (GlobalSettings.dualMotorEnable)
                    {
                        if (O_RLY_0 || O_RLY_1)
                        {
                            motor1FaultFlag = 1;
                            lastDryRunMotor = 1;
                        }
                        if (O_RLY_2 || O_RLY_3)
                        {
                            motor2FaultFlag = 1;
                            lastDryRunMotor = 2;
                        }

                        Motor1_Off();
                        Motor2_Off();

                        dryRunBuzzerActive = 1;
                        dryRunBuzzerStartTicks = getTicks100ms();

                        if (GlobalSettings.motorMode == MOTOR_MODE_SUM || GlobalSettings.motorMode == MOTOR_MODE_BOR || GlobalSettings.motorMode == MOTOR_MODE_COM)
                        {
                            state = MOTOR_OFF_DRY_RUN_FAULT;
                            printf("\r\nDry Run Fault (Non-Alternating Mode)");
                        }
                        else if (motor1FaultFlag && motor2FaultFlag)
                        {
                            state = MOTOR_OFF_DRY_RUN_FAULT;
                            printf("\r\nAll Motors Failed");
                        }
                        else
                        {
                            state = MOTOR_ON; // Recycle with delay to switch to secondary motor
                        }
                    }
                    else
                    {
                        state = MOTOR_OFF_DRY_RUN_FAULT;
                        lastDryRunMotor = 1;
                        Motor1_Off();
                        Motor2_Off();
                        dryRunBuzzerActive = 1;
                        dryRunBuzzerStartTicks = getTicks100ms();
                        printf("\r\nDry Run Fault");
                    }
                }
            }
            else if (!dryRunState)
            {
                state = MOTOR_ON_SECONDARY_DRY_CHECK;
                printf("\r\nMotor Running..");
                state_ticks_100ms = getTicks100ms();
            }
        }
    }
    else
    {
        state = MOTOR_ON_SECONDARY_DRY_CHECK;
        state_ticks_100ms = getTicks100ms();
    }
}

static void StateHandler_DryRunFault(void)
{
    flowLEDsDryRunFault();
    printf("\r\nST:DRY_FAULT");
    if (!dryRunResetInitFlag)
    {
        dryRunResetInitFlag = 1;
        state_ticks_100ms = getTicks100ms();
        while ((getTicks100ms() - state_ticks_100ms) < 50)
            ;
        Motor1_Off();
        Motor2_Off();
        
        dryRunBuzzerActive = 1;
        dryRunBuzzerStartTicks = getTicks100ms();
    }
    else
    {
        if ((getTicks100ms() - state_ticks_100ms) / 10 < (GlobalSettings.dryRunReset * 60))
        {
            printf("\r\nDry Run Reset Timeout:%ld", (GlobalSettings.dryRunReset * 60) - (getTicks100ms() - state_ticks_100ms) / 10);
            resetMinutes = ((GlobalSettings.dryRunReset * 60) - (getTicks100ms() - state_ticks_100ms) / 10) / 60;
            if (resetMinutes < 1)
            {
                resetMinutes = (GlobalSettings.dryRunReset * 60) - (getTicks100ms() - state_ticks_100ms) / 10;
                dispCount = 10;
            }
        }
        else
        {
            printf("\r\nDry Run Reset Over");
            state = MOTOR_OFF_SYSTEM_IDLE;
            motor1FaultFlag = motor2FaultFlag = 0; // Clear faults
            lastDryRunMotor = 0;
            motor1InRest = motor2InRest = 0; // Clear ALT rest flags
        }
        if (reset_press_count)
        {
            state = MOTOR_OFF_SYSTEM_IDLE;
            motor1FaultFlag = motor2FaultFlag = 0; // Clear faults
            lastDryRunMotor = 0;
            motor1InRest = motor2InRest = 0; // Clear ALT rest flags
            reset_press_count = 0;
            printf("\r\nDry Run Reset Over");
        }
    }
}

static void StateHandler_SecondaryDryRunCheck(void)
{
    printf("\r\nST:SEC_DRY");
    if (dryRunState == 0)
    {
        state_ticks_100ms = getTicks100ms();
        flowLEDsMotorOn();
        printf("\r\nMotor Running ");
    }
    else
    {
        if (GlobalSettings.dryRunTimeOut)
        {
            if ((getTicks100ms() - state_ticks_100ms) / 10 < GlobalSettings.secondaryDryRunDuration)
            {
                printf("\r\nSecondary Dry Run Timeout:%ld", (GlobalSettings.secondaryDryRunDuration) - (getTicks100ms() - state_ticks_100ms) / 10);
                flowLEDsMotorDryRunCheck();
                if (tankLevel >= 100)
                {
                    // In COM mode with Motor 2 (bore to sump), tank is already full.
                    // Don't abort dry run check — Motor 2 fills the sump, not the tank.
                    if (!(GlobalSettings.motorMode == MOTOR_MODE_COM && runningMotor == 2))
                    {
                        Motor1_Off();
                        Motor2_Off();
                        state = MOTOR_OFF_SYSTEM_IDLE;
                        flowLEDsMotorOff();
                    }
                }
            }
            else
            {
                if (GlobalSettings.dualMotorEnable)
                {
                    if (O_RLY_0 || O_RLY_1)
                    {
                        motor1FaultFlag = 1;
                        lastDryRunMotor = 1;
                    }
                    if (O_RLY_2 || O_RLY_3)
                    {
                        motor2FaultFlag = 1;
                        lastDryRunMotor = 2;
                    }

                    Motor1_Off();
                    Motor2_Off();

                    dryRunBuzzerActive = 1;
                    dryRunBuzzerStartTicks = getTicks100ms();

                    if (GlobalSettings.motorMode == MOTOR_MODE_SUM || GlobalSettings.motorMode == MOTOR_MODE_BOR || GlobalSettings.motorMode == MOTOR_MODE_COM)
                    {
                        state = MOTOR_OFF_DRY_RUN_FAULT;
                        printf("\r\nDry Run Fault (Non-Alternating Mode)");
                    }
                    else if (motor1FaultFlag && motor2FaultFlag)
                    {
                        state = MOTOR_OFF_DRY_RUN_FAULT;
                        printf("\r\nDry Run Fault All");
                    }
                    else
                    {
                        state = MOTOR_ON;
                    }
                }
                else
                {
                    state = MOTOR_OFF_DRY_RUN_FAULT;
                    lastDryRunMotor = 1;
                    Motor1_Off();
                    Motor2_Off();
                    dryRunBuzzerActive = 1;
                    dryRunBuzzerStartTicks = getTicks100ms();
                    printf("\r\nDry Run Fault");
                }
            }
        }
        else
        {
            flowLEDsMotorDryRunCheck();
        }
    }
    if (checkMotorOffConditions())
    {
        printf("\r\nMotor Off");
    }
}

// ALT Mode: Check and expire per-motor rest timers
static void updateALTRestTimers(void)
{
    static unsigned long last_print_sec = 0;
    unsigned long current_sec = getTicks100ms() / 10;
    unsigned char should_print = (current_sec != last_print_sec);

    if (motor1InRest)
    {
        unsigned long elapsed = (getTicks100ms() - motor1RestStartTicks) / 10;
        unsigned long total = (unsigned long)GlobalSettings.motorOffTime1 * 60;
        if (elapsed >= total)
        {
            motor1InRest = 0;
            printf("\r\nM1 Rest Over");
        }
        else if (should_print)
        {
            printf("\r\nM1 Rest Rem:%lu s", total - elapsed);
        }
    }
    if (motor2InRest)
    {
        unsigned long elapsed = (getTicks100ms() - motor2RestStartTicks) / 10;
        unsigned long total = (unsigned long)GlobalSettings.motorOffTime2 * 60;
        if (elapsed >= total)
        {
            motor2InRest = 0;
            printf("\r\nM2 Rest Over");
        }
        else if (should_print)
        {
            printf("\r\nM2 Rest Rem:%lu s", total - elapsed);
        }
    }

    if (should_print)
    {
        last_print_sec = current_sec;
    }
}

// ALT Mode: Returns 1 if at least one motor is not resting and not faulted
static unsigned char isALTMotorAvailable(void)
{
    if (!motor1InRest && !motor1FaultFlag && sumpLevel > 25) return 1;
    if (!motor2InRest && !motor2FaultFlag) return 1;
    return 0;
}

static void StateHandler_MotorOff(void)
{
    printf("\r\nState:MOTOR_OFF");
    if (!motorOffInitFlag)
    {
        motorOffInitFlag = 1;
        state_ticks_100ms = getTicks100ms();
    }
    else
    {
        if ((getTicks100ms() - state_ticks_100ms) / 10 < GlobalSettings.offTimeTankOffLevel)
        {
            printf("\r\nMOT OFF Time:%ld", (long)GlobalSettings.offTimeTankOffLevel - (long)((getTicks100ms() - state_ticks_100ms) / 10));
            flowLEDsMotorOn();
        }
        else
        {
            unsigned char stoppedMotor = runningMotor; // Capture before Motor_Off clears it
            unsigned int currentOffTime = (runningMotor == 2) ? GlobalSettings.motorOffTime2 : GlobalSettings.motorOffTime1;
            printf("\r\nMOT SW OFF");
            L_FLOW_1 = L_FLOW_2 = L_FLOW_3 = 0;
            Motor1_Off();
            Motor2_Off();

            // ALT Mode: Per-motor independent rest
            if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ALT || GlobalSettings.motorMode == MOTOR_MODE_ACC || GlobalSettings.motorMode == MOTOR_MODE_SAL)
                && currentOffTime && (motorOffBySensorFlag == 0))
            {
                // Mark the stopped motor as resting
                if (stoppedMotor == 2)
                {
                    motor2InRest = 1;
                    motor2RestStartTicks = getTicks100ms();
                    printf("\r\nM2 Rest Start: %d min", GlobalSettings.motorOffTime2);
                }
                else
                {
                    motor1InRest = 1;
                    motor1RestStartTicks = getTicks100ms();
                    printf("\r\nM1 Rest Start: %d min", GlobalSettings.motorOffTime1);
                }

                // Check if the OTHER motor can run now
                updateALTRestTimers();
                if (isALTMotorAvailable())
                {
                    // Route through IDLE so checkMotorOnConditions() validates
                    // tank level, sump level, and voltage before starting
                    printf("\r\nALT: Other motor available, checking conditions");
                    state = MOTOR_OFF_SYSTEM_IDLE;
                }
                else
                {
                    printf("\r\nALT: Both motors resting");
                    state = MOTOR_OFF_RESET_DELAY;
                    motorResetTicks = getTicks100ms();
                }
            }
            else if (currentOffTime && (motorOffBySensorFlag == 0))
            {
                state = MOTOR_OFF_RESET_DELAY;
                enableMotorOffTimer(currentOffTime);
            }
            else
            {
                state = MOTOR_OFF_SYSTEM_IDLE;
            }
        }
        if (dryRunState)
        {
            flowLEDsMotorOff();
        }
    }
}

static void StateHandler_MotorOffResetDelay(void)
{
    printf("\r\nState:MOTOR_OFF_RESET_DELAY");
    flowLEDsMotorOff();

    if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ALT || GlobalSettings.motorMode == MOTOR_MODE_ACC || GlobalSettings.motorMode == MOTOR_MODE_SAL))
    {
        updateALTRestTimers();
        if (isALTMotorAvailable())
        {
            printf("\r\nALT: Motor available after rest");
            state_ticks_100ms = getTicks100ms();
            state = MOTOR_OFF_SYSTEM_IDLE;
        }
        else if (readButton() == 1)
        {
            // Manual override: clear all rest timers
            motor1InRest = motor2InRest = 0;
            state = MOTOR_OFF_SYSTEM_IDLE;
        }
    }
    else
    {
        checkMotorResetTimeOut();
    }
}

static void StateHandler_VoltageFault(void)
{
    long remainingTime = (long)VOLTAGE_RESET_TIME - (long)((getTicks100ms() - voltageFaultTicks) / 10);
    printf("\r\nState:MOTOR_OFF_VOLTAGE_FAULT. V:%0.1f", lineVoltage);

    if (remainingTime > 0)
    {
        printf("\r\nVoltage Fault Timer:%ld", remainingTime);
        flowLEDsMotorOff();
        Motor1_Off();
        Motor2_Off();
    }
    else
    {
        // Timer expired, now check hysteresis for recovery
        if (lineVoltage < (GlobalSettings.minACVolt + VOLTAGE_HYSTERESIS))
        {
            printf("\r\nWaiting for Voltage Recovery (>%d V)", GlobalSettings.minACVolt + VOLTAGE_HYSTERESIS);
            flowLEDsMotorOff();
            Motor1_Off();
            Motor2_Off();
        }
        else if (lineVoltage > GlobalSettings.maxACVolt)
        {
            printf("\r\nWaiting for Voltage to drop below %d V", GlobalSettings.maxACVolt);
            flowLEDsMotorOff();
            Motor1_Off();
            Motor2_Off();
        }
        else
        {
            printf("\r\nVoltage Recovered. Transitioning to IDLE.");
            state = MOTOR_OFF_SYSTEM_IDLE;
        }
    }
}

static void StateHandler_SystemIdle(void)
{
    printf("\r\nState:MOTOR_OFF_SYSTEM_IDLE");
    flowLEDsMotorOff();
    motorOnIndicationFlag = motorRunningCount = motorOffInitFlag = dryRunResetInitFlag = 0;
    printf("\r\nIdle.");
    printf("S_LVL: %d", sumpLevel);
    printf(" T_LVL: %d", tankLevel);
    if (GlobalSettings.workMode == WORKING_MODE_AUTO)
    {
        checkMotorOnConditions();
    }
}

static void StateHandler_ManualMode(void)
{
    if (state == MOTOR_ON_MANUAL)
    {
        state = MOTOR_ON;
        state_ticks_100ms = getTicks100ms();
        motorOnInitFlag = 0;
    }
    else if (state == MOTOR_RUNNING_MANUAL)
    {
        state = MOTOR_RUNNING_MANUAL;
        printf("\r\nMOT Run ");
        if (dryRunState == 0)
        {
            flowLEDsMotorOn();
        }
        else
        {
            flowLEDsMotorDryRunCheck();
        }
        checkMotorOffConditions();
    }
    else if (state == MOTOR_OFF_MANUAL)
    {
        Motor1_Off();
        Motor2_Off();
        printf("\r\nMOT Off MAN, idle\r\n");
        state = MOTOR_OFF_SYSTEM_IDLE;
    }
}

static void StateHandler_SensorError(void)
{
    state = MOTOR_OFF_SENSOR_ERROR;
    Motor1_Off();
    Motor2_Off();
    printf("\r\nSensor Error\r\n");
}

void wlcLoop(void)
{
    Handle_ButtonPresses();
    if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ALT || GlobalSettings.motorMode == MOTOR_MODE_ACC || GlobalSettings.motorMode == MOTOR_MODE_SAL))
    {
        updateALTRestTimers();
    }

    switch (state)
    {
    case BOOT_START:
        StateHandler_BootStart();
        break;
    case MOTOR_ON:
        StateHandler_MotorOnDelay();
        break;
    case MOTOR_ON_PRIMARY_DRY_RUN_CHECK:
        StateHandler_PrimaryDryRunCheck();
        break;
    case MOTOR_ON_SECONDARY_DRY_CHECK:
        StateHandler_SecondaryDryRunCheck();
        break;
    case MOTOR_OFF:
        StateHandler_MotorOff();
        break;
    case MOTOR_OFF_DRY_RUN_FAULT:
        StateHandler_DryRunFault();
        break;
    case MOTOR_OFF_RESET_DELAY:
        StateHandler_MotorOffResetDelay();
        break;
    case MOTOR_OFF_VOLTAGE_FAULT:
        StateHandler_VoltageFault();
        break;
    case MOTOR_OFF_OVERCURRENT_FAULT:
        StateHandler_VoltageFault(); // Assuming same auto-reset logic as voltage for now, otherwise change to DryRunFault logic depending on UI preference
        break;
    case MOTOR_OFF_SYSTEM_IDLE:
        StateHandler_SystemIdle();
        break;
    case MOTOR_OFF_SENSOR_ERROR:
        StateHandler_SensorError();
        break;
    case MOTOR_ON_MANUAL:
    case MOTOR_RUNNING_MANUAL:
    case MOTOR_OFF_MANUAL:
        StateHandler_ManualMode();
        break;
    default:
        break;
    }
}

void startSelectedMotor(void)
{
#ifdef MINIMUM_FUNCTIONALITY
    Motor1_On();
#else
    if (GlobalSettings.dualMotorEnable)
    {
        switch (GlobalSettings.motorMode)
        {
        case MOTOR_MODE_ACO: // Auto Change Over
            if (motor1FaultFlag && !motor2FaultFlag)
            {
                Motor2_On(); // Switch to Borewell
            }
            else if (motor2FaultFlag && !motor1FaultFlag)
            {
                if (sumpLevel > 25)
                {
                    Motor1_On(); // Switch back to Sump if water exists
                }
                else
                {
                    state = MOTOR_OFF_RESET_DELAY;
                    motorResetTicks = getTicks100ms();
                    enableMotorOffTimer(GlobalSettings.dryRunReset ? GlobalSettings.dryRunReset : 2);
                }
            }
            else if (motor1FaultFlag && motor2FaultFlag)
            {
                state = MOTOR_OFF_DRY_RUN_FAULT;
            }
            else
            {
                if (sumpLevel <= 25)
                {
                    Motor2_On(); // Start Borewell
                }
                else
                {
                    Motor1_On(); // Start Sump Motor
                }
            }
            break;

        case MOTOR_MODE_ALT: // Sump Alternate — simple 1:1 alternation, no cycles
            updateALTRestTimers();

            if (motor1FaultFlag && !motor2FaultFlag)
            {
                if (!motor2InRest) Motor2_On();
                else { state = MOTOR_OFF_RESET_DELAY; motorResetTicks = getTicks100ms(); }
            }
            else if (motor2FaultFlag && !motor1FaultFlag)
            {
                if (!motor1InRest && sumpLevel > 25) Motor1_On();
                else { state = MOTOR_OFF_RESET_DELAY; motorResetTicks = getTicks100ms(); }
            }
            else if (motor1FaultFlag && motor2FaultFlag)
            {
                state = MOTOR_OFF_DRY_RUN_FAULT;
            }
            else if (!motor1FaultFlag && !motor2FaultFlag)
            {
                if (motorAlternateFlag) // Motor 2's turn
                {
                    if (!motor2InRest)
                    {
                        Motor2_On();
                        motorAlternateFlag = 0;
                    }
                    else if (!motor1InRest && sumpLevel > 25)
                    {
                        Motor1_On();
                        motorAlternateFlag = 1;
                    }
                    else
                    {
                        state = MOTOR_OFF_RESET_DELAY;
                        motorResetTicks = getTicks100ms();
                    }
                }
                else // Motor 1's turn
                {
                    if (!motor1InRest && sumpLevel > 25)
                    {
                        Motor1_On();
                        motorAlternateFlag = 1;
                    }
                    else if (!motor2InRest)
                    {
                        Motor2_On();
                        motorAlternateFlag = 0;
                    }
                    else
                    {
                        state = MOTOR_OFF_RESET_DELAY;
                        motorResetTicks = getTicks100ms();
                    }
                }
            }
            break;

        case MOTOR_MODE_SAL: // Sump Alternate — both motors check sump level
            updateALTRestTimers();

            if (motor1FaultFlag && !motor2FaultFlag)
            {
                if (!motor2InRest && sumpLevel > 25) Motor2_On();
                else { state = MOTOR_OFF_RESET_DELAY; motorResetTicks = getTicks100ms(); }
            }
            else if (motor2FaultFlag && !motor1FaultFlag)
            {
                if (!motor1InRest && sumpLevel > 25) Motor1_On();
                else { state = MOTOR_OFF_RESET_DELAY; motorResetTicks = getTicks100ms(); }
            }
            else if (motor1FaultFlag && motor2FaultFlag)
            {
                state = MOTOR_OFF_DRY_RUN_FAULT;
            }
            else if (!motor1FaultFlag && !motor2FaultFlag)
            {
                if (motorAlternateFlag) // Motor 2's turn
                {
                    if (!motor2InRest && sumpLevel > 25)
                    {
                        Motor2_On();
                        motorAlternateFlag = 0;
                    }
                    else if (!motor1InRest && sumpLevel > 25)
                    {
                        Motor1_On();
                        motorAlternateFlag = 1;
                    }
                    else
                    {
                        state = MOTOR_OFF_RESET_DELAY;
                        motorResetTicks = getTicks100ms();
                    }
                }
                else // Motor 1's turn
                {
                    if (!motor1InRest && sumpLevel > 25)
                    {
                        Motor1_On();
                        motorAlternateFlag = 1;
                    }
                    else if (!motor2InRest && sumpLevel > 25)
                    {
                        Motor2_On();
                        motorAlternateFlag = 0;
                    }
                    else
                    {
                        state = MOTOR_OFF_RESET_DELAY;
                        motorResetTicks = getTicks100ms();
                    }
                }
            }
            break;

        case MOTOR_MODE_ACC: // Auto Change Over with Cycles (motor1Cycles:motor2Cycles)
            updateALTRestTimers();

            if (motor1FaultFlag && !motor2FaultFlag)
            {
                if (!motor2InRest)
                    Motor2_On();
                else
                {
                    state = MOTOR_OFF_RESET_DELAY;
                    motorResetTicks = getTicks100ms();
                }
            }
            else if (motor2FaultFlag && !motor1FaultFlag)
            {
                if (!motor1InRest && sumpLevel > 25)
                    Motor1_On();
                else
                {
                    state = MOTOR_OFF_RESET_DELAY;
                    motorResetTicks = getTicks100ms();
                }
            }
            else if (motor1FaultFlag && motor2FaultFlag)
            {
                state = MOTOR_OFF_DRY_RUN_FAULT;
            }
            else if (!motor1FaultFlag && !motor2FaultFlag)
            {
                if (motorAlternateFlag) // Motor 2's turn
                {
                    if (!motor2InRest)
                    {
                        Motor2_On();
                        if (GlobalSettings.motor2Cycles > 0)
                        {
                            motor2CycleCounter++;
                            if (motor2CycleCounter >= GlobalSettings.motor2Cycles)
                            {
                                motorAlternateFlag = 0; // Switch back to Motor 1
                                motor2CycleCounter = 0;
                                motor1CycleCounter = 0;
                            }
                        }
                        else
                        {
                            motorAlternateFlag = 0; // No cycles set, alternate immediately
                        }
                    }
                    else if (!motor1InRest && sumpLevel > 25)
                    {
                        // Motor 2 resting, changeover to Motor 1
                        Motor1_On();
                        motorAlternateFlag = 0;
                        motor1CycleCounter = 0;
                        motor2CycleCounter = 0;
                    }
                }
                else // Motor 1's turn
                {
                    if (!motor1InRest && sumpLevel > 25)
                    {
                        Motor1_On();
                        if (GlobalSettings.motor1Cycles > 0)
                        {
                            motor1CycleCounter++;
                            if (motor1CycleCounter >= GlobalSettings.motor1Cycles)
                            {
                                motorAlternateFlag = 1; // Switch to Motor 2
                                motor1CycleCounter = 0;
                                motor2CycleCounter = 0;
                            }
                        }
                        else
                        {
                            motorAlternateFlag = 1; // No cycles set, alternate immediately
                        }
                    }
                    else if (!motor2InRest)
                    {
                        // Sump dry or M1 resting — Change Over to Motor 2 (borewell)
                        Motor2_On();
                        motorAlternateFlag = 1;
                        motor1CycleCounter = 0;
                        motor2CycleCounter = 0;
                    }
                }
            }
            break;

        case MOTOR_MODE_SUM: // Sump Only
            if (motor1FaultFlag)
            {
                state = MOTOR_OFF_DRY_RUN_FAULT;
            }
            else
            {
                Motor1_On();
            }
            break;

        case MOTOR_MODE_BOR: // Borewell Only
            if (motor2FaultFlag)
            {
                state = MOTOR_OFF_DRY_RUN_FAULT;
            }
            else
            {
                Motor2_On();
            }
            break;

        case MOTOR_MODE_COM: // Combo Mode
        {
            unsigned char tankNeedsFilling = GlobalSettings.dualTankEnable
                ? (GlobalSensorCount.stateT25 == 0 || GlobalSensorCount.stateT75 == 0)
                : (tankLevel < GlobalSettings.tankOnLevel);

            if (sumpLevel < 25)
            {
                if (!motor2FaultFlag)
                {
                    Motor2_On(); // Fill Sump from Bore
                }
                else
                {
                    state = MOTOR_OFF_DRY_RUN_FAULT;
                }
            }
            else if (sumpLevel >= 50 && tankNeedsFilling)
            {
                if (!motor1FaultFlag)
                {
                    Motor1_On(); // Fill Tank from Sump
                }
                else
                {
                    state = MOTOR_OFF_DRY_RUN_FAULT;
                }
            }
            else
            {
                if (!motor2FaultFlag)
                {
                    Motor2_On();
                }
                else
                {
                    state = MOTOR_OFF_DRY_RUN_FAULT;
                }
            }
        }
        break;

        default:
            Motor1_On();
            break;
        }
    }
    else
    {
        Motor1_On();
    }
#endif
}

unsigned int checkMotorOnConditions(void)
{
    unsigned int ret_val = 0;

    if (GlobalSettings.dualTankEnable == 0)
    {
        if (state == BOOT_START)
        {
            printf("\r\nBoot Motor Start1");
#ifdef MINIMUM_FUNCTIONALITY
            if ((tankLevel < 100) && sumpLevel)
#else
            if ((tankLevel < 100) && (sumpLevel || (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ACO || GlobalSettings.motorMode == MOTOR_MODE_BOR))))
#endif
            {
                if (!motorState)
                {
                    state = MOTOR_ON;
                    printf("\r\nBoot Motor Start2");
                    state_ticks_100ms = getTicks100ms();
                    motorOnInitFlag = 0;
                    ret_val = 1;
                }
            }
            else
            {
                state = MOTOR_OFF_SYSTEM_IDLE;
            }
            return ret_val;
        }

#ifndef MINIMUM_FUNCTIONALITY
        // COM Mode Special Triggers
        if (GlobalSettings.dualMotorEnable && GlobalSettings.motorMode == MOTOR_MODE_COM)
        {
            // Case 1: Tank Full -> Fill Sump (Maintenance)
            if (tankLevel >= GlobalSettings.tankOffLevel && sumpLevel < 50)
            {
                if (!motor2FaultFlag && !motorState)
                {
                    state = MOTOR_ON;
                    printf("\r\nCOM: Tank Full, Filling Sump");
                    state_ticks_100ms = getTicks100ms();
                    motorOnInitFlag = 0;
                    ret_val = 1;
                }
                return ret_val;
            }
            // Case 2: Tank Low, Sump Empty -> Start Bore to fill Sump
            if (tankLevel < GlobalSettings.tankOnLevel && sumpLevel < 25)
            {
                if (!motor2FaultFlag && !motorState)
                {
                    state = MOTOR_ON;
                    printf("\r\nCOM: Tank Low, Sump Empty -> Start Bore");
                    state_ticks_100ms = getTicks100ms();
                    motorOnInitFlag = 0;
                    ret_val = 1;
                }
                return ret_val;
            }
        }
        // ACO/BOR/ACC: bypass sumpEmptyDetectFlag — borewell doesn't need sump recovery
        if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ACO || GlobalSettings.motorMode == MOTOR_MODE_BOR || GlobalSettings.motorMode == MOTOR_MODE_ACC || GlobalSettings.motorMode == MOTOR_MODE_ALT) && (tankLevel < GlobalSettings.tankOnLevel))
        {
            if (!motorState)
            {
                state = MOTOR_ON;
                printf("\r\nACO/BOR motor start");
                state_ticks_100ms = getTicks100ms();
                motorOnInitFlag = 0;
                ret_val = 1;
                sumpEmptyDetectFlag = 0;
            }
        }
        else
#endif
        if ((tankLevel < GlobalSettings.tankOnLevel) && (sumpLevel >= 50) && (sumpEmptyDetectFlag) && (GlobalSensorCount.stateS100 + GlobalSensorCount.stateS75 + GlobalSensorCount.stateS50 + GlobalSensorCount.stateS25 > 1))
        {
            if (!(GlobalSettings.motorMode == MOTOR_MODE_COM && motor1FaultFlag) && !motorState)
            {
                state = MOTOR_ON;
                printf("\r\nmotor start");
                state_ticks_100ms = getTicks100ms();
                motorOnInitFlag = 0;
                ret_val = 1;
                sumpEmptyDetectFlag = 0;
            }
        }
        else if ((tankLevel < GlobalSettings.tankOnLevel) && (sumpLevel > 0) && ((!sumpEmptyDetectFlag)))
        {
            unsigned char isFaulted = 0;
            if (GlobalSettings.motorMode == MOTOR_MODE_COM)
            {
                if (sumpLevel >= 50 && motor1FaultFlag) isFaulted = 1;
                else if (sumpLevel < 50 && motor2FaultFlag) isFaulted = 1;
            }

            if (!isFaulted && !motorState)
            {
                state = MOTOR_ON;
                printf("\r\nmotor start");
                state_ticks_100ms = getTicks100ms();
                motorOnInitFlag = 0;
                ret_val = 1;
                sumpEmptyDetectFlag = 0;
            }
        }
        else if ((tankLevel < 75) && (sumpLevel == 100) && ((!sumpEmptyDetectFlag)) && (sumpFullTankNotFufllFlag == 0))
        {
            if (!(GlobalSettings.motorMode == MOTOR_MODE_COM && motor1FaultFlag) && !motorState)
            {
                state = MOTOR_ON;
                sumpFullTankNotFufllFlag = 1;
                printf("\r\nmotor start");
                state_ticks_100ms = getTicks100ms();
                motorOnInitFlag = 0;
                ret_val = 1;
                sumpEmptyDetectFlag = 0;
            }
        }
        else
        {
            Motor1_Off();
#ifndef MINIMUM_FUNCTIONALITY
            Motor2_Off();
#endif
            state = MOTOR_OFF_SYSTEM_IDLE;
            ret_val = 0;
        }
    }
    else if (GlobalSettings.dualTankEnable)
    {
        unsigned char dualTankLow = ((GlobalSensorCount.stateT25 == 0) || (GlobalSensorCount.stateT75 == 0));

        // Boot start logic for dual tank
        if (state == BOOT_START)
        {
            printf("\r\nBoot Motor Start DualTank");
#ifdef MINIMUM_FUNCTIONALITY
            if (dualTankLow && sumpLevel)
#else
            if (dualTankLow && (sumpLevel || (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ACO || GlobalSettings.motorMode == MOTOR_MODE_BOR))))
#endif
            {
                if (!motorState)
                {
                    state = MOTOR_ON;
                    printf("\r\nBoot Motor Start DualTank2");
                    state_ticks_100ms = getTicks100ms();
                    motorOnInitFlag = 0;
                    ret_val = 1;
                }
            }
            else
            {
                state = MOTOR_OFF_SYSTEM_IDLE;
            }
            return ret_val;
        }

#ifndef MINIMUM_FUNCTIONALITY
        // COM Mode Special Triggers for dual tank
        if (GlobalSettings.dualMotorEnable && GlobalSettings.motorMode == MOTOR_MODE_COM)
        {
            // Case 1: Sump empty → Bore to Sump
            if (sumpLevel < 25)
            {
                if (!motor2FaultFlag && !motorState)
                {
                    state = MOTOR_ON;
                    printf("\r\nCOM DT: Sump Empty -> Bore to Sump");
                    state_ticks_100ms = getTicks100ms();
                    motorOnInitFlag = 0;
                    ret_val = 1;
                }
                return ret_val;
            }
            // Case 2: Sump >= 50 AND at least one tank empty → Sump to Tank
            if (sumpLevel >= 50 && (GlobalSensorCount.stateT25 == 0 || GlobalSensorCount.stateT75 == 0))
            {
                if (!motor1FaultFlag && !motorState)
                {
                    state = MOTOR_ON;
                    printf("\r\nCOM DT: Sump Ready -> Sump to Tank");
                    state_ticks_100ms = getTicks100ms();
                    motorOnInitFlag = 0;
                    ret_val = 1;
                }
                return ret_val;
            }
            // Case 3: Both tanks full AND sump below 75 → Bore to Sump (maintenance)
            if ((GlobalSensorCount.stateT50 && GlobalSensorCount.stateT100) && sumpLevel < 75)
            {
                if (!motor2FaultFlag && !motorState)
                {
                    state = MOTOR_ON;
                    printf("\r\nCOM DT: Tanks Full, Sump Low -> Bore to Sump");
                    state_ticks_100ms = getTicks100ms();
                    motorOnInitFlag = 0;
                    ret_val = 1;
                }
                return ret_val;
            }
            // No COM trigger matched → idle
            state = MOTOR_OFF_SYSTEM_IDLE;
            return ret_val;
        }

        // ACO/BOR/ACC: bypass sumpEmptyDetectFlag for dual tank
        if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ACO || GlobalSettings.motorMode == MOTOR_MODE_BOR || GlobalSettings.motorMode == MOTOR_MODE_ACC || GlobalSettings.motorMode == MOTOR_MODE_ALT) && dualTankLow)
        {
            if (!motorState)
            {
                state = MOTOR_ON;
                printf("\r\nACO/BOR DT motor start");
                state_ticks_100ms = getTicks100ms();
                motorOnInitFlag = 0;
                ret_val = 1;
                sumpEmptyDetectFlag = 0;
            }
        }
        else
#endif
        if (dualTankLow && (sumpLevel >= 50) && (sumpEmptyDetectFlag) && (GlobalSensorCount.stateS100 + GlobalSensorCount.stateS75 + GlobalSensorCount.stateS50 + GlobalSensorCount.stateS25 > 1))
        {
            printf("\r\nMotor start");
            state = MOTOR_ON;
            state_ticks_100ms = getTicks100ms();
            motorOnInitFlag = 0;
            sumpEmptyDetectFlag = 0;
        }
        else if (dualTankLow && (sumpLevel > 0) && ((!sumpEmptyDetectFlag)))
        {
            printf("\r\nMotor start");
            state = MOTOR_ON;
            state_ticks_100ms = getTicks100ms();
            motorOnInitFlag = 0;
            sumpEmptyDetectFlag = 0;
        }
        else
        {
            Motor1_Off();
#ifndef MINIMUM_FUNCTIONALITY
            Motor2_Off();
#endif
            state = MOTOR_OFF_SYSTEM_IDLE;
            ret_val = 0;
        }
    }
    {
        unsigned int minV = (GlobalSettings.voltageLimitEnable == 1) ? GlobalSettings.minACVolt : 145;
        unsigned int maxV = (GlobalSettings.voltageLimitEnable == 1) ? GlobalSettings.maxACVolt : 275;
        if (checkLineVoltage(minV, maxV, motorState, GlobalSettings.transformerType) == 0)
        {
            printf("\r\n Voltage Out of Range.");
            voltageFaultCount++;
            if (voltageFaultCount > 5)
            {
                ret_val = 0;
                state = MOTOR_OFF_VOLTAGE_FAULT;
                voltageFaultTicks = getTicks100ms();
            }
        }
        else
        {
            voltageFaultCount = 0;
        }
    }
    return ret_val;
}
unsigned int checkMotorResetTimeOut(void)
{
    unsigned int ret_val = 0;
    long tempOffTime = 0;
    if (motorOffTime_val)
    {
        tempOffTime = motorOffTime_val - ((getTicks100ms() - motorResetTicks) / 10);
        if ((((getTicks100ms() - motorResetTicks) / 10) % (60 * 1) == 0))
        {
            // Elapsed minute, update value in EEPROM every 1 minutes
            if (((getTicks100ms() - motorResetTicks) / 10) > 10)
            {
                updateMotorOffRemainingRemainingTime(motorOffTime_val - ((getTicks100ms() - motorResetTicks) / 10));
                _delay_ms(1200);
            }
        }
        if ((tempOffTime) <= 0 || (readButton() == 1))
        {
            printf("\r\nMotorResetTimeOver");
            state_ticks_100ms = getTicks100ms();
            state = MOTOR_OFF_SYSTEM_IDLE;
            ret_val = 1;
            disableMotorOffTimer();
            motor1FaultFlag = 0;
            motor2FaultFlag = 0;
            lastDryRunMotor = 0;
        }
    }
    return ret_val;
}
unsigned int checkMotorOffConditions(void)
{
    unsigned int ret_val = 0;
    unsigned char sourceEmpty = 0;
    if (motorState)
    {
        motorRunTimeMinutes = ((getTicks100ms() - motorElapsedTicks) / 600);

#ifndef MINIMUM_FUNCTIONALITY
        if (GlobalSettings.dualMotorEnable && GlobalSettings.sumpType == 2)
        {
            if ((O_RLY_0 || O_RLY_1) && !GlobalSensorCount.stateS25 && !GlobalSensorCount.stateS50)
                sourceEmpty = 1;
            else if ((O_RLY_2 || O_RLY_3) && !GlobalSensorCount.stateS75 && !GlobalSensorCount.stateS100)
                sourceEmpty = 1;
        }
        else
#endif
        if (sumpLevel < 25 && (runningMotor == 1 || (runningMotor == 2 && GlobalSettings.motorMode == MOTOR_MODE_SAL)))
        {
            sourceEmpty = 1;
        }
        if (GlobalSettings.dualTankEnable == 0)
        {
            printf("\r\nMotor %d Running. Mode: %s. Sump: %d Tank: %d",
                   (int)runningMotor,
#ifdef MINIMUM_FUNCTIONALITY
                   "SUM",
#else
                   (GlobalSettings.motorMode == 0 ? "ACO" : GlobalSettings.motorMode == 1 ? "ALT"
                                                        : GlobalSettings.motorMode == 2   ? "SUM"
                                                        : GlobalSettings.motorMode == 3   ? "BOR"
                                                        : GlobalSettings.motorMode == 4   ? "COB"
                                                        : GlobalSettings.motorMode == 5   ? "ACC"
                                                                                          : "SAL"),
#endif
                   sumpLevel, tankLevel);
            if (sourceEmpty)
            {
#ifndef MINIMUM_FUNCTIONALITY
                // ACO/ACC: Switch to Borewell with delay
                if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ACO || GlobalSettings.motorMode == MOTOR_MODE_ACC || GlobalSettings.motorMode == MOTOR_MODE_ALT) && runningMotor == 1)
                {
                    printf("\r\nACO: Sump Empty, switching to Borewell");
                    Motor1_Off();
                    state_ticks_100ms = getTicks100ms();
                    motorOnInitFlag = 0;
                    state = MOTOR_ON; // Will start Motor 2 after delay
                }
                else
#endif
                {
                    printf("\r\nSource empty - Motor off");
                    state_ticks_100ms = getTicks100ms();
                    if (state == MOTOR_ON_PRIMARY_DRY_RUN_CHECK)
                    {
                        Motor1_Off();
#ifndef MINIMUM_FUNCTIONALITY
                        Motor2_Off();
#endif
                        state = MOTOR_OFF_SYSTEM_IDLE;
                    }
                    else
                    {
                        state = MOTOR_OFF;
                    }
                    ret_val = 1;
                }
            }
#ifndef MINIMUM_FUNCTIONALITY
            if (GlobalSettings.motorMode == MOTOR_MODE_COM)
            {
                    if (runningMotor == 1) // Sump to Tank
                    {
                        if (tankLevel >= GlobalSettings.tankOffLevel)
                        {
                            printf("\r\nCOM: Tank Full - M1 Off");
                            state_ticks_100ms = getTicks100ms();
                            state = MOTOR_OFF;
                            ret_val = 1;
                            motorOffBySensorFlag = 1;
                        }
                    }
                    else if (runningMotor == 2) // Bore to Sump
                    {
                        if (sumpLevel >= 50 && tankLevel < 100)
                        {
                            printf("\r\nCOM: Sump above 50 %% - M2 Off");
                            state_ticks_100ms = getTicks100ms();
                            state = MOTOR_OFF;
                            ret_val = 1;
                            motorOffBySensorFlag = 1;
                        }
                        else if (sumpLevel == 100 && tankLevel == 100)
                        {
                            printf("\r\nCOM: Sump Full and Tank Full - M2 Off");
                            state_ticks_100ms = getTicks100ms();
                            state = MOTOR_OFF;
                            ret_val = 1;
                            motorOffBySensorFlag = 1;
                        }
                    }
                }
            else
#endif
            if (tankLevel >= GlobalSettings.tankOffLevel)
            {
                // Motor1_Off();
                printf("\r\nMotor off");
                state_ticks_100ms = getTicks100ms();
                state = MOTOR_OFF;
                ret_val = 1;
                motorOffBySensorFlag = 1;
            }
        }
        else if (GlobalSettings.dualTankEnable)
        {
#ifndef MINIMUM_FUNCTIONALITY
            if (GlobalSettings.motorMode == MOTOR_MODE_COM)
            {
                    if (runningMotor == 1) // Sump to Tank
                    {
                        if (GlobalSensorCount.stateT50 && GlobalSensorCount.stateT100)
                        {
                            printf("\r\nCOM DT: Both Tanks Full - M1 Off");
                            state_ticks_100ms = getTicks100ms();
                            state = MOTOR_OFF;
                            ret_val = 1;
                            motorOffBySensorFlag = 1;
                        }
                    }
                    else if (runningMotor == 2) // Bore to Sump
                    {
                        if (sumpLevel >= 50 && (GlobalSensorCount.stateT25 == 0 || GlobalSensorCount.stateT75 == 0))
                        {
                            printf("\r\nCOM DT: Sump above 50%% and Tanks Empty - M2 Off");
                            state_ticks_100ms = getTicks100ms();
                            state = MOTOR_OFF;
                            ret_val = 1;
                            motorOffBySensorFlag = 1;
                        }
                        else if (sumpLevel == 100)
                        {
                            printf("\r\nCOM DT: Sump Full and Both Tanks Full - M2 Off");
                            state_ticks_100ms = getTicks100ms();
                            state = MOTOR_OFF;
                            ret_val = 1;
                            motorOffBySensorFlag = 1;
                        }
                    }
                }
            else
#endif
            if (GlobalSensorCount.stateT50 && GlobalSensorCount.stateT100)
            {
                state = MOTOR_OFF;
                ret_val = 1;
                motorOffBySensorFlag = 1;
            }
            if (sourceEmpty)
            {
                if (GlobalSettings.dualMotorEnable && (GlobalSettings.motorMode == MOTOR_MODE_ACO || GlobalSettings.motorMode == MOTOR_MODE_ACC || GlobalSettings.motorMode == MOTOR_MODE_ALT) && runningMotor == 1)
                {
                    printf("\r\nACO DT: Sump Empty, switching to Borewell");
                    Motor1_Off();
                    state_ticks_100ms = getTicks100ms();
                    motorOnInitFlag = 0;
                    state = MOTOR_ON; // Will start Motor 2 after delay
                }
                else
                {
                    // Motor1_Off();
                    printf("\r\nMotor off");
                    state_ticks_100ms = getTicks100ms();
                    if (state == MOTOR_ON_PRIMARY_DRY_RUN_CHECK)
                    {
                        Motor1_Off();
#ifndef MINIMUM_FUNCTIONALITY
                        Motor2_Off();
#endif
                        state = MOTOR_OFF_SYSTEM_IDLE;
                    }
                    else
                    {
                        state = MOTOR_OFF;
                    }
                    ret_val = 1;
                }
            }
        }

        {
            unsigned int currentMotorOnTime = (runningMotor == 2) ? GlobalSettings.motorOnTime2 : GlobalSettings.motorOnTime1;
            if (currentMotorOnTime)
            {
                unsigned long elapsedMinutes = (getTicks100ms() - motorElapsedTicks) / 600;
                if (elapsedMinutes >= currentMotorOnTime)
                {
                    printf("\r\nMotor off - on time exceeded");
                    state_ticks_100ms = getTicks100ms();
                    state = MOTOR_OFF;
                    ret_val = 1;
                }
                else
                {
                    printf("\r\nMotorOnTimeRemaining:%ld ", (long)(currentMotorOnTime - elapsedMinutes));
                }
            }
        }
        {
            unsigned int minV = (GlobalSettings.voltageLimitEnable == 1) ? GlobalSettings.minACVolt : 145;
            unsigned int maxV = (GlobalSettings.voltageLimitEnable == 1) ? GlobalSettings.maxACVolt : 275;
            if (checkLineVoltage(minV, maxV, motorState, GlobalSettings.transformerType) == 0)
            {
                printf("\r\n Voltage Out of Range.");
                voltageFaultCount++;
                if (voltageFaultCount > 5)
                {
                    ret_val = 1;
                    state = MOTOR_OFF_VOLTAGE_FAULT;
                    voltageFaultTicks = getTicks100ms();
                }
            }
            else
            {
                voltageFaultCount = 0;
            }
        }

        if (overCurrentFlag)
        {
            printf("\r\n Overcurrent Tripped. %0.1fA", currentRMS);
            ret_val = 1;
            state = MOTOR_OFF_OVERCURRENT_FAULT;
            voltageFaultTicks = getTicks100ms();
        }
    }
    return ret_val;
}
void buzzerLoop(void)
{
    if (dryRunBuzzerActive)
    {
        if (((getTicks100ms() - dryRunBuzzerStartTicks) / 10) < 120) // 120 seconds = 2 minutes
        {
            BUZZ = !BUZZ;
            return;
        }
        else
        {
            dryRunBuzzerActive = 0;
            BUZZ = 0;
        }
    }

#ifdef ENABLE_TANK_BUZZER
    if (tankEmptyDetectFlag)
    {
        if (((getTicks100ms() - buzzerTank_ticks_100ms) / 10) < 120)
        {
            BUZZ = !BUZZ;
            printf("\r\nBuzzer Timeout: %d", 120 - (getTicks100ms() - buzzerTank_ticks_100ms) / 10);
        }
        else
        {
            BUZZ = 0;
        }
    }
    else if (tankFullDetectFlag)
    {
        if (((getTicks100ms() - buzzerTank_ticks_100ms) / 10) < 120)
        {
            BUZZ = !BUZZ;
            printf("\r\nBuzzer Timeout:%d", 120 - (getTicks100ms() - buzzerTank_ticks_100ms) / 10);
        }
        else
        {
            BUZZ = 0;
        }
    }
    else
    {
        BUZZ = 0;
    }
#endif

#ifdef ENABLE_SUMP_BUZZER
    if (sump1EmptyDetectFlag)
    {
        if (((getTicks100ms() - buzzerSump_ticks_100ms) / 10) < 120)
        {
            BUZZ = !BUZZ;
            printf("\r\nBuzzer Timeout:%d", 120 - (getTicks100ms() - buzzerSump_ticks_100ms) / 10);
        }
        else
        {
            BUZZ = 0;
        }
    }
    else if (sump1FullDetectFlag)
    {
        if (((getTicks100ms() - buzzerSump_ticks_100ms) / 10) < 120)
        {
            BUZZ = !BUZZ;
            printf("\r\nBuzzer Timeout:%d", 120 - (getTicks100ms() - buzzerSump_ticks_100ms) / 10);
        }
        else
        {
            BUZZ = 0;
        }
    }
    else
    {
        BUZZ = 0;
    }
#endif
}