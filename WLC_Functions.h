#ifndef __WLC_functions__
#define __WLC_functions__

// #define ENABLE_SUMP_BUZZER								1
// #define ENABLE_TANK_BUZZER								1
#define MOTOR_ON_DELAY                   10
#define SETTINGS_ENABLE_DOUBLE_OH_TANK   1
#define SETTINGS_DRY_RUN_TIMEOUT         2
#define SETTINGS_SECONDARY_DRY_RUN_DELAY 3
#define SETTINGS_DRY_RUN_RESET           4

#define SETTINGS_TANK_ON_LEVEL           5
#define SETTINGS_TANK_OFF_LEVEL          6
#define SETTINGS_SUMP_ON_LEVEL           7

#define SETTINGS_MOTOR_OFF_DELAY         8
#define SETTINGS_RELAY_PULSE             9

#define SETTINGS_MIN_VOLT                10
#define SETTINGS_MAX_VOLT                11

#define SETTINGS_SUMP_SEDIMENT_DELAY     12
#define SETTINGS_STARTER_SELECTION       13

#define VOLTAGE_RESET_TIME               45
#define VOLTAGE_HYSTERESIS               30

#define MODE_LED_AUTO_GREEN              0
#define MODE_LED_MANUAL_RED              1
#define SETTINGS_SELECT_TIMEOUT          150 // seconds * 10

#define SENSOR_READING_COUNT             6
#define VOLTAGE_FAULT_COUNT              20
#define TH0_INIT                         0x3C
#define TL0_INIT                         0xB0
void setSedimentDelayFlag(void);
char getSedimentDelayFlag(void);
void clearSedimentDelayFlag(void);
void enableSedimentTimer(void);
void updateSensorLED(void);
void refreshCurrentDisplay(void);
void handleDisplayRolling(void);
void allLEDsON(void);
void allLEDsOFF(void);
void configSettings(void);
void configSettings2(void);
void configSettings3(void);
void readSettingsEEPROM(void);
void writeSettingEEPROM(void);
void writeSettingSedimentEEPROM(void);
void readSettingsSedimentEEPROM(void);
void updateSedimentRemainingTime(long);
void Motor1_On();
void Motor1_Off();
void Motor2_On();
void Motor2_Off();
void startSelectedMotor(void);
void wlcLoop(void);
long getRemainingTimeSedimentation(void);
long getRemainingTimeSediSaved(void);
void checkSumpSensorProblem(void);
void disableSedimentTimer(void);
void powerOnDelayCounter(void);
void buzzerLoop(void);
unsigned int checkMotorOnConditions(void);
unsigned int checkMotorOffConditions(void);
unsigned int checkMotorResetTimeOut(void);
void writeSettingMotorOffRemainingEEPROM(void);
void readSettingsMotorOffRemainingEEPROM(void);
long getRemainingTimeMotorOffSaved(void);
int getRemainingOffTime(void);
typedef enum {
    BOOT_START = 1,
    MOTOR_START_AT_POWER_UP,
    MOTOR_ON,
    MOTOR_ON_PRIMARY_DRY_RUN_CHECK,
    MOTOR_ON_SECONDARY_DRY_CHECK,
    MOTOR_OFF_DRY_RUN_RESET,
    MOTOR_OFF_DRY_RUN_FAULT,
    MOTOR_OFF_OVERCURRENT_FAULT,
    MOTOR_OFF,
    MOTOR_OFF_SYSTEM_IDLE,
    MOTOR_OFF_RESET_DELAY,
    MOTOR_ON_MANUAL,
    MOTOR_OFF_MANUAL,
    MOTOR_RUNNING_MANUAL,
    MOTOR_OFF_VOLTAGE_FAULT,
    MOTOR_OFF_SENSOR_ERROR
} SYSTEM_STATES;

typedef enum {
    MOTOR_STATE_ON = 1,
    MOTOR_STATE_STARTING,
    MOTOR_STATE_POWER_DOWN,
    MOTOR_STATE_OFF

} MOTOR_STATES;

typedef enum {
    WORKING_MODE_AUTO = 1,
    WORKING_MODE_MANUAL,

} WORKING_MODE;

typedef enum {
    MOTOR_MODE_ACO = 0,
    MOTOR_MODE_SAL,
    MOTOR_MODE_SUM,
    MOTOR_MODE_BOR,
    MOTOR_MODE_COM,
    MOTOR_MODE_ACC
} MOTOR_MODES;

#endif