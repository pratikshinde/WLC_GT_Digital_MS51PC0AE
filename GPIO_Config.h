#ifndef __GPIO_CONFIG___
#define __GPIO_CONFIG___

#include "MS51_32K.h"
#include "Function_define_MS51_32K.h"

sbit P21 = P2 ^ 1;
sbit P22 = P2 ^ 2;
sbit P23 = P2 ^ 3;
sbit P24 = P2 ^ 4;
sbit P25 = P2 ^ 5;

/* Sensor Inputs Definition*/
#define S_SUMP_25  P33
#define S_SUMP_50  P01
#define S_SUMP_75  P00
#define S_SUMP_100 P10

#define S_TANK_25  P11
#define S_TANK_50  P12
#define S_TANK_75  P32
#define S_TANK_100 P31

#define S_DRY_RUN  P35

/* Output Definitions */
#define L_FLOW_1    P17
#define L_FLOW_2    P15
#define L_FLOW_3    P25

#define L_MOT1_ON   P02
#define L_MODE_AUTO P16

#define DISP_IO     P37
#define DISP_CLK    P36

#define O_RLY_0     P22
#define O_RLY_1     P23
#define O_RLY_2     P24
#define O_RLY_3     P13

#define BUZZ        P34

/* ADC Inpus */
/* Buttons connected to P05 ADC_CH2 */
/* AC_Voltage_Sense connected to P07 ADC_CH6 */
/* CT Coil Connected to P06 ADC_CH3 */
/* Mode Selection Switch connected to P04 ADC_CH5 */

/* GPIO Inititalizing MACROs*/
#define S_SUMP_25_INIT  P33_PULLUP_ENABLE
#define S_SUMP_50_INIT  P01_PULLUP_ENABLE
#define S_SUMP_75_INIT  P00_PULLUP_ENABLE
#define S_SUMP_100_INIT P10_PULLUP_ENABLE

#define S_TANK_25_INIT  P11_PULLUP_ENABLE
#define S_TANK_50_INIT  P12_PULLUP_ENABLE
#define S_TANK_75_INIT  P32_PULLUP_ENABLE
#define S_TANK_100_INIT P31_PULLUP_ENABLE

#define S_DRY_RUN_INIT  P35_PULLUP_ENABLE

/* Output Definitions */
#define L_FLOW_1_INIT    P17_PUSHPULL_MODE
#define L_FLOW_2_INIT    P15_PUSHPULL_MODE
#define L_FLOW_3_INIT    P25_PUSHPULL_MODE

#define L_MOT1_ON_INIT   P02_PUSHPULL_MODE
#define L_MODE_AUTO_INIT P16_PUSHPULL_MODE

#define O_RLY_0_INIT     P22_PUSHPULL_MODE
#define O_RLY_1_INIT     P23_PUSHPULL_MODE
#define O_RLY_2_INIT     P24_PUSHPULL_MODE
#define O_RLY_3_INIT     P13_PUSHPULL_MODE

#define BUZZ_INIT        P34_PUSHPULL_MODE

void init_GPIO(void);
#endif
