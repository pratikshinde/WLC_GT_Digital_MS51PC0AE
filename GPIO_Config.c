#include "GPIO_config.h"
#include "MS51_32K.h"
#include "Function_define_MS51_32K.h"


void init_GPIO(void)
{
	S_SUMP_25_INIT;
	S_SUMP_50_INIT;
	S_SUMP_75_INIT;
	S_SUMP_100_INIT;

	S_TANK_25_INIT;
	S_TANK_50_INIT;
	S_TANK_75_INIT;
	S_TANK_100_INIT;

	S_DRY_RUN_INIT;

	L_FLOW_1_INIT; 		L_FLOW_1 = 0;
	L_FLOW_2_INIT; 		L_FLOW_2 = 0;
	L_FLOW_3_INIT; 		L_FLOW_3 = 0;
	
	L_MOT1_ON_INIT; 	L_MOT1_ON = 0;
	L_MODE_AUTO_INIT;	L_MODE_AUTO = 0;
	
	O_RLY_0_INIT; 		O_RLY_0 = 0;
	O_RLY_1_INIT; 		O_RLY_1 = 0;
	O_RLY_2_INIT; 		O_RLY_2 = 0;
	O_RLY_3_INIT; 		O_RLY_3 = 0;
	
	BUZZ_INIT;
	BUZZ = 0;
	
}