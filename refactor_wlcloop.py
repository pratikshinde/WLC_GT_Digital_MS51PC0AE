import os

new_code = """static void Handle_ButtonPresses(void) {
	printf("\\r\\nReading Button");
	switch_val = readButton();
	if(sumpLevel<25)
	{
			sumpEmptyDetectFlag = 1;
	}
	if(switch_val== 1)//Motor On Button Pressed, It will also clear dry run fault state
	{
		if((GlobalSettings.workMode==WORKING_MODE_AUTO) && (state != MOTOR_ON_SECONDARY_DRY_CHECK))
		{
			if((tankLevel!=100)&&(motorState==0))
			{
				if(state != MOTOR_OFF_SENSOR_ERROR)
				{
					state_ticks_100ms = getTicks100ms();
					Motor1_On();
					state = MOTOR_ON_PRIMARY_DRY_RUN_CHECK;
				}
			}
			else
			{
				printf("\\r\\nTank Full");
			}
		}
		else if(GlobalSettings.workMode==WORKING_MODE_MANUAL)
		{
			if(state != MOTOR_RUNNING_MANUAL)
			{
				state = MOTOR_ON_MANUAL;
			}
		}
	}
	else if(switch_val==3)//Motor Off Button Pressed
	{
		if(tankEmptyDetectFlag||tankFullDetectFlag)
		{
			tankEmptyDetectFlag = tankFullDetectFlag = 0;
			BUZZ = 0;
			//_delay_ms(2000);	
		}
		else if(sump1EmptyDetectFlag||sump1FullDetectFlag)
		{
			sump1EmptyDetectFlag = sump1FullDetectFlag = 0;
			BUZZ = 0;
			//_delay_ms(2000);	
		}
		else
		{
			if(GlobalSettings.workMode==WORKING_MODE_MANUAL)
			{
				Motor1_Off();
				state = MOTOR_OFF_SYSTEM_IDLE;
			}
			
		}
	}
	else if(switch_val==2)//Mode Change/Middle Button Pressed
	{
		if(GlobalSettings.workMode==WORKING_MODE_AUTO)
		{
			GlobalSettings.workMode=WORKING_MODE_MANUAL;
			printf("\\r\\nMAN");
			//L_MODE_SEL = MODE_LED_MANUAL_RED;
			state = MOTOR_OFF_MANUAL;
			TM1637_WriteString(1,"MOD");
			TM1637_WriteString(2,"MAN");
			L_MODE_AUTO = 1;
			//L_MAN_MODE = 1;L_AUTO_MODE = 0;
		}
		else if(GlobalSettings.workMode==WORKING_MODE_MANUAL)
		{
			GlobalSettings.workMode=WORKING_MODE_AUTO;
			printf("\\r\\nAUT");
			//L_MODE_SEL = MODE_LED_AUTO_GREEN;
			TM1637_WriteString(1,"MOD");
			TM1637_WriteString(2,"ATO");
			L_MODE_AUTO = 0;
			//L_MAN_MODE = 0;L_AUTO_MODE = 1;
		}
		writeSettingEEPROM();
		_delay_ms(5000);
	}
}

static void StateHandler_BootStart(void) {
	if(GlobalSettings.workMode==WORKING_MODE_AUTO)
	{
		printf("\\r\\nState:BOOT_START");
		if(motorOffTime_val)
		{
			printf("\\r\\nPending Motor Off Time: %ld",motorOffTime_val);
			state = MOTOR_OFF_RESET_DELAY;
		}
		else
		{
			if(checkMotorOnConditions())
			{
				printf("\\r\\nMotor On");
			}
			if(checkMotorOffConditions())
			{
				printf("\\r\\nMotor Off");
			}
		}
	}
	else if(GlobalSettings.workMode==WORKING_MODE_MANUAL)
	{
		printf("\\r\\nState:BOOT_START_MANUAL");
		state = MOTOR_OFF_SYSTEM_IDLE;
	}
}

static void StateHandler_MotorOnDelay(void) {
	printf("\\r\\nState:MOTOR_ON");
	if(!motorOnInitFlag)
	{
		motorOnInitFlag = 1;
		state_ticks_100ms = getTicks100ms();
	}
	else
	{
		if(((MOTOR_ON_DELAY) - (getTicks100ms()-state_ticks_100ms)/10)>0)
		{
			printf("\\r\\nMOT On Delay:%d",(MOTOR_ON_DELAY) - (getTicks100ms()-state_ticks_100ms)/10);
		}
		else
		{
			Motor1_On();
			state = MOTOR_ON_PRIMARY_DRY_RUN_CHECK;
			state_ticks_100ms = getTicks100ms();
			motorOnInitFlag = 0;
		}
	}
}

static void StateHandler_PrimaryDryRunCheck(void) {
	printf("\\r\\nState:MOTOR_ON_PRIMARY_DRY_RUN_CHECK");
	if(GlobalSettings.dryRunTimeOut)
	{
		if(checkMotorOffConditions())
		{
			printf("\\r\\nMotor Off");
		}
		if(GlobalSettings.dryRunTimeOut - ((getTicks100ms()-state_ticks_100ms)/10)>0)
		{
			if(!dryRunState)
			{
				printf("\\r\\nMotor Running..");
				flowLEDsMotorOn();
			}
			else
			{
				flowLEDsMotorDryRunCheck();
				printf("\\r\\nPrimary Dry Run Timeout:%ld",(GlobalSettings.dryRunTimeOut) - (getTicks100ms()-state_ticks_100ms)/10);
			}
		}
		else
		{
			if(dryRunState)
			{
				if(GlobalSettings.secondaryDryRunDuration)
				{
					state = MOTOR_ON_SECONDARY_DRY_CHECK;
					state_ticks_100ms = getTicks100ms();
				}
				else
				{
					state = MOTOR_OFF_DRY_RUN_FAULT;
					Motor1_Off();
					printf("\\r\\nDry Run Fault");					
				}
			}
			else if(!dryRunState)
			{
				state = MOTOR_ON_SECONDARY_DRY_CHECK;
				printf("\\r\\nMotor Running..");
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

static void StateHandler_DryRunFault(void) {
	flowLEDsDryRunFault();
	printf("\\r\\nState:MOTOR_OFF_DRY_RUN_FAULT");
	if(!dryRunResetInitFlag)
	{
		dryRunResetInitFlag = 1;
		state_ticks_100ms = getTicks100ms();
		while((getTicks100ms()-state_ticks_100ms)<50);
		Motor1_Off();
	}
	else
	{
		if(((GlobalSettings.dryRunReset*60) - (getTicks100ms()-state_ticks_100ms)/10)>0)
		{
			printf("\\r\\nDry Run Reset Timeout:%ld",(GlobalSettings.dryRunReset*60) - (getTicks100ms()-state_ticks_100ms)/10);
			resetMinutes = ((GlobalSettings.dryRunReset*60) - (getTicks100ms()-state_ticks_100ms)/10)/60;
			if(resetMinutes<1)
			{
				resetMinutes = (GlobalSettings.dryRunReset*60) - (getTicks100ms()-state_ticks_100ms)/10;
				dispCount = 10;
			}
		}
		else
		{
			printf("\\r\\nDry Run Reset Over");
			state = MOTOR_OFF_SYSTEM_IDLE;
		}
		if(reset_press_count)
		{
			state = MOTOR_OFF_SYSTEM_IDLE;
			reset_press_count = 0;
			printf("\\r\\nDry Run Reset Over");
		}
	}
}

static void StateHandler_SecondaryDryRunCheck(void) {
	printf("\\r\\nState:MOTOR_ON_SECONDARY_DRY_CHECK");
	if(dryRunState == 0)
	{
		state_ticks_100ms = getTicks100ms();
		flowLEDsMotorOn();
		printf("\\r\\nMotor Running ");
	}
	else
	{
		if(GlobalSettings.dryRunTimeOut)
		{
			if(((GlobalSettings.secondaryDryRunDuration) - (getTicks100ms()-state_ticks_100ms)/10)>0)
			{
				printf("\\r\\nSecondary Dry Run Timeout:%ld",(GlobalSettings.secondaryDryRunDuration) - (getTicks100ms()-state_ticks_100ms)/10);
				flowLEDsMotorDryRunCheck();
				if(tankLevel >= 100)
				{
					Motor1_Off();
					state = MOTOR_OFF_SYSTEM_IDLE;
					flowLEDsMotorOff();
				}
			}
			else
			{
				state = MOTOR_OFF_DRY_RUN_FAULT;
				printf("\\r\\nDry Run Fault");
			}
		}
		else
		{
			flowLEDsMotorDryRunCheck();
		}
	}
	if(checkMotorOffConditions())
	{
		printf("\\r\\nMotor Off");
	}
}

static void StateHandler_MotorOff(void) {
	printf("\\r\\nState:MOTOR_OFF");
	if(!motorOffInitFlag)
	{
		motorOffInitFlag = 1;
		state_ticks_100ms = getTicks100ms();
	}
	else
	{
		if(((GlobalSettings.offTimeTankOffLevel) - (getTicks100ms()-state_ticks_100ms)/10)>0)
		{
			printf("\\r\\nMOT OFF Time:%ld",(GlobalSettings.offTimeTankOffLevel) - (getTicks100ms()-state_ticks_100ms)/10);
			flowLEDsMotorOn();
		}
		else
		{
			printf("\\r\\nMOT SW OFF");
			L_FLOW_1 = L_FLOW_2 = L_FLOW_3 = 0;
			Motor1_Off();
			if(GlobalSettings.motorOffTime && (motorOffBySensorFlag == 0))
			{
				state = MOTOR_OFF_RESET_DELAY;
				enableMotorOffTimer();
			}
			else
			{
				state = MOTOR_OFF_SYSTEM_IDLE;
			}
		}
		if(dryRunState)
		{
			flowLEDsMotorOff();
		}
	}
}

static void StateHandler_MotorOffResetDelay(void) {
	printf("\\r\\nState:MOTOR_OFF_RESET_DELAY");
	flowLEDsMotorOff();
	checkMotorResetTimeOut();
}

static void StateHandler_VoltageFault(void) {
	printf("\\r\\nState:MOTOR_OFF_VOLTAGE_FAULT");
	if(((VOLTAGE_RESET_TIME) - (getTicks100ms()-voltageFaultTicks)/10)>0)
	{
		printf("\\r\\nVoltage Fault:%ld",(VOLTAGE_RESET_TIME) - (getTicks100ms()-voltageFaultTicks)/10);
		flowLEDsMotorOff();
		Motor1_Off();
		TM1637_WriteString(1,"VOL");
		TM1637_WriteNumber(2,getLineVoltage());
	}
	else
	{
		state = MOTOR_OFF_SYSTEM_IDLE;
	}
}

static void StateHandler_SystemIdle(void) {
	printf("\\r\\nState:MOTOR_OFF_SYSTEM_IDLE");
	flowLEDsMotorOff();  
	motorOnIndicationFlag = motorRunningCount = motorOffInitFlag = dryRunResetInitFlag = 0;
	printf("\\r\\nIdle.");
	printf("S_LVL: %d",sumpLevel);
	printf(" T_LVL: %d",tankLevel);
	if(GlobalSettings.workMode==WORKING_MODE_AUTO)
	{
		checkMotorOnConditions();
	}
}

static void StateHandler_ManualMode(void) {
	if(state == MOTOR_ON_MANUAL)
	{
		Motor1_On();
		state = MOTOR_RUNNING_MANUAL;
	}
	else if(state == MOTOR_RUNNING_MANUAL)
	{
		state = MOTOR_RUNNING_MANUAL;
		printf("\\r\\nMOT Run ");
		if(dryRunState == 0)
		{
			flowLEDsMotorOn();
		}
		else
		{
			flowLEDsMotorDryRunCheck();
		}
		checkMotorOffConditions();
	}
	else if(state == MOTOR_OFF_MANUAL)
	{
		Motor1_Off();
		printf("\\r\\nMOT Off MAN, idle\\r\\n");
		state = MOTOR_OFF_SYSTEM_IDLE;
	}
}

static void StateHandler_SensorError(void) {
	state = MOTOR_OFF_SENSOR_ERROR;
	Motor1_Off();
	printf("\\r\\nSensor Error\\r\\n");
}

void wlcLoop(void) {
	Handle_ButtonPresses();
	switch(state) {
		case BOOT_START: StateHandler_BootStart(); break;
		case MOTOR_ON: StateHandler_MotorOnDelay(); break;
		case MOTOR_ON_PRIMARY_DRY_RUN_CHECK: StateHandler_PrimaryDryRunCheck(); break;
		case MOTOR_ON_SECONDARY_DRY_CHECK: StateHandler_SecondaryDryRunCheck(); break;
		case MOTOR_OFF: StateHandler_MotorOff(); break;
		case MOTOR_OFF_DRY_RUN_FAULT: StateHandler_DryRunFault(); break;
		case MOTOR_OFF_RESET_DELAY: StateHandler_MotorOffResetDelay(); break;
		case MOTOR_OFF_VOLTAGE_FAULT: StateHandler_VoltageFault(); break;
		case MOTOR_OFF_SYSTEM_IDLE: StateHandler_SystemIdle(); break;
		case MOTOR_OFF_SENSOR_ERROR: StateHandler_SensorError(); break;
		case MOTOR_ON_MANUAL:
		case MOTOR_RUNNING_MANUAL:
		case MOTOR_OFF_MANUAL: StateHandler_ManualMode(); break;
		default: break;
	}
}
"""

with open('WLC_Functions.c', 'r') as f:
    lines = f.readlines()

# Find the exact start and end of wlcLoop function
start_idx = -1
end_idx = -1
for i, line in enumerate(lines):
    if 'void wlcLoop(void)' in line:
        start_idx = i
        break

if start_idx != -1:
    brace_count = 0
    in_function = False
    for i in range(start_idx, len(lines)):
        line = lines[i]
        if '{' in line:
            brace_count += line.count('{')
            in_function = True
        if '}' in line:
            brace_count -= line.count('}')
            if in_function and brace_count == 0:
                end_idx = i + 1
                break

if start_idx != -1 and end_idx != -1:
    del lines[start_idx:end_idx]
    lines.insert(start_idx, new_code + "\n")
    with open('WLC_Functions.c', 'w') as f:
        f.writelines(lines)
    print("Successfully refactored wlcLoop.")
else:
    print(f"Could not find wlcLoop exactly. Start: {start_idx}, End: {end_idx}")

