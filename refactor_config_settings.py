import os

new_code = """
typedef enum {
    DISP_NUMERIC,
    DISP_ENA_DIS,
    DISP_DIS_ENA,
    DISP_OLD_ELE
} ParameterDisplayType;

static uint16_t editParameter(const char* label, uint16_t current_val, uint16_t min_val, uint16_t max_val, uint16_t up_step, uint16_t down_step, ParameterDisplayType dispType)
{
    uint16_t count_val;
    allLEDsOFF();
    setting_ticks_100ms = getTicks100ms();
    while((getTicks100ms() - setting_ticks_100ms) < 20);

    if(GlobalSettings.state)
    {
        printf("\\r\\nSV %s ", label);
        TM1637_WriteString(1, (char*)label);
        if(dispType == DISP_NUMERIC) {
            TM1637_WriteNumber(2, current_val);
            printf("%d", current_val);
        } else if(dispType == DISP_ENA_DIS) {
            TM1637_WriteString(2, current_val ? "ENA" : "DIS");
            printf("%s", current_val ? "Enabled" : "Disabled");
        } else if(dispType == DISP_DIS_ENA) {
            TM1637_WriteString(2, current_val ? "DIS" : "ENA");
            printf("%s", current_val ? "Disabled" : "Enabled");
        } else if(dispType == DISP_OLD_ELE) {
            TM1637_WriteString(2, current_val ? "ELE" : "OLD");
            printf("%s", current_val ? "ELECTRONIC" : "OLD");
        }
    }
    count_val = current_val;
    option_select_ticks_100ms = getTicks100ms();
    
    while((getTicks100ms() - option_select_ticks_100ms) < SETTINGS_SELECT_TIMEOUT)
    {
        if((getTicks100ms() - setting_ticks_100ms) > 1) // basic debounce
        {
            setting_ticks_100ms = getTicks100ms();
            sw_val = readButton();
            if(sw_val == 1) // UP
            {
                option_select_ticks_100ms = getTicks100ms();
                if(dispType != DISP_NUMERIC) { count_val = 1; }
                else {
                    count_val += up_step;
                    if(count_val > max_val) { count_val = (up_step == 1 && min_val == 0) ? 0 : max_val; }
                }
            }
            else if(sw_val == 2) // OK / NEXT
            {
                break;
            }
            else if(sw_val == 3) // DOWN
            {
                option_select_ticks_100ms = getTicks100ms();
                if(dispType != DISP_NUMERIC) { count_val = 0; }
                else {
                    if(count_val >= down_step) { count_val -= down_step; }
                    else { count_val = 0; }
                    
                    if(count_val < min_val) {
                         count_val = min_val;
                    }
                }
            }
            
            if(sw_val == 1 || sw_val == 3) {
                printf("\\r\\n%s ", label);
                TM1637_WriteString(1, (char*)label);
                if(dispType == DISP_NUMERIC) {
                    TM1637_WriteNumber(2, count_val);
                    printf("%d", count_val);
                } else if(dispType == DISP_ENA_DIS) {
                    TM1637_WriteString(2, count_val ? "ENA" : "DIS");
                    printf("%s", count_val ? "Enabled" : "Disabled");
                } else if(dispType == DISP_DIS_ENA) {
                    TM1637_WriteString(2, count_val ? "DIS" : "ENA");
                    printf("%s", count_val ? "Disabled" : "Enabled");
                } else if(dispType == DISP_OLD_ELE) {
                    TM1637_WriteString(2, count_val ? "ELE" : "OLD");
                    printf("%s", count_val ? "ELECTRONIC" : "OLD");
                }
            }
        }
    }
    
    TM1637_WriteString(2, "SAV");
    return count_val;
}

void configSettings(void)
{
    allLEDsOFF();
    setting_ticks_100ms = getTicks100ms();
    while((getTicks100ms() - setting_ticks_100ms) < 20);

    GlobalSettings.powerOnDelay = editParameter("POD", GlobalSettings.powerOnDelay, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.dryRunTimeOut = editParameter("DR1", GlobalSettings.dryRunTimeOut, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.secondaryDryRunDuration = editParameter("DR2", GlobalSettings.secondaryDryRunDuration, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.dryRunReset = editParameter("RST", GlobalSettings.dryRunReset, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.tankOnLevel = editParameter("TON", GlobalSettings.tankOnLevel, 25, 75, 25, 25, DISP_NUMERIC);
    
    uint16_t min_tank_off = GlobalSettings.tankOnLevel + 25;
    GlobalSettings.tankOffLevel = editParameter("TOF", GlobalSettings.tankOffLevel, min_tank_off, 100, 25, 25, DISP_NUMERIC);
    GlobalSettings.dualTankEnable = editParameter("DTK", GlobalSettings.dualTankEnable, 0, 1, 1, 1, DISP_ENA_DIS);
    GlobalSettings.sumpType = editParameter("SUM", GlobalSettings.sumpType, 0, 1, 1, 1, DISP_DIS_ENA);
    GlobalSettings.motorOnTime = editParameter("ONT", GlobalSettings.motorOnTime, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.motorOffTime = editParameter("OFT", GlobalSettings.motorOffTime, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.minACVolt = editParameter("VOL", GlobalSettings.minACVolt, 110, 180, 1, 1, DISP_NUMERIC);
    GlobalSettings.maxACVolt = editParameter("VOH", GlobalSettings.maxACVolt, 270, 300, 1, 1, DISP_NUMERIC);
    GlobalSettings.offTimeTankOffLevel = editParameter("OFF", GlobalSettings.offTimeTankOffLevel, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.relayPulse = editParameter("RPL", GlobalSettings.relayPulse, 0, 999, 1, 1, DISP_NUMERIC);
    GlobalSettings.starterSelection = editParameter("STA", GlobalSettings.starterSelection, 0, 1, 1, 1, DISP_OLD_ELE);
    
    writeSettingEEPROM();
}
"""

with open('WLC_Functions.c', 'r') as f:
    lines = f.readlines()

start_idx = -1
end_idx = -1
for i, line in enumerate(lines):
    if 'void configSettings(void)' in line:
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
    lines.insert(start_idx, new_code + "\\n")
    with open('WLC_Functions.c', 'w') as f:
        f.writelines(lines)
    print("Successfully refactored configSettings.")
else:
    print(f"Could not find configSettings exactly. Start: {start_idx}, End: {end_idx}")
