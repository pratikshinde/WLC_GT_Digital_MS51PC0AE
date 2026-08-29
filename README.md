# Water Level Controller Firmware (Indicator Model V1)

This project contains the firmware for a dual-motor and dual-source Water Level Controller developed on the Nuvoton MS51 series (MS51PC0AE). It is designed to intelligently manage water transfer between a Borewell, a Sump, and an Overhead Tank.

## Core Features
- **Dual Motor Support**: Independent control of two motors (Sump Pump and Borewell Pump) via dedicated relays.
- **Advanced Operation Modes**: Six distinct modes to handle various plumbing architectures:
    - **ACO (Auto Change Over)**: Primary Sump operation with automatic fallback to Borewell if the Sump is dry.
    - **ACC (Auto Change Over with Cycles)**: Combines configurable cycle-based alternation with automatic source switching (ACO). Supports M1:M2 cycle ratio (e.g., 2:4).
    - **ALT (Alternate)**: Alternates between two sources/motors with simple 1:1 logic.
    - **SUM (Sump Mode)**: Dedicated single-source Sump-to-Tank operation.
    - **BOR (Borewell Mode)**: Dedicated single-source Borewell-to-Tank operation.
    - **COM (Combo Mode)**: Intelligent 3-stage logic (Borewell -> Sump -> Tank).
    - **SAL (Sump Alternate)**: Both motors draw from the same Sump and run alternatively using 1:1 logic. If one motor faults, the other motor runs. If both run dry, it goes to dry run reset timeout.
- **Dual Tank Support**: Ability to monitor and manage two separate destination tanks using shared sensor inputs.
- **Dual Sump Support**: Ability to monitor two separate source sumps.
- **Independent Timing**: Separate configurable "Motor Off Time" settings for Motor 1 and Motor 2.
- **Failover Protection**: Automatic motor switching during dry run faults.
- **Voltage Safety**: Real-time AC voltage monitoring with trip points.
- **Current Sensing (Optional)**: Support for CT coil to monitor motor load (Amperes) and detect dry run/overload.

---

## Input Mapping

The system uses 8 physical sensor inputs which can be configured for either high-resolution (4 levels) or multi-reservoir (2 levels each) monitoring.

### Standard Mode (Single Sump & Single Tank)
- **Sump Sensors**: S25, S50, S75, S100 (Source level: 25%, 50%, 75%, 100%)
- **Tank Sensors**: T25, T50, T75, T100 (Destination level: 25%, 50%, 75%, 100%)

### Dual Sump Mode (`SUM` = 2)
Splits the sump inputs to monitor two reservoirs:
- **Sump 1 (SM1)**: Uses `S25` (50%) and `S50` (100%)
- **Sump 2 (SM2)**: Uses `S75` (50%) and `S100` (100%)

### Dual Tank Mode (`DTK` = ENA)
Splits the tank inputs to monitor two destination tanks:
- **Tank 1 (TN1)**: Uses `T25` (50%) and `T50` (100%)
- **Tank 2 (TN2)**: Uses `T75` (50%) and `T100` (100%)

---

## Hardware Configuration

### Output Mapping
- **Relay 0 & 1**: Motor 1 Control (Primary Sump Motor).
- **Relay 2 & 3**: Motor 2 Control (Borewell or Alternate Motor).
- **TM1637 Display**: 4-digit 7-segment display for UI feedback.
- **LED Indicators**: Status LEDs for flow, power, and faults.

---

## System Architecture

```mermaid
graph TD
    MCU[MS51PC0AE Microcontroller]
    
    subgraph Inputs
        S_Sensors[Sump Sensors: Single or Dual]
        T_Sensors[Tank Sensors: Single or Dual]
        AC_Volt[AC Voltage Sensor]
        CT_Sens[Current Sensor - Optional]
    end

    subgraph Modes
        ACO[Auto Change Over]
        ALT[Alternate]
        COM[Combo Logic]
        DTK[Dual Tank Logic]
    end

    subgraph Outputs
        RLY1[Motor 1: Sump]
        RLY2[Motor 2: Borewell]
        DISP[TM1637 Display]
        UART[Serial Debug]
    end
    
    S_Sensors --> MCU
    T_Sensors --> MCU
    AC_Volt --> MCU
    CT_Sens --> MCU
    
    MCU --> Modes
    MCU --> RLY1
    MCU --> RLY2
    MCU --> DISP
    MCU --> UART
```

---

## User Interface

The device features a 4-digit TM1637 display and three user buttons for configuration and operation.

### Buttons
- **AUTO**: Switches the system to Automatic mode. In this mode, the controller manages the pumps based on water levels and voltage conditions.
- **MANUAL**: Switches the system to Manual mode. Allows the user to start/stop motors manually using the UP/DOWN buttons.
- **SET**: Enters the configuration menu. Press SET to cycle through parameters, and use the other buttons to adjust values.

### Display Rolling
In **Dual Tank** or **Dual Sump** modes, the display rolls every 3 seconds to show different reservoir levels:
- `SM1` / `SM2`: Sump 1 and Sump 2 levels.
- `TN1` / `TN2`: Tank 1 and Tank 2 levels.
- `BOR`: Displayed when in Borewell mode.
- `AMP`: Shows the motor current in Amperes (if CT is enabled).

---

## Software State Machine
The firmware utilizes a robust state machine to manage transitions:

1.  **MOTOR_OFF_SYSTEM_IDLE**: Monitoring levels and waiting for start conditions.
2.  **MOTOR_ON_PRIMARY_DRY_RUN_CHECK**: Initial start-up phase where water flow is expected.
3.  **MOTOR_ON_SECONDARY_DRY_CHECK**: Extended check if flow is delayed.
4.  **MOTOR_OFF_DRY_RUN_FAULT**: Emergency shutdown if no water is detected.
5.  **MOTOR_OFF_VOLTAGE_FAULT**: Shutdown due to unsafe AC line voltage.
6.  **MOTOR_OFF_OVERCURRENT_FAULT**: Shutdown due to high motor current.
7.  **MOTOR_OFF_RESET_DELAY**: Configurable cooldown period after a cycle or fault.
8.  **MOTOR_OFF_SENSOR_ERROR**: Safety shutdown if sensor wiring is inconsistent.

---

## File Structure
- `WLC_Indicator_model.c`: Main entry point and system loop.
- `WLC_Functions.c / .h`: Core logic for water level management, modes, and settings.
- `GPIO_Config.c / .h`: Hardware pin assignments and initialization.
- `ADC_inputs.c / .h`: Drivers for ADC sensing (Voltage, Buttons, Current Transformer).
- `TM1637.c / .h`: Display driver for the 7-segment LED module.
- `Timer3_Int.c / .h`: System tick timer (100ms) for timing and delays.

---

## Configuration Settings
Accessible via the onboard UI (SET button):

| Parameter | Description | Options |
| :--- | :--- | :--- |
| **POD** | Power On Delay | 0 - 999 Seconds |
| **DR1 / DR2** | Dry Run Timeouts | Primary / Secondary timeouts |
| **RST** | Dry Run Reset Time | Minutes until auto-restart |
| **TON / TOF** | Tank On/Off Levels | Start/Stop thresholds (e.g., 50/100) |
| **DTK** | Dual Tank Mode | ENA (Enable) / DIS (Disable) |
| **SUM** | Sump Type | BOR (Bore only), 1 (Single), 2 (Dual) |
| **MO1 / MF1** | Motor 1 Timing | On Time / Off Timer (Minutes) |
| **VOL / VOH** | AC Voltage Range | Min/Max trip points |
| **OFF** | Off-time Tank Level | Level to trigger off-timer reset |
| **RPL** | Relay Pulse | Duration for pulse-start starters |
| **STA** | Starter Type | OLD (Mechanical) / ELE (Electronic) |
| **DM** | Dual Motor Enable | ENA / DIS |
| **MOD** | Operational Mode | ACO, ACC, ALT, SUM, BOR, COB, SAL (0-6) |
| **CY1** | Motor 1 Cycles (ACC only) | Number of Motor 1 runs per alternation |
| **CY2** | Motor 2 Cycles (ACC only) | Number of Motor 2 runs per alternation |
| **CTE** | CT Enable | Enable current sensing |
| **AM1** | Reference Current 1 | Motor 1 rated current |
| **AM2** | Reference Current 2 | Motor 2 rated current |

---

## Build Instructions
1. Open the project using **Keil µVision**.
2. Select the **Nuvoton MS51PC0AE** target.
3. Ensure the **HIRC 24MHz** internal oscillator is selected.
4. Compile the project and flash using a **Nu-Link** debugger.

---

## Recent Improvements & Stability
The firmware has recently undergone a comprehensive stability and logic audit to ensure reliability in complex configurations.

### Reliability & Core Fixes
- **Atomic Timing**: Implemented interrupt-safe 32-bit tick reads to prevent "torn" values on the 8-bit MCU.
- **Underflow Protection**: Fixed multiple unsigned arithmetic underflow bugs in state machine timeouts to ensure reliable state transitions.
- **Boot Stability**: Corrected uninitialized timer variables at boot and replaced fragile magic-number loop escapes with robust flags.
- **AC Voltage Optimization**: 
    - Reduced fault detection latency from 4 seconds to ~1 second for better hardware protection.
    - Improved sensing accuracy by implementing 20-sample averaging over a full 10ms (100Hz) ripple cycle, combined with an IIR low-pass filter (alpha = 0.7).
    - **Voltage Recovery Hysteresis**: Implemented a 30V hysteresis for lower-limit recovery to prevent rapid "chatter" cycling. The system now requires the voltage to reach `minACVolt + 30V` before clearing a fault.
- **Display Stability**: Centralized display update logic into a single controller function to eliminate flickering during fault states.
- **EEPROM Relocation**: Relocated parameter storage blocks to the end pages of the 32KB flash (`0x7F00` and `0x7F80`) to expand compiled code boundaries up to `0x7EFF`, allowing full serial UART debug prints to fit cleanly.
- **Motor 2 Start Loop Fix (Combo Mode)**: Guarded start conditions with `!motor2FaultFlag` and disabled `sourceEmpty` checks for Motor 2 in dual-sump configurations when running in COB/ACO/BOR modes. This ensures dry-run faults reliably trigger the cooldown countdown rather than immediately restarting the motor.
- **Motor Off Guard & Timer Fix**: Added state guards inside `Motor1_Off` and `Motor2_Off` to ensure the turn-off sequence and TM1637 display settling delay execute exactly once per transition. This resolves the bug where the display was repeatedly reset and the `motorResetTicks` variable was constantly overwritten, preventing the motor cooldown/reset timers from counting down.
- **MOTOR_MODE_COM (Combo Mode) Enhancements**:
    - Guarded Bore-to-Sump (Motor 2) start triggers with `sumpEmptyDetectFlag == 1` so the system only switches to the borewell pump once the sump has run dry, rather than proactively draining it.
    - Resolved starting/stopping oscillations at 50% sump and 50% tank levels by checking if the tank has reserves (>= 50%); if the tank has reserves, the borewell fills the sump to 100%, and if the tank is low, the borewell stops at 50%.
    - Whitelisted `MOTOR_MODE_COM` in `BOOT_START` and corrected Selected Motor selection so that if water is present (`sumpLevel >= 25`) and the tank is not full, it immediately runs Motor 1.
- **COB Mode Deadlock Fix**: Fixed a critical deadlock in Combo (COB) mode where neither motor would start after Motor 1 (Sump→Tank) drained the sump and raised the tank past `tankOnLevel`. The Borewell trigger condition was changed from `tankLevel < tankOnLevel` to `tankLevel < tankOffLevel`, ensuring the borewell pump refills the sump whenever the sump is empty and the tank is not yet full.
- **Differentiated & Decoupled Dry Run Displays/Alarms**: 
    - Updated the 7-segment display logic to show `RS1` or `RS2` depending on which motor (Motor 1 or Motor 2) triggered the dry-run fault, showing `RST` if both motors run dry.
    - Decoupled the dry-run buzzer and display warnings from the final shutdown state. The buzzer now sounds for 2 minutes whenever *any* motor runs dry, even during automatic switchover to the alternate motor (in ACO, SAL, ALT, COB, or ACC modes). During this 2-minute alarm period, digits 1 and 2 of the TM1637 display rotate `RS1`/`RS2` to indicate which motor faulted, alternating with the running motor's levels and current.
- **Dynamic Dry-Run Switchover (Combo Mode)**: Instead of transitioning to dry-run fault shutdown immediately when a single motor runs dry in Combo (COM/COB) mode, the system now attempts to switch over and run the alternate motor (e.g. running Motor 2 to fill the sump if Motor 1 is faulted, or vice-versa) if it is healthy and needed. It falls back to the dry-run fault reset countdown only if both motors are faulted, or the alternate motor cannot be started.
- **Sump Type (SUM) setting in Minimum Functionality mode**: Exposed the Sump Type (SUM) parameter configuration in `MINIMUM_FUNCTIONALITY` mode (restricting options to `0` - Borewell and `1` - Single Sump). When configured as a Borewell (`sumpType == 0`), sump level checks are bypassed (`sumpLevel` is set to `50` so it has water), and `"BOR"` is displayed on Digit 2 of the display instead of the level. Also capped `sumpType` to `1` in read settings/loading rather than blindly hardcoding it to `1` at boot.
- **Shivappa Digital Board Support**: Added custom pin configuration selector (`SHIVAPPA_DIGITAL`) in `GPIO_Config.h` to support alternative GPIO mappings for Sump/Tank level inputs and Dry Run signals.
- **Display Overlap and Reset Logic Fix**: Fixed single-tank level and off-time display sequence issues during `MOTOR_OFF_RESET_DELAY` state, aligning the alternating cycle to prevent overlapping values.
- **Minimum Functionality Configuration**: Active compilation of the `MINIMUM_FUNCTIONALITY` mode (configured in `WLC_Functions.h`) to fit within ROM space limits and optimize active parameters.



### Logic Enhancements
- **ALT Mode Independent Rest & Failover**: 
    - Transitioned from a global system rest to independent per-motor rest tracking.
    - Fixed a critical bug where `motorOffBySensorFlag` was not resetting, which previously blocked rest timers from starting.
    - Added real-time second-by-second countdown logs for resting motors.
    - **Sump-Empty Failover**: Resolved a bug in Alternate (ALT) mode where the Borewell motor (Motor 2) would fail to start or switch over when the Sump was empty. Added ALT to the startup bypass whitelist and runtime switchover logic.
- **Full Dual-Tank Support**: Restored missing operational paths for Dual-Tank mode, including automatic boot-starting, ACO/BOR mode bypass, and COM mode replenishment logic.
- **Enhanced Failover**: Extended dry-run failover logic to support all dual-motor modes, ensuring the system can automatically switch pumps if the primary motor faults.
- **Settings Transparency**: Fixed bit-width artifacts in debug logs (e.g., DualMotorEnable: 256) and added explicit Motor Mode reporting.
- **New ACC Mode (Auto Change Over + Cycles)**: 
    - Combined configurable M1:M2 cycle ratio with ACO's automatic source switching.
    - User sets `CY1` and `CY2` to configure cycles (e.g., CY1=2, CY2=4 means Motor1 runs 2 times then Motor2 runs 4 times).
    - Implemented "Motor 2 Sump Bypass": Motor 2 (Borewell) now operates without checking sump sensors, while Motor 1 (Sump) maintains strict level safety.
    - Added automatic in-cycle switching: If Motor 1 runs dry, the system seamlessly transitions to Motor 2 to complete the tank fill.
- **ALT Mode Simplified**: 
    - ALT mode now uses simple 1:1 alternation without cycle configuration.
    - Per-motor independent rest timers (MF1/MF2) are supported in both ALT and ACC modes.
- **Sump Indicator Relay (Single Motor / Single Sump)**:
    - Repurposed the unused `O_RLY_2` relay in Single Motor (`dualMotorEnable == 0`) and Single Sump (`sumpType == 1`) configurations to serve as a sump low-level indicator.
    - The relay turns ON (active-low, set to `0`) when `sumpLevel` falls below 50%, and turns OFF (set to `1`) when `sumpLevel` is 50% or above.
    - Protected the repurposed `O_RLY_2` pin from being reset during calls to `Motor2_Off()`.
- **Default Settings Adjustments**:
    - Changed default `minACVolt` setting from 180V to 140V for wider voltage tolerance by default.
- **Unified CT Coil Support**:
    - Optimized hardware usage by supporting a single CT coil on ADC Channel 9 for both motors.
    - Added independent reference current settings (`AM1` and `AM2`) to allow accurate monitoring of motors with different power ratings.
    - Implemented automatic reference switching in the monitoring loop based on the active motor.
- **Motor Off-Timer (MF1 / MF2) Fixes & Live Countdown**:
    - Fixed off-timer enforcement: Configured motor off-times (`MF1` / `MF2`) are now unconditionally observed and accurately timed regardless of stop reasons (sensor cutoff, dry-run, or on-time expiry).
    - Initialized `motorResetTicks` timestamp accurately upon state transition to ensure precise second-by-second cooldown.
    - Added live real-time countdown output (`Motor Off Time Remaining: <sec>`) via UART during `MOTOR_OFF_RESET_DELAY`.
    - Added button press manual override in `Handle_ButtonPresses()` to clear active off-timers when switching between AUTO and MANUAL modes.
### Changes Made on August 29, 2026 (2026-08-29)

- **Dual Motor Rest & Off-Time TM1637 Rolling Display (`OF1` / `OF2`)**:
    - Implemented dedicated remaining off-time calculation and multi-screen rolling feedback on the TM1637 6-digit (2x3) display for dual motor configurations.
    - When both motors are in rest or OFF time (`motor1InRest && motor2InRest`), the display rolls every 3 seconds:
      - **Screen 1**: Reservoir Levels (Tank % on Line 1, Sump % on Line 2, or Dual Tank / Dual Sump sequences).
      - **Screen 2**: Motor 1 Off Time -> `OF1` on Line 1 (top), remaining off-time in minutes with decimal point on Line 2 (bottom).
      - **Screen 3**: Motor 2 Off Time -> `OF2` on Line 1 (top), remaining off-time in minutes with decimal point on Line 2 (bottom).
    - If only a single motor is in rest, the display automatically shows only the active resting motor's countdown (`OF1` or `OF2`).
    - Single Motor mode continues displaying `OFF` with its single countdown.

- **COM/COB (Combo Mode) Borewell Sump Replenishment & Off-Time Completion**:
    - Fixed premature 10-second Bore motor cutoff in Combo mode (`MOTOR_MODE_COM`).
    - When Motor 1 (Sump to Tank) stops and goes to rest (`MF1`), Motor 2 (Borewell) starts to refill the sump.
    - If OFF times (`MF1` / `MF2`) are set, Motor 2 is no longer prematurely stopped at 50% sump level. It continues running to fill the sump until:
      1. **Sump reaches 100% full** (`sumpLevel >= 100`), OR
      2. **Motor 1 finishes its rest/off-time** (`!motor1InRest` with water present and tank not full), allowing Motor 1 to take over and resume pumping to the tank, OR
      3. **Motor 2's on-time (`MO2`) expires**.
    - When Motor 2 stops, it transitions into its configured rest period (`MF2`).
    - When OFF times are not set (`motorOffTime1 == 0 && motorOffTime2 == 0`), standard cutoff (50% sump when tank < 50%) is preserved.

- **Shivappa Digital Board Hardware Profile (`SHIVAPPA_DIGITAL`)**:
    - Configured `#define SHIVAPPA_DIGITAL` in `GPIO_Config.h` supporting dedicated board pin assignments:
      - Sump Level Inputs: 25% on `P1.0`, 50% on `P0.0`, 75% on `P0.1`, 100% on `P3.3`.
      - Tank Level Inputs: 25% on `P3.5`, 50% on `P3.1`, 75% on `P3.2`, 100% on `P1.2`.
      - Dry Run Sensor Input: `P1.1`.

- **Flash Memory & String Optimization**:
    - Streamlined verbose debug strings across `readSettingsEEPROM`, `StateHandler_VoltageFault`, `StateHandler_DryRunFault`, and `checkMotorOffConditions`.
    - Eliminated ROM overflow errors, enabling clean builds within the 32 KB budget (0 Errors, 0 Warnings) with both standard and full-functionality configurations.


