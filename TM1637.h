#ifndef _TM1637_H_
#define _TM1637_H_
#include "Function_Define_MS51_32K.h"
#include "MS51_32K.h"
#include "GPIO_config.h"
#include "delay.h"
// Main Settings
#define TM1637_DIO_PIN        P37
#define TM1637_CLK_PIN        P36
#define TM1637_DELAY_US       (5)
#define TM1637_BRIGHTNESS_MAX (7)
#define TM1637_POSITION_MAX   (6)

// TM1637 commands
#define TM1637_CMD_SET_DATA    0x40
#define TM1637_CMD_SET_ADDR    0xC0
#define TM1637_CMD_SET_DSIPLAY 0x80

// TM1637 data settings (use bitwise OR to contruct complete command)
#define TM1637_SET_DATA_WRITE  0x00 // write data to the display register
#define TM1637_SET_DATA_READ   0x02 // read the key scan data
#define TM1637_SET_DATA_A_ADDR 0x00 // automatic address increment
#define TM1637_SET_DATA_F_ADDR 0x04 // fixed address
#define TM1637_SET_DATA_M_NORM 0x00 // normal mode
#define TM1637_SET_DATA_M_TEST 0x10 // test mode

// TM1637 display control command set (use bitwise OR to consruct complete command)
#define TM1637_SET_DISPLAY_OFF 0x00 // off
#define TM1637_SET_DISPLAY_ON  0x08 // on

/**
 * Initialize TM1637 display driver.
 * Clock pin (TM1637_CLK_PIN) and data pin (TM1637_DIO_PIN)
 * are defined at the top of this file.
 */
void TM1637_init(const uint8_t enable, const uint8_t brightness);

/**
 * Turn display on/off.
 * value: 1 - on, 0 - off
 */
void _delay_ms(uint16_t delayms);
void TM1637_WriteString(uint8_t lineNumber, char *str);
void TM1637_enable(const uint8_t value);
void TM1637_WriteNumber(uint8_t lineNumber, uint16_t number);
void TM1637_WriteNumberDP(uint8_t lineNumber, uint16_t number);
void TM1637_WriteNumberCurrent(uint8_t lineNumber, uint16_t number);
void TM1637_WriteDualSumpStatus(char sump1, char sump2);
/**
 * Set display brightness.
 * Min value: 0
 * Max value: 7
 */
void TM1637_set_brightness(const uint8_t value);
void TM1637_Continous_DataTest();
/**
 * Display raw segments at position (0x00..0x03)
 *
 *      bits:
 *        -- 0 --
 *       |       |
 *       5       1
 *       |       |
 *        -- 6 --
 *       |       |
 *       4       2
 *       |       |
 *        -- 3 --
 *
 * Example segment configurations:
 * - for character 'H', segments=0b01110110
 * - for character '-', segments=0b01000000
 * - etc.
 */
void TM1637_display_segments(const uint8_t position, const uint8_t segments);

/**
 * Display digit ('0'..'9') at position (0x00..0x03)
 */
void TM1637_display_digit(const uint8_t position, const uint8_t digit);
void TM1637_Write6Digits();
/**
 * Display colon on/off.
 * value: 1 - on, 0 - off
 */
void TM1637_display_colon(const uint8_t value);

/**
 * Clear all segments (including colon).
 */
void TM1637_clear(void);

#endif /* !_ATTINY_TM1637_H_ */