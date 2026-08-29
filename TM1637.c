#include "tm1637.h"

#define TM1637_DIO_HIGH() (TM1637_DIO_PIN = 1)
#define TM1637_DIO_LOW() (TM1637_DIO_PIN = 0)
#define TM1637_DIO_OUTPUT() P36_QUASI_MODE
#define TM1637_DIO_INPUT() P36_QUASI_MODE
#define TM1637_DIO_READ() (P36)
#define TM1637_CLK_HIGH() (TM1637_CLK_PIN = 1)
#define TM1637_CLK_LOW() (TM1637_CLK_PIN = 0)

void TM1637_send_config(const uint8_t enable, const uint8_t brightness);
void TM1637_send_command(const uint8_t value);
void TM1637_start(void);
void TM1637_stop(void);
uint8_t TM1637_write_byte(uint8_t value);

uint8_t _config = TM1637_SET_DISPLAY_ON | TM1637_BRIGHTNESS_MAX;
uint8_t _segments = 0xff;
uint8_t _digitsData[6];
uint8_t _digit2segments[] =
    {
        0x3F, // 0
        0x06, // 1
        0x5B, // 2
        0x4F, // 3
        0x66, // 4
        0x6D, // 5
        0x7D, // 6
        0x07, // 7
        0x7F, // 8
        0x6F  // 9
};
uint8_t _char2segments[] =
    {
        0x77, // 065 A
        0x7c, // 066 B
        0x39, // 067 C
        0x5e, // 068 D
        0x79, // 069 E
        0x71, // 070 F
        0x3d, // 071 G
        0x76, // 072 H
        0x06, // 073 I
        0x1e, // 074 J
        0x72, // 075 K
        0x38, // 076 L
        0x55, // 077 M (For display use Mm)
        0x37, // 078 N
        0x3F, // 079 O
        0x73, // 080 P
        0x67, // 081 Q
        0x31, // 082 R
        0x6D, // 083 S
        0x78, // 084 T
        0x3e, // 085 U
        0x3e, // 086 V
        0x3c, // 087 W (For display use Ww)
        0x76, // 088 X
        0x6e, // 089 Y
        0x5b  // 090 Z

};
void _delay_us(uint8_t delayUs)
{
    Timer0_Delay(24000000, delayUs, 1);
}
void _delay_ms(uint16_t delayms)
{
    Timer0_Delay(24000000, delayms, 1000);
}
void TM1637_init(const uint8_t enable, const uint8_t brightness)
{

    P37_QUASI_MODE;
    P36_PUSHPULL_MODE;
    TM1637_DIO_PIN = 0;
    TM1637_CLK_PIN = 0;
    TM1637_send_config(enable, brightness);
    TM1637_clear();
}

#if 0
void TM1637_enable(const uint8_t value)
{
    TM1637_send_config(value, _config & TM1637_BRIGHTNESS_MAX);
}

void TM1637_set_brightness(const uint8_t value)
{
    TM1637_send_config(_config & TM1637_SET_DISPLAY_ON, value & TM1637_BRIGHTNESS_MAX);
}
#endif

#if 0
void TM1637_display_segments(const uint8_t position, const uint8_t segments)
{
    TM1637_send_command(TM1637_CMD_SET_DATA | TM1637_SET_DATA_F_ADDR);
    TM1637_start();
    TM1637_write_byte(TM1637_CMD_SET_ADDR | (position & (TM1637_POSITION_MAX - 1)));
    TM1637_write_byte(segments);
    TM1637_stop();
}
#endif

#if 0
void TM1637_display_digit(const uint8_t position, const uint8_t digit)
{
    uint8_t segments = (digit < 10 ? (_digit2segments[digit]) : 0x00);
    /*if (position == 0x01)
    {
        segments = segments | (_segments & 0x80);
        _segments = segments;
    }*/
    TM1637_display_segments(position, segments);
}
#endif

void TM1637_clear(void)
{
    uint8_t i;
    for (i = 0; i < TM1637_POSITION_MAX; ++i)
    {
        // TM1637_display_segments(i, 0xFF);
        _digitsData[i] = 0xFF;
    }
    TM1637_Write6Digits();
}

void TM1637_send_config(const uint8_t enable, const uint8_t brightness)
{

    _config = (enable ? TM1637_SET_DISPLAY_ON : TM1637_SET_DISPLAY_OFF) | (brightness > TM1637_BRIGHTNESS_MAX ? TM1637_BRIGHTNESS_MAX : brightness);
    TM1637_send_command(TM1637_CMD_SET_DSIPLAY | _config);
}

void TM1637_send_command(const uint8_t value)
{
    TM1637_start();
    TM1637_write_byte(value);
    TM1637_stop();
}

void TM1637_start(void)
{
    TM1637_DIO_HIGH();
    TM1637_CLK_HIGH();
    _delay_us(TM1637_DELAY_US);
    TM1637_DIO_LOW();
}

void TM1637_stop(void)
{

    TM1637_CLK_LOW();
    _delay_us(TM1637_DELAY_US);

    TM1637_DIO_LOW();
    _delay_us(TM1637_DELAY_US);

    TM1637_CLK_HIGH();
    _delay_us(TM1637_DELAY_US);

    TM1637_DIO_HIGH();
}
#if 0
void TM1637_Continous_DataTest()
{
    uint8_t n = 0, j = 0;

    for (n = 0; n < 24; n++)
    {
        _digitsData[0] = _digit2segments[j];
        _digitsData[1] = _digit2segments[j + 1];
        _digitsData[2] = _digit2segments[j + 2];
        j++;
        if (j > 7)
        {
            j = 0;
        }

        _digitsData[3] = _char2segments[n];
        _digitsData[4] = _char2segments[n + 1];
        _digitsData[5] = _char2segments[n + 2];

        TM1637_Write6Digits();
        _delay_ms(500);
    }
}
#endif
void TM1637_WriteString(uint8_t lineNumber, char *str)
{
    uint8_t i;
    uint8_t nullReached = 0;
    for (i = 0; i < 3; i++)
    {
        uint8_t seg = 0x00;
        if (!nullReached)
        {
            if (str[i] == '\0')
            {
                nullReached = 1;
            }
            else
            {
                char c = str[i];
                if (c >= '0' && c <= '9')
                {
                    seg = _digit2segments[c - '0'];
                }
                else if (c >= 'A' && c <= 'Z')
                {
                    seg = _char2segments[c - 'A'];
                }
                else if (c >= 'a' && c <= 'z')
                {
                    seg = _char2segments[c - 'a'];
                }
                else if (c == '-')
                {
                    seg = 0x40; // G segment (dash)
                }
                else if (c == '_')
                {
                    seg = 0x08; // D segment (underscore)
                }
                else
                {
                    seg = 0x00; // blank/unknown
                }
            }
        }

        if (lineNumber == 1)
        {
            _digitsData[i + 3] = seg;
        }
        else if (lineNumber == 2)
        {
            _digitsData[i] = seg;
        }
        else
        {
            _digitsData[i + 3] = seg;
        }
    }

    TM1637_Write6Digits();
}
void TM1637_WriteNumberDP(uint8_t lineNumber, uint16_t number)
{
    uint8_t digits[3] = {0}, i = 0;
    if (number > 0 && number < 1000)
    {
        while (number)
        {
            // printf("%d\n", score % 10);
            digits[i++] = number % 10;
            number /= 10;
        }
    }
    if (lineNumber == 1)
    {

        _digitsData[3] = _digit2segments[digits[2]];
        _digitsData[4] = _digit2segments[digits[1]];
        _digitsData[5] = _digit2segments[digits[0]] | 0x80;
    }
    else if (lineNumber == 2)
    {
        _digitsData[0] = _digit2segments[digits[2]];
        _digitsData[1] = _digit2segments[digits[1]];
        _digitsData[2] = _digit2segments[digits[0]] | 0x80;
    }
    else
    {
        _digitsData[3] = _digit2segments[digits[2]];
        _digitsData[4] = _digit2segments[digits[1]];
        _digitsData[5] = _digit2segments[digits[0]] | 0x80;
    }
    TM1637_Write6Digits();
}
void TM1637_WriteNumberCurrent(uint8_t lineNumber, uint16_t number)
{
    uint8_t digits[3] = {0}, i = 0;
    if (number > 0 && number < 1000)
    {
        while (number)
        {
            digits[i++] = number % 10;
            number /= 10;
        }
    }
    if (lineNumber == 1)
    {
        _digitsData[3] = _digit2segments[digits[2]];
        _digitsData[4] = _digit2segments[digits[1]] | 0x80; // Add DP here
        _digitsData[5] = _digit2segments[digits[0]];
    }
    else if (lineNumber == 2)
    {
        _digitsData[0] = _digit2segments[digits[2]];
        _digitsData[1] = _digit2segments[digits[1]] | 0x80; // Add DP here
        _digitsData[2] = _digit2segments[digits[0]];
    }
    else
    {
        _digitsData[3] = _digit2segments[digits[2]];
        _digitsData[4] = _digit2segments[digits[1]] | 0x80; // Add DP here
        _digitsData[5] = _digit2segments[digits[0]];
    }
    TM1637_Write6Digits();
}

void TM1637_WriteNumber(uint8_t lineNumber, uint16_t number)
{
    uint8_t digits[3] = {0}, i = 0;
    if (number > 0 && number < 1000)
    {
        while (number)
        {
            // printf("%d\n", score % 10);
            digits[i++] = number % 10;
            number /= 10;
        }
    }
    if (lineNumber == 1)
    {

        _digitsData[3] = _digit2segments[digits[2]];
        _digitsData[4] = _digit2segments[digits[1]];
        _digitsData[5] = _digit2segments[digits[0]];
    }
    else if (lineNumber == 2)
    {
        _digitsData[0] = _digit2segments[digits[2]];
        _digitsData[1] = _digit2segments[digits[1]];
        _digitsData[2] = _digit2segments[digits[0]];
    }
    else
    {
        _digitsData[3] = _digit2segments[digits[2]];
        _digitsData[4] = _digit2segments[digits[1]];
        _digitsData[5] = _digit2segments[digits[0]];
    }
    TM1637_Write6Digits();
}
void TM1637_Write6Digits()
{
    TM1637_start();
    TM1637_write_byte(0x40);
    TM1637_stop();
    TM1637_start();
    TM1637_write_byte(0xc0);
    // First digit/Char starts here
    TM1637_write_byte(_digitsData[0]);
    TM1637_write_byte(_digitsData[1]);
    TM1637_write_byte(_digitsData[2]);
    TM1637_write_byte(_digitsData[3]);
    TM1637_write_byte(_digitsData[4]);
    TM1637_write_byte(_digitsData[5]);
    TM1637_stop();
}
uint8_t TM1637_write_byte(uint8_t value)
{
    uint8_t i;

    for (i = 0; i < 8; ++i)
    {
        TM1637_CLK_LOW();
        _delay_us(TM1637_DELAY_US);

        if (value & 0x01)
        {
            TM1637_DIO_HIGH();
        }
        else
        {
            TM1637_DIO_LOW();
        }
        _delay_us(TM1637_DELAY_US);
        TM1637_CLK_HIGH();
        _delay_us(TM1637_DELAY_US);
        value >>= 1;
    }
    TM1637_CLK_LOW();
    // TM1637_DIO_INPUT();
    // TM1637_DIO_HIGH();
    _delay_us(TM1637_DELAY_US);

    // ack = TM1637_DIO_READ();
    /*ack = P21;
    if (ack)
    {
        TM1637_DIO_OUTPUT();
        TM1637_DIO_LOW();
    }
    _delay_us(TM1637_DELAY_US);
    */
    TM1637_CLK_HIGH();
    TM1637_DIO_HIGH();
    _delay_us(TM1637_DELAY_US);

    // TM1637_CLK_LOW();
    //_delay_us(TM1637_DELAY_US);

    // TM1637_DIO_OUTPUT();

    return P21;
}

#if 0
void TM1637_WriteDualSumpStatus(char sump1, char sump2)
{
    // First digit (Sump 1 status) is at position 0 (2nd display)
    if (sump1 == 'E')
    {
        _digitsData[0] = _char2segments['E' - 65];
    }
    else if (sump1 == 'L')
    {
        _digitsData[0] = _char2segments['L' - 65];
    }
    else if (sump1 == 'F')
    {
        _digitsData[0] = _char2segments['F' - 65];
    }
    else
    {
        _digitsData[0] = 0x00; // Blank
    }

    // Second digit (Sump 2 status) is at position 1 (2nd display)
    if (sump2 == 'E')
    {
        _digitsData[1] = _char2segments['E' - 65];
    }
    else if (sump2 == 'L')
    {
        _digitsData[1] = _char2segments['L' - 65];
    }
    else if (sump2 == 'F')
    {
        _digitsData[1] = _char2segments['F' - 65];
    }
    else
    {
        _digitsData[1] = 0x00; // Blank
    }

    // Clear the 3rd decimal place slot just in case
    _digitsData[2] = 0x00;

    TM1637_Write6Digits();
}
#endif