#include "UART2_functions.h"
#include "MS51_32K.h"

void init_uart_2(void)
{
    P03_QUASI_MODE;                              /* Set UART2_TXD pin P0.3 as Quasi mode */
    //P04_INPUT_MODE;                              /* Set UART2_RXD pin P0.4 as Input mode */
    ENABLE_UART2_TXD_P03;
    //ENABLE_UART2_RXD_P04;
    UART2_Open(24000000,115200);                 /* Open UART2 use timer1 as baudrate generate and baud rate = 115200*/
    //ENABLE_SC0_RECEIVE_DATA_REACH_INTERRUPT;
    //ENABLE_GLOBAL_INTERRUPT;
	
}
void UART2_Open(unsigned long u32SysClock, unsigned long u32Baudrate)
{
    set_SC0CR0_SCEN;         /* Enable SC module */
    set_SC0CR1_UARTEN;       /* set as UART mode */
    set_SC0CR1_CLKKEEP;      
    clr_SC0CR0_CONSEL;       /* clr CONSEL and AUTOCEN*/
    clr_SC0CR0_AUTOCEN;
 
    SC0ETURD0 = LOBYTE((u32SysClock/8/u32Baudrate)-1);  /* define baud rate low byte */
    SC0ETURD1 &= 0xF0;                                /* define baud rate high byte */
    SC0ETURD1 |= (HIBYTE(u32SysClock/8/u32Baudrate-1))&0x0F; 
    set_SC0CR1_PBOFF;     //parity bit disable
    SC0CR1&=0xCF;        //datalegth 8bit
    set_SC0CR0_NSB;      //stop bit = 1bit
}

void UART2_Send_Data(unsigned char c)
{
      clr_SC0CR0_TXOFF;
      SC0DR = c;
      while(!(SC0TSR&SET_BIT3));
      clr_SC0CR0_TXOFF;
}
void SendStringUART2(char* str)
{
	while(*str)
	{
		UART2_Send_Data(*str++);
	}
}
void SendStringUART2_LN(char* str)
{
	while(*str)
	{
		UART2_Send_Data(*str++);
	}
	UART2_Send_Data('\r');
	UART2_Send_Data('\n');
}
//function to reverse a string
void reverse(char str[], int length)
{
    int start;char ch;
    int end = length -1;
    for(start = 0; start < end; ++start, --end)
    {
        ch = str[start];
        str[start] = str[end];
        str[end] = ch;
    }
}
//Implemented own itoa function
char* itoa(int num, char* str, int base)
{
    int i = 0;
    char isNegNum = 0;
    /*Handle 0 explicitly, 
      otherwise empty string is printed for 0 */
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
    }
    else
    {
        // In library itoa function -ve numbers handled only with
        // base 10. SO here we are also following same concept
        if ((num < 0) && (base == 10))
        {
            isNegNum = 1;
            num = -num; // make num positive
        }
        // Process individual digits
        do
        {
            const int rem = (num % base);
            str[i++] = (rem > 9)? ((rem-10) + 'a') : (rem + '0');
            num = num/base;
        }
        while (num != 0);
        // If number is negative, append '-'
        if (isNegNum)
        {
            str[i++] = '-';
        }
        // Append string terminator
        str[i] = '\0';
        // Reverse the string
        reverse(str, i);
    }
    return str;
}
char* ltoa(long num, char* str, int base)
{
    int i = 0;
    char isNegNum = 0;
    /*Handle 0 explicitly, 
      otherwise empty string is printed for 0 */
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
    }
    else
    {
        // In library itoa function -ve numbers handled only with
        // base 10. SO here we are also following same concept
        if ((num < 0) && (base == 10))
        {
            isNegNum = 1;
            num = -num; // make num positive
        }
        // Process individual digits
        do
        {
            const int rem = (num % base);
            str[i++] = (rem > 9)? ((rem-10) + 'a') : (rem + '0');
            num = num/base;
        }
        while (num != 0);
        // If number is negative, append '-'
        if (isNegNum)
        {
            str[i++] = '-';
        }
        // Append string terminator
        str[i] = '\0';
        // Reverse the string
        reverse(str, i);
    }
    return str;
}
void SendIntegerUART2(int n)
{
		 
	char str[10]= {0};
	itoa(n, str, 10);
	SendStringUART2(str);
}
void SendLongUART2(long n)
{
	char str[10]= {0};
	ltoa(n, str, 10);
	SendStringUART2(str);
}