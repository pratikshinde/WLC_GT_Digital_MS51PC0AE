#ifndef __UART2_FUNCTIONS__
#define __UART2_FUNCTIONS__

void init_uart_2(void);
void SendStringUART2(char* str);
void SendStringUART2_LN(char* str);
void SendIntegerUART2(int n);
void SendLongUART2(long n);

#endif