#include "MS51_32K.H"
#include "Timer3_Int.h"

volatile uint32_t ticks_100ms = 0;
uint32_t getTicks100ms(void)
{
	return ticks_100ms;
}
/************************************************************************************************************
*    TIMER 3 interrupt subroutine
************************************************************************************************************/
void Timer3_ISR (void) interrupt 16 
{
    _push_(SFRS);
  	clr_T3CON_TF3;
    //P02 = ~P02;
		ticks_100ms++;
		///SendStringUART2_LN("T3 ISR");
    _pop_(SFRS);
}	

/************************************************************************************************************
*    TIMER 3 interrupt Initial setting
************************************************************************************************************/
void Timer3_INT_Initial(unsigned char u8TMDIV, 
                        unsigned char u8RH3, 
                        unsigned char u8RL3)
{
    SFRS = 0x00;
    switch (u8TMDIV)
    {
      case DIV1:T3CON&=0xF8; break;
      case DIV2:T3CON&=0xF8;T3CON|=0x01; break;
      case DIV4:T3CON&=0xF8;T3CON|=0x02; break;
      case DIV8:T3CON&=0xF8;T3CON|=0x03; break;
      case DIV16:T3CON&=0xF8;T3CON|=0x04; break;
      case DIV32:T3CON&=0xF8;T3CON|=0x05; break;
      case DIV64:T3CON&=0xF8;T3CON|=0x06; break;
      case DIV128:T3CON&=0xF8;T3CON|=0x07; break;
    }
		clr_T3CON_TF3;
		RH3 = u8RH3;
		RL3 = u8RL3;
		set_EIE1_ET3;                              //enable Timer3 interrupt
		//set_IE_EA;                                  //enable global interrupts
		
		ENABLE_GLOBAL_INTERRUPT;
		set_T3CON_TR3;                              //Timer3 run
			
}
