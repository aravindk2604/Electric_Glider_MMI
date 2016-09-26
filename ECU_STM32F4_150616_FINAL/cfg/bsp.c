/**
*  ***********************************************************************
*  @file    BSP.c
*  @author  B. Mysliwetz
*  @version V2.20
*  @date    12/11/2013
*
*  @brief   Elementary STM32F10x board support package for projects
*           NOT using uC/OS; contains low level serial character-IO 
*           and LED-IO functions.
*
*           'Retarget' layer with TARGET-SPECIFIC functions for the Keil 
*           MCBSTM32x evalboards  
*
*  Changes:
*  16.11.2008 keil  V1.0
*  18.11.2008 wu    V1.1 Adaption of SVC Handler
*  03.03.2011 mys   V1.2 Serial IO functions for fputc(), fgetc() outsourced 
*  25.02.2013 mys   V2.0 Doxygen compatible & new project folder structure 
*  03.11.2013 mys   V2.1 For portable STM32 embedded projects NOT using uC/OS 
*  12.11.2013 Im    V2.2 For additionaly Portable to Discovery STM32F407VG NOT using uC/OS
*
**************************************************************************
*/


#ifdef USE_DISCOVERY
  #include "stm32f4_discovery.h"     // Functions specific to discovery board
  #include "stm32f4xx_conf.h"         // Periph. library configuration file
#elif defined USE_MCBSTM32C
  #include  "stm32f10x_conf.h"	                  //!<STM32F10x Configuration File
  #include  "STM32_Init.h"       					  			//!< STM32 initialization package
#else
#endif

#include  <rt_misc.h>
#include  <stdio.h>                  // needed here for defs of FILE, EOF
#include  "platform_cfg.h"           // hardware selection & abstraction defs


unsigned long  BSP_CPU_ClkFreq (void)
/**
*  ***********************************************************************
*  @brief  Read CPU registers to determine clock frequency of the chip.
*  @retval CPU clock frequency, in [Hz].
**************************************************************************
*/
{
    RCC_ClocksTypeDef  rcc_clocks;

    RCC_GetClocksFreq(&rcc_clocks);
    return ((unsigned long)rcc_clocks.HCLK_Frequency);
}

//#ifdef UCOS
unsigned long  OS_CPU_SysTickClkFreq (void)
/**
*  ***********************************************************************
*  @brief  Get system tick clock frequency.
*  @retval Clock frequency (of system tick).
**************************************************************************
*/
{
    unsigned long  freq;

    freq = BSP_CPU_ClkFreq();
    return (freq);
}
//#endif





void    BSP_SetLED ( unsigned char LED_bitpattern )
/**
*  ***********************************************************************
*  @brief  Write to IO port that controls LEDs (row of max 8 LEDs assumed).
*  @param  LED_bitpattern : 1 turns on, 0 turns off individual LED
*  @retval none
**************************************************************************
*/
{
  GPIO_LED->ODR = LED_bitpattern << 8;    // 1 turns on LED on MCBSTM32B
}




void    BSP_SetEXTIPR ( unsigned int EXTI_bitpattern )
/**
*  ***********************************************************************
*  @brief  Write to IO port that accesses the EXTI->PendingRegister.
*  @param  EXTI_bitpattern : a 1 bit turns off pending interrupt
*  @retval none
**************************************************************************
*/
{
  EXTI->PR = EXTI_bitpattern;    // 1 resets a pending EXTI bit
}


unsigned int  BSP_GetEXTIPR ( )
/**
*  ***********************************************************************
*  @brief  Reads IO port that accesses the EXTI->PendingRegister.
*  @retval none
**************************************************************************
*/
{
  return EXTI->PR;    // a 1 means EXTI is pending
}



char  _putch (int ch)
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial port#1.
*  @param  ch : character to output
*  @retval output character
**************************************************************************
*/
{
  while ( !(USART_x->SR & USART_FLAG_TXE) );
  USART_x->DR = (ch & 0xFF);
  return (ch);
}


char  _getch (void)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial port#1 (BLOCKiNG MODE).
*  @retval input character
**************************************************************************
*/
{
  while ( !(USART_x->SR & USART_FLAG_RXNE) );
  return (USART_x->DR & 0xFF);
}


char  _keyhit (void)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial port#1 (NON-BLOCKiNG MODE).
*  @retval input character
**************************************************************************
*/
{
  if ( USART_x->SR & USART_FLAG_RXNE )
    return (USART_x->DR & 0xFF);
  else
    return (0);
}


int  fputc (int ch, FILE *f)
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial port#1.
*  @param  ch : character to output
*  @param  f  : filepointer (as yet ignored here)
*  @retval output character
**************************************************************************
*/
{
  return ( _putch(ch) );
}


int  fgetc (FILE *f)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial port#1 (BLOCKiNG MODE).
*  @param  f  : filepointer (as yet ignored here)
*  @retval input character
**************************************************************************
*/
{
  return ( _putch( _getch() ) );
}


void  _ttywrch (int ch)
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial port#1.
*  @param  ch : character to output
*  @retval none
**************************************************************************
*/
{
  _putch( ch );
}


int  ferror (FILE *f)
/**
*  ***********************************************************************
*  @brief  Return error status of file f, not yet implemented.
*  @param  f  : filepointer (as yet ignored here)
*  @retval error code (as yet const EOF)
**************************************************************************
*/
{
  // your implementation of ferror
  return EOF;
}


void _sys_exit (int return_code)
/**
*  ***********************************************************************
*  @brief  Handle system exit (regular uC/OS tasks never return).
*  @param  return_code : pass info from exiting task.
*  @retval none.
**************************************************************************
*/

{
  label:  goto label;              // endless loop
}





//__asm void SVC_Handler (void)
///**
//*  ***********************************************************************
//*  @brief  SVC_Handler.
//*  @retval none.
//**************************************************************************
//*/
//{
//  PRESERVE8

//  TST     LR,#4                         ; Called from Handler Mode ?
//  MRSNE   R12,PSP                       ; Yes, use PSP
//  MOVEQ   R12,SP                        ; No, use MSP
//  LDR     R12,[R12,#24]                 ; Read Saved PC from Stack
//  LDRH    R12,[R12,#-2]                 ; Load Halfword
//  BICS    R12,R12,#0xFF00               ; Extract SVC Number

//  PUSH    {R4,LR}                       ; Save Registers
//  LDR     LR,=SVC_Count
//  LDR     LR,[LR]
//  CMP     R12,LR
//  BHS     SVC_Dead                      ; Overflow
//  LDR     LR,=SVC_Table
//  LDR     R12,[LR,R12,LSL #2]           ; Load SVC Function Address
//  BLX     R12                           ; Call SVC Function

//  POP     {R4,LR}
//  TST     LR,#4
//  MRSNE   R12,PSP
//  MOVEQ   R12,SP
//  STM     R12,{R0-R3}                   ; Function return values
//  BX      LR                            ; RETI

//SVC_Dead
//  B       SVC_Dead                      ; None Existing SVC

//SVC_Cnt    EQU    (SVC_End-SVC_Table)/4
//SVC_Count  DCD    SVC_Cnt

//; Import user SVC functions here.
//;*IMPORT  __SVC_0
//;*IMPORT  __SVC_1
//;*IMPORT  __SVC_2
//;*IMPORT  __SVC_3

//SVC_Table
//; Insert user SVC functions here
//;*DCD     __SVC_0                   ; SVC 0 Function Entry
//;*DCD     __SVC_1                   ; SVC 1 Function Entry
//;*DCD     __SVC_2                   ; SVC 2 Function Entry
//;*DCD     __SVC_3                   ; SVC 2 Function Entry

//  DCD    0                          ; dummy vectors
//  DCD    0
//  DCD    0
//  DCD    0

//SVC_End

//  ALIGN
//}



/************************************************************************\
*    END OF MODULE BSP.c for the Keil MCBSTM32 evalboard NOT using uC/OS
\************************************************************************/





