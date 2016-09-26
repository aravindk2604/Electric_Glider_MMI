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
*  16.11.2008 keil    V1.0
*  18.11.2008 wu      V1.1 Adaption of SVC Handler
*  03.03.2011 mys     V1.2 Serial IO functions for fputc(), fgetc() outsourced 
*  25.02.2013 mys     V2.0 Doxygen compatible & new project folder structure 
*  03.11.2013 mys     V2.1 For portable STM32 embedded projects NOT using uC/OS 
*  12.11.2013 Im      V2.2 For additionaly Portable to Discovery STM32F407VG NOT using uC/OS
*  02.06.2015 jan/som V2.3 Adapted file to be platform independent, Serial IO functions are placed in basicIO
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
  #error: "Illegal target board selection!"
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





/************************************************************************\
*    END OF MODULE BSP.c for the Keil MCBSTM32 evalboard NOT using uC/OS
\************************************************************************/





