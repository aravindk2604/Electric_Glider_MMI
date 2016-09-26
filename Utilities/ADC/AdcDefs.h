/**
******************************************************************************
* @file    AdcDefs.h
* @author  Rina and Pramodh
* @version V0.01
* @date    28/08/2014
* @brief   Basic ADC functions
* Modified:
* 01.10.2014  Janezic Sommerauer; added adaptation for STM32F4 and STM321xx Families  
*/



/************************************************************************\
*    Headerfiles 
\************************************************************************/
#include "mctDefs.h"
  
#ifdef USE_MCBSTM32C                  //!< STM32F107 board specific files 
  #include    <stm32f10x_adc.h>
#elif defined USE_DISCOVERY           //!< STM32F4 board specific files 
  #include    <stm32f4xx_adc.h>
#else
  #error: "Illegal target board selection!"
#endif
/************************************************************************\
*        NUMERICAL CONSTANTS
\************************************************************************/

#define     AD_ON      0x00000001               //!< Bitmask for Start Conversion
#define     ADC_EOC    0x00000002               //!< Bitmask for End of Conversion

/************************************************************************\
*        FUNCTIONS
\************************************************************************/ 
UINT16   Get_Potvalue(void);                    //!< Getting the pot value from the ADC
void     adc_Printvalues(UINT16 Potvalue);      //!< Print the Poti Values in scale(0 to 100), (-1024 to 1024) and in hex.
