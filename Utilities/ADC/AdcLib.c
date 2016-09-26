
/**
******************************************************************************
* @file    AdcLib.c
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
#include "bsp.h"
#include "BasicIO.h" 
#include "platform_cfg.h" 
#include "AdcDefs.h"

#if defined USE_DISCOVERY
  #include "stm32f4xx_adc.h"
#elif defined USE_MCBSTM32C
  #include "stm32f10x.h"
#elif defined USE_MCBSTM32

#else
  #error: "Illegal target board selection!"
#endif

#define     PRINT_ADCValues                     //!< conditional compile flag


UINT16 Get_Potvalue(void)  
	/**  
	*  ***********************************************************************
	*  @brief  Getting the pot value from the ADC        
	*  @retval Potvalue corresponding to Poti position in the range 0 to 4095
	***************************************************************************
	*/
{
  UINT16  Potvalue = 0xFFF;                           // Dummy ADC Value
  while (1)                                           // do forever
  {    
    #if defined USE_MCBSTM32
      ADC_SoftwareStartConvCmd(ADC_x, ENABLE);        // start conversion
      while(!ADC_GetFlagStatus(ADC_x, ADC_FLAG_EOC)); // wait for end of conversion
      Potvalue = ADC_GetConversionValue(ADC_x);       // Potvalue is obtained from ADC1 Channel#0
    #elif defined USE_MCBSTM32C
      ADC_SoftwareStartConvCmd(ADC_x, ENABLE);        // start conversion
      while(!ADC_GetFlagStatus(ADC_x, ADC_FLAG_EOC)); // wait for end of conversion
      Potvalue = ADC_GetConversionValue(ADC_x);       // Potvalue is obtained from ADC1 Channel#0
    #elif defined USE_DISCOVERY
      ADC_SoftwareStartConv(ADC_x);                   // start conversion
      while(!ADC_GetFlagStatus(ADC_x, ADC_FLAG_EOC)); // wait for end of conversion
      Potvalue = ADC_GetConversionValue(ADC_x);       // Potvalue is obtained from ADC1 Channel#0
    #else
      #error: "Illegal target board selection!"
    #endif
  return Potvalue;                                    // Potvalue is returned    
  }
}  

