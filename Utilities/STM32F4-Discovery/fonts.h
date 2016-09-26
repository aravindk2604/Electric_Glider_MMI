/**
******************************************************************************
* @file    fonts.h
* @author  MCD Application Team
* @version V4.6.1
* @date    18-April-2011
* @brief   Header for fonts.c
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
******************************************************************************  
*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FONTS_H
#define __FONTS_H



/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/*** FONTS_Exported_Types ***/ 
typedef struct _tFont           //!< All font related parameters
{    
  const uint32_t *table;  // Pointer to the table containing the characters in a pixel manner
  uint16_t Width;         // Width of the font
  uint16_t Height;        // Height of the font
} sFONT;

//Collection fo fonts stored in fonts.c
extern sFONT Arial_18x27; 
extern sFONT Arial_14x22;
extern sFONT Arial_13x19; 
extern sFONT Arial_11x18; 
extern sFONT Arial_10x15;
extern sFONT Arial_8x13 ;
extern sFONT Arial_7x10 ;


/*** FONTS_Exported_Macros ***/ 
#define LINE(x) ((x) * (((sFONT *)LCD_GetFont())->Height))      //!< Macro to read the height of the font and select the right line 
#define COLUMN(x) ((x) * (((sFONT *)LCD_GetFont())->Width))     //!< Macro to read the width of the font and select the right column 


#endif /* __FONTS_H */



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
