/**
*  ***********************************************************************
* @file    LcdDefs.h
* @author  Madhan and Aravind
* @version V2.00
* @date    27/09/2015
* @brief   Basic LCD functions  
*
*  changes:  25.09.2014 Janezic Sommerauer File adaptation for STM32F4 Board
*            16.06.2015 Madhan and Aravind: Introduced parameters for coloured 
*                       Bar display of Battery
*           
**************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#if defined USE_MCBSTM32C
  #include "GLCD.h"
#elif defined USE_DISCOVERY
   #include "stm32f4_discovery_lcd.h"
#elif defined  USE_MCBSTM32
#else
    #error: "Illegal target board selection!"
#endif

#include "BasicIO.h"                        //!< useful constants, avoid multiple include files

/* Defines used by STM32F1 ---------------------------------------------------*/
#define  BACKGROUND     BLACK               //!< Specifies the background color
#define  TEXT           WHITE               //!< Specifies the Variables color
#define  TEXT_HEAD      RED                 //!< Specifies the Constant Text color
#define State_Boot_Screen         7         //!< Display memu
#define State_Display_Screen      9         //!< Display Motor and BMS Parameters


/* Defines used by STM32F4 ---------------------------------------------------*/
/* Initial graphic properties dependent on display size*/
// Large Battery  
#define DRAW_BAT_BASE1_X        LCD_PIXEL_WIDTH/4
#define DRAW_BAT_BASE1_Y        200
#define DRAW_BAT_LENGTH         LCD_PIXEL_WIDTH/2
#define DRAW_BAT_HEIGHT         LCD_PIXEL_HEIGHT/7
#define DRAW_BAT_S_HEIGHT       DRAW_BAT_HEIGHT/3
#define DRAW_BAT_BASE2_X        DRAW_BAT_BASE1_X+DRAW_BAT_LENGTH-1
#define DRAW_BAT_BASE2_Y        DRAW_BAT_BASE1_Y+DRAW_BAT_S_HEIGHT
#define DRAW_BAT_TIP_LENGTH     DRAW_BAT_S_HEIGHT/3
#define DRAW_BAT_TIP_HEIGHT     DRAW_BAT_HEIGHT/3+1
#define DRAW_BAT_LOWLEVEL       0.1 
#define DRAW_BAT_MIDLEVEL       0.4         // newly added for battery percentage colour display

// Speed meter frame
#define DRAW_THROT_BASE1_X        LCD_PIXEL_WIDTH/8-1
#define DRAW_THROT_BASE1_Y        60
#define DRAW_THROT_LENGTH         LCD_PIXEL_WIDTH*3/4
#define DRAW_THROT_HEIGHT         LCD_PIXEL_HEIGHT/15
#define DRAW_THROT_MDLEVEL        0.6
#define DRAW_THROT_HLEVEL         0.8

/* Extern used by STM32F1 -----------------------------------------------------*/
extern  UINT8 LCD_state;                    //!< The contents on the LCD
extern  UINT8 LCD_Update_FLAG;              //!< If LDC need a refresh

/* Extern used by all boards --------------------------------------------------*/
extern UINT32 ThrottleCmdNew;               //!< Smoothened throttle command value
extern UINT8  ThrottleCmdNew_str[5];        //!< Throttle command value in string 

/* Declare---------------------------------------------------------------------*/
void     LCD_Init(void);                          //!<Init LCD
void     LCD_Display_Measured_Values(void);       //!<Display the measured values on the LCD 
void     LCD_Display_Parameters (void);           //!<Display parameters/units on the LCD 

/*Functions only used by STM32F1 board*/
void     Boot_Screen(void);                 //!<Shows menu 
void     wr_LCD_state(UINT8 lcd_state);     //!<Write LCD Display state
UINT8    get_LCD_state(void);

