/**
******************************************************************************
* @file    stm324xg_discovery_lcd.h
* @author  MCD Application Team
* @version V1.0.0
* @date    30-September-2011
* @brief   This file contains all the functions prototypes for the 
*          stm324xg_discovery_lcd.c driver.
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
* <h2><center>&copy; Portions COPYRIGHT 2011 STMicroelectronics</center></h2>
******************************************************************************  
*/ 
/**
******************************************************************************
* <h2><center>&copy; Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.</center></h2>
* @file    stm32f4_discovery_lcd.h
* @author  CMP Team
* @version V1.0.0
* @date    28-December-2012
* @brief   This file contains all the functions prototypes for the 
*          stm324xg_discovery_lcd.c driver.
*          Modified to support the STM32F4DISCOVERY, STM32F4DIS-BB, STM32F4DIS-CAM
*          and STM32F4DIS-LCD modules.      
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, Embest SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
* OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
* OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
* CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
******************************************************************************
*
*
*      Changes:  25.11.2014 Janezic Sommerauer File adaptation MyTouchOne LCD Module
*
*
*/



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4_DISCOVERY_LCD_H
#define __STM32F4_DISCOVERY_LCD_H



/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "fonts.h"
#include "mctdefs.h"
#include "LcdDefs.h"

/* Typedefs ------------------------------------------------------------------*/
/** 
* Structures
*/
typedef struct 
{
	int16_t X;
	int16_t Y;
} Point, * pPoint; 


/* Defines ------------------------------------------------------------------*/
/** 
* Various definitions for the hardware initialization  
*/
#define  LCD_SSD1289_ID1  0x1289    //!<  ID vom SSD1289 (NEU)
#define  LCD_SSD1289_ID2  0x8989    //!<  ID vom SSD1289 (ALT)
#define  LCD_SSD1289_PAUSE   250    //!<  Delay duration 1
#define  LCD_SSD1289_PAUSE2  250    //!<   Delay duration 2
#define  LCD_SSD1289_FSMC_AST  15   //!<  AdressSetupTime  (AST >= 9)
#define  LCD_SSD1289_FSMC_DST  15   //!<  DataSetupTime    (DST >= 7)
#define  LCD_SSD1289_FSMC_AHT  12   //!<  DataHoldTime      (AHT >= 7)

/** 
*  Various definitions controlling coordinate space mapping and drawing
*  direction in the four supported orientations.   
*/
#define LANDSCAPE                        //!< Landscape Mode
//#define PORTRAIT                        //!< Portrait Mode


/** 
*  The dimensions of the LCD panel.
*/
#if defined(LANDSCAPE)
#define LCD_PIXEL_WIDTH          320    //!< Resolution in X direction 
#define LCD_PIXEL_HEIGHT         240    //!< Resolution in Y direction
#else
#define LCD_PIXEL_WIDTH          240    //!< Resolution in X direction 
#define LCD_PIXEL_HEIGHT         320    //!< Resolution in Y direction
#endif


/** 
* Selected fonts 
*/ 
#define LCD_BIG_DATA_FONT    Arial_18x27
#define LCD_SM_DATA_FONT     Arial_13x19
#define LCD_BIG_UNIT_FONT    Arial_13x19
#define LCD_SM_UNIT_FONT     Arial_10x15
#define LCD_SM_SWV_FONT      Arial_7x10   //!< Small font for displaying SW Version


/** 
*  Alignment definitions when writing to LCD  
*/ 
#define LEFT  0  
#define RIGHT 1  

/** 
* LCD Direction declaration for drawing 
*/ 
#define LCD_DIR_HORIZONTAL       0x0000
#define LCD_DIR_VERTICAL         0x0001



/** 
* Various internal SD2189 registers name labels
*/
#define SSD1289_DEVICE_CODE_READ_REG       0x00        // Display-ID
#define SSD1289_OSC_START_REG              0x00        // Oscillation Start
#define SSD1289_OUTPUT_CTRL_REG            0x01        // Driver output control
#define SSD1289_LCD_DRIVE_AC_CTRL_REG      0x02        // LCD drive AC control
#define SSD1289_PWR_CTRL_1_REG             0x03        // Power control (1)
#define SSD1289_COMPARE_1_REG              0x05        // Compare register (1)
#define SSD1289_COMPARE_2_REG              0x06        // Compare register (1)
#define SSD1289_DISPLAY_CTRL_REG           0x07        // Display control
#define SSD1289_FRAME_CYCLE_CTRL_REG       0x0B        // Frame cycle control
#define SSD1289_PWR_CTRL_2_REG             0x0C        // Power control (2)
#define SSD1289_PWR_CTRL_3_REG             0x0D        // Power control (3)
#define SSD1289_PWR_CTRL_4_REG             0x0E        // Power control (4)
#define SSD1289_GATE_SCAN_START_REG        0x0F        // Gate scan start position
#define SSD1289_SLEEP_MODE_1_REG           0x10        // Sleep mode
#define SSD1289_ENTRY_MODE_REG             0x11        // Display entry mode
#define SSD1289_SLEEP_MODE_2_REG           0x12        // Optimize Access Speed 3
#define SSD1289_GEN_IF_CTRL_REG            0x15        // Generic Interface Control
#define SSD1289_H_PORCH_REG                0x16        // Horizontal Porch
#define SSD1289_V_PORCH_REG                0x17        // Vertical Porch
#define SSD1289_PWR_CTRL_5_REG             0x1E        // Power control (5)
#define SSD1289_RAM_DATA_REG               0x22        // RAM data write[R/W,D/C=01] /RAM data read [R/W,D/C=11]
#define SSD1289_RAM_WR_DATA_MSK_1_REG      0x23        // RAM write data mask (1)
#define SSD1289_RAM_WR_DATA_MSK_2_REG      0x24        // RAM write data mask (2)
#define SSD1289_FRAME_FREQ_REG             0x25        // Frame Frequency
#define SSD1289_VCOM_OTP_1_REG             0x28        // VCOM OTP (optimize access speed 1)
#define SSD1289_VCOM_OTP_2_REG             0x29        // VCOM OTP
#define SSD1289_OPT_ACCESS_SPEED_2_REG     0x2F        // Optimize Access Speed 2
#define SSD1289_GAMMA_CTRL_1_REG           0x30        // Gamma control (1)
#define SSD1289_GAMMA_CTRL_2_REG           0x31        // Gamma control (2)
#define SSD1289_GAMMA_CTRL_3_REG           0x32        // Gamma control (3)
#define SSD1289_GAMMA_CTRL_4_REG           0x33        // Gamma control (4)
#define SSD1289_GAMMA_CTRL_5_REG           0x34        // Gamma control (5)
#define SSD1289_GAMMA_CTRL_6_REG           0x35        // Gamma control (6)
#define SSD1289_GAMMA_CTRL_7_REG           0x36        // Gamma control (7)
#define SSD1289_GAMMA_CTRL_8_REG           0x37        // Gamma control (8)
#define SSD1289_GAMMA_CTRL_9_REG           0x3A        // Gamma control (9)
#define SSD1289_GAMMA_CTRL_10_REG          0x3B        // Gamma control (10)
#define SSD1289_V_SCROLL_1_CTRL_REG        0x41        // Vertical scroll control (1)
#define SSD1289_V_SCROLL_2_CTRL_REG        0x42        // Vertical scroll control (2)
#define SSD1289_V_RAM_POS_REG              0x44        // Horizontal RAM address position
#define SSD1289_H_RAM_START_REG            0x45        // Vertical RAM address start position
#define SSD1289_H_RAM_END_REG              0x46        // Vertical RAM address end position
#define SSD1289_FIRST_WIN_START_REG       0x48        // First window start
#define SSD1289_FIRST_WIN_END_REG         0x49        // First window end
#define SSD1289_SCND_WIN_START_REG         0x4A        // Second window start
#define SSD1289_SCND_WIN_END_REG           0x4B        // Second window end
#define SSD1289_X_RAM_ADDR_REG             0x4E        // Cursor-Pos (X)
#define SSD1289_Y_RAM_ADDR_REG             0x4F        // Cursor-Pos (y)





/** 
* @brief  LCD color definitions 
*/ 
#define LCD_COLOR_WHITE          0xFFFE        
#define LCD_COLOR_BLACK          0x0000
#define LCD_COLOR_GREY           0xF7DE
#define LCD_COLOR_BLUE           0x001F
#define LCD_COLOR_BLUE2          0x051F
#define LCD_COLOR_RED            0xF800
#define LCD_COLOR_MAGENTA        0xF81F
#define LCD_COLOR_GREEN          0x07E0
#define LCD_COLOR_CYAN           0x7FFF
#define LCD_COLOR_YELLOW         0xFFE0

#define WHITE             LCD_COLOR_WHITE
#define BLACK             LCD_COLOR_BLACK
#define RED               LCD_COLOR_RED
#define BLUE              LCD_COLOR_BLUE
#define GREEN             LCD_COLOR_GREEN
#define CYAN              LCD_COLOR_CYAN
#define YELLOW            LCD_COLOR_YELLOW  

/** 
* STM32F4_DISCOVERY_LCD_Exported_Macros
*/
#define ASSEMBLE_RGB(R ,G, B)    ((((R)& 0xF8) << 8) | (((G) & 0xFC) << 3) | (((B) & 0xF8) >> 3)) 

#define _delay_     delay      /* !< Default _delay_ function with less precise timing */


/* STM32F4_DISCOVERY_LCD_Exported_Functions ---------------------------------*/

/* Functions used for initialization */
void STM32f4_Discovery_LCD_Init(void);
void LCD_CtrlLinesConfig(void);
void LCD_FSMCConfig(void);
void LCD_SSD1289_InitChip(void);

/* Functions used for basic window adjusments */
void LCD_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width);
void LCD_WindowModeDisable(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);

/* Functions used to prepare the display */
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void LCD_SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor); 
void LCD_GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor);
void LCD_SetTextColor(__IO uint16_t Color);
void LCD_SetBackColor(__IO uint16_t Color);
void LCD_SetFont(sFONT *fonts);
sFONT *LCD_GetFont(void);

/* Functions used for Initialization */
void LCD_ClearLine(uint16_t Line);
void LCD_Clear(uint16_t Color);

/* Functions used to display data/graphics */
void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint32_t *c);
void LCD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii);
void LCD_DisplayString(uint16_t Xpos, uint16_t Ypos, char *ptr);
void LCD_DisplayString_Pos_Align(uint16_t Xpos, uint16_t Ypos, uint8_t align, char *ptr);
void LCD_DisplayString_Col_Line_Align(uint16_t col_x, uint16_t line_y, uint8_t align, char *ptr);
char* LCD_Form_Str(char *ptr, uint16_t length);

void LCD_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction);
void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_DrawMonoPict(const uint32_t *Pict);
void LCD_WriteBMP(uint32_t BmpAddress);
void LCD_DrawUniLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void LCD_DrawFullCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_PolyLine(pPoint Points, uint16_t PointCount);
void LCD_PolyLineRelative(pPoint Points, uint16_t PointCount);
void LCD_ClosedPolyLine(pPoint Points, uint16_t PointCount);
void LCD_ClosedPolyLineRelative(pPoint Points, uint16_t PointCount);
void LCD_FillPolyLine(pPoint Points, uint16_t PointCount);
void LCD_RGB_Test(void);


#endif /* __STM32F4_DISCOVERY_LCD_H */

