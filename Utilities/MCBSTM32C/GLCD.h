/******************************************************************************/
/* GLCD.h: Graphic LCD function prototypes and defines                        */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2011 Keil - An ARM Company. All rights reserved.        */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#ifndef _GLCD_H
#define _GLCD_H

/*------------------------------------------------------------------------------
Color coding
GLCD is coded:   15..11 red, 10..5 green, 4..0 blue  (unsigned short)  GLCD_R5, GLCD_G6, GLCD_B5   
original coding: 17..12 red, 11..6 green, 5..0 blue                    ORG_R6,  ORG_G6,  ORG_B6

ORG_R1..5 = GLCD_R0..4,  ORG_R0 = GLCD_R4
ORG_G0..5 = GLCD_G0..5,
ORG_B1..5 = GLCD_B0..4,  ORG_B0 = GLCD_B4
*----------------------------------------------------------------------------*/

/* GLCD RGB color definitions                                                 */
#define BLACK           0x0000      /*   0,   0,   0 */
#define NAVY            0x000F      /*   0,   0, 128 */
#define DARKGREEN       0x03E0      /*   0, 128,   0 */
#define DARKCYAN        0x03EF      /*   0, 128, 128 */
#define MAROON          0x7800      /* 128,   0,   0 */
#define PURPLE          0x780F      /* 128,   0, 128 */
#define OLIVE           0x7BE0      /* 128, 128,   0 */
#define LIGHTGREY       0xC618      /* 192, 192, 192 */
#define DARKGREY        0x7BEF      /* 128, 128, 128 */
#define BLUE            0x001F      /*   0,   0, 255 */
#define GREEN           0x07E0      /*   0, 255,   0 */
#define CYAN            0x07FF      /*   0, 255, 255 */
#define RED             0xF800      /* 255,   0,   0 */
#define MAGENTA         0xF81F      /* 255,   0, 255 */
#define YELLOW          0xFFE0      /* 255, 255, 0   */
#define WHITE           0xFFFF      /* 255, 255, 255 */

#define MYBLUE          0x4BBD
#define FH_ORANGE		0xF480      /* 241, 147, 0	  */	 //1111 0001    1001 0011  0000 0000
//565
//1111 0        100 100     0 0000

/* Instruction List Table For SPFD5408A                                        */
#define     ID_Read                              	0x00
#define     Drive_Output_Control                 	0x01
#define     LCD_AC_Drive_Control                 	0x02
#define     Entry_Mode                           	0x03
#define     Resizing_Control           			 	0x04
#define     Display_Control_1           	   	 	0x07
#define     Display_Control_2           		 	0x08
#define     Display_Control_3                    	0x09
#define     Display_Control_4                    	0x0A
#define     External_Interface_Control_1            0x0C
#define     Frame_Maker_Position           		 	0x0D
#define     External_Interface_Control_2           	0x0F
#define     Power_Control_1           	   	  	   	0x10
#define     Power_Control_2           			   	0x11
#define     Power_Control_3                        	0x12
#define     Power_Control_4           		        0x13
#define     Power_Control_5                        	0x17
#define     GRAMADDRET_Horizontal_ADDR              0x20
#define     GRAMADDRET_Vertical_ADDR           		0x21
#define     GRAM_Data           	   	  	   	    0x22
#define     NVM_Read_Data_1           			   	0x28
#define     NVM_Read_Data_2                        	0x29
#define     NVM_Read_Data_3           	            0x2A
#define     r_Control_1                        		0x30
#define     r_Control_2                        	    0x31
#define     r_Control_3           			   	    0x32
#define     r_Control_4           	   	  	   		0x33
#define     r_Control_5           			   		0x34
#define     r_Control_6                        		0x35
#define     r_Control_7                             0x36
#define     r_Control_8                             0x37
#define     r_Control_9           			   		0x38
#define     r_Control_10           	   	  	   		0x39
#define     r_Control_11          			   		0x3A
#define     r_Control_12                       		0x3B
#define     r_Control_13                            0x3C
#define     r_Control_14                            0x3D
#define     r_Control_15                            0x3E
#define     r_Control_16                            0x3F
#define     Window_Horzontal_RAMADDR_START_1        0x50
#define     Window_Horzontal_RAMADDR_START_2        0x51
#define     Window_Vertical_RAMADDR_START_1         0x52
#define     Window_Vertical_RAMADDR_START_2         0x53
#define     Driver_Output_Control                   0x60
#define     Image_Output_Control                    0x61
#define     Vertical_Scolling_Control               0x6A
#define     Display_Position_1                      0x80
#define     GRAM_Start_Line_ADDR_1           	    0x81
#define     GRAM_End_Line_ADDR_1           	   	  	0x82
#define     Display_Position_2           			0x83
#define     GRAM_Start_Line_ADDR_2                  0x84
#define     GRAM_End_Line_ADDR_2                    0x85
#define     Panel_Interface_Control_1               0x90
#define     Panel_Interface_Control_2               0x92
#define     Panel_Interface_Control_3           	0x93
#define     Panel_Interface_Control_4           	0x95
#define     Panel_Interface_Control_5           	0x97
#define     Panel_Interface_Control_6               0x98
#define     NVM_Conmtrol_1           	   	  	    0xA0
#define     NVM_Conmtrol_2           			    0xA1
#define     Calibration_Control                     0xA4



extern void GLCD_Init                       (void);
extern void GLCD_WindowMax                  (void);
extern void GLCD_PutPixel                   (unsigned int x, unsigned int y);
extern void GLCD_SetTextColor               (unsigned short color);
extern void GLCD_SetBackColor               (unsigned short color);
extern void GLCD_Clear                      (unsigned short color);
extern void GLCD_ClearRegion                (unsigned short color, unsigned int x, unsigned int y, unsigned int cw, unsigned int ch) ;
extern void GLCD_DrawChar                   (unsigned int x,  unsigned int y, unsigned int cw, unsigned int ch, unsigned char *c);
extern void GLCD_DisplayChar                (unsigned int ln, unsigned int col, unsigned char fi, unsigned char  c);
extern void GLCD_DisplayString              (unsigned int ln, unsigned int col, unsigned char fi, unsigned char *s);
extern void GLCD_DrawCharWithColor 			(unsigned int x, unsigned int y, unsigned int cw, unsigned int ch, unsigned char *c, unsigned short BackColor, unsigned short TextColor);
extern void GLCD_DisplayCharWithColor       (unsigned int ln, unsigned int col, unsigned char fi, unsigned char  c, unsigned short BackColor, unsigned short TextColor);
extern void GLCD_DisplayStringWithColor     (unsigned int ln, unsigned int col, unsigned char fi, unsigned char *s, unsigned short BackColor, unsigned short TextColor);
extern void GLCD_ClearLn                    (unsigned int ln, unsigned char fi);
extern void GLCD_Bargraph                   (unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned int val);
extern void GLCD_BargraphColor              (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int val, unsigned short frontC, unsigned short backC);
extern void GLCD_BargraphWithCorlorLineX    (unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned int val/*, unsigned short BackColor, unsigned short TextColor*/); 
extern void GLCD_BargraphWithCorlorLineY    (unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned int val/*, unsigned short BackColor, unsigned short TextColor*/); 
extern void GLCD_Bitmap                     (unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned char *bitmap);
//extern void GLCD_Bitmap                     (unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned short *bitmap);	//char bitmap

extern void GLCD_ScrollVertical             (unsigned int dy);

extern void GLCD_WrCmd                      (unsigned char cmd);
extern void GLCD_WrReg                      (unsigned char reg, unsigned short val); 

#endif /* _GLCD_H */
