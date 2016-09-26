/**
*  ***********************************************************************
 * @file    STM32_Init.h
 * @brief   STM32 peripherals initialisation definitions
 * @version V1.00
 * Note(s):



 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2005-2008 Keil Software. All rights reserved.
**************************************************************************
*/
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __STM32_INIT_H
#define __STM32_INIT_H

extern void         stm32_Init     (void);		   //!< STM32 initialization
extern unsigned int stm32_GetPCLK1 (void);		   //!< STM32 get PCLK1 deliver the PCLK1
extern unsigned int stm32_GetPCLK2 (void);		   //!< STM32 get PCLK2 deliver the PCLK2
#endif
