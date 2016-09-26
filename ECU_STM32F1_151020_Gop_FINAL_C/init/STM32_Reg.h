/**
*  **************************************************************
 * @file    STM32_Reg.h
 * @brief   STM32Register values and Bit definitions
 * @version V1.02
 *  changes:  V1.02 added register RCC_APB2ENR values
 *            V1.01 added register RCC_CSR values
 *            V1.00 Initial Version
 
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2005-2008 Keil Software. All rights reserved.
**************************************************************
*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __STM32_REG_H
#define __STM32_REG_H

/*----------------------------------------------------------------------------
   SysTick
 *----------------------------------------------------------------------------*/
/* register SYSTICK_CSR ------------------------------------------------------*/
#define SYSTICK_CSR_ENABLE    ((unsigned long)0x00000001)	   //!< register SYSTICK_CSR
#define SYSTICK_CSR_COUNTFLAG ((unsigned long)0x00010000)	   //!< register SYSTICK_CSR


/*----------------------------------------------------------------------------
   PWR
 *----------------------------------------------------------------------------*/
/* register PWR_CR -----------------------------------------------------------*/
#define PWR_CR_DBP            ((unsigned long)0x00000100)	   //!< register PWR_CR


/*----------------------------------------------------------------------------
   RCC
 *----------------------------------------------------------------------------*/
/* register RCC_CFGR ---------------------------------------------------------*/
#define RCC_CR_HSION          ((unsigned long)0x00000001)	   //!< register RCC_CFGR
#define RCC_CR_HSIRDY         ((unsigned long)0x00000002)	   //!< register RCC_CFGR
#define RCC_CR_HSEON          ((unsigned long)0x00010000)	   //!< register RCC_CFGR
#define RCC_CR_HSERDY         ((unsigned long)0x00020000)	   //!< register RCC_CFGR
#define RCC_CR_PLLON          ((unsigned long)0x01000000)	   //!< register RCC_CFGR
#define RCC_CR_PLLRDY         ((unsigned long)0x02000000)	   //!< register RCC_CFGR

/* register RCC_CFGR ---------------------------------------------------------*/
#define RCC_CFGR_SW           ((unsigned long)0x00000003)		//!< register RCC_CFGR
#define RCC_CFGR_SWS          ((unsigned long)0x0000000C)		//!< register RCC_CFGR
#define RCC_CFGR_HPRE         ((unsigned long)0x000000F0)		//!< register RCC_CFGR
#define RCC_CFGR_PRE1         ((unsigned long)0x00000700)		//!< register RCC_CFGR
#define RCC_CFGR_PRE2         ((unsigned long)0x00003800)		//!< register RCC_CFGR
#define RCC_CFGR_PLLSRC       ((unsigned long)0x00010000)		//!< register RCC_CFGR
#define RCC_CFGR_PLLXTPRE     ((unsigned long)0x00020000)		//!< register RCC_CFGR
#define RCC_CFGR_PLLMULL      ((unsigned long)0x003C0000)		//!< register RCC_CFGR
#define RCC_CFGR_USBPRE       ((unsigned long)0x00400000)		//!< register RCC_CFGR
#define RCC_CFGR_MCO          ((unsigned long)0x07000000)		//!< register RCC_CFGR

/* register RCC_APB1ENR ------------------------------------------------------*/
#define RCC_APB1ENR_TIM2EN    ((unsigned long)0x00000001)	   //!< register RCC_APB1ENR
#define RCC_APB1ENR_TIM3EN    ((unsigned long)0x00000002)	   //!< register RCC_APB1ENR
#define RCC_APB1ENR_TIM4EN    ((unsigned long)0x00000004)	   //!< register RCC_APB1ENR
#define RCC_APB1ENR_USART2EN  ((unsigned long)0x00020000)	   //!< register RCC_APB1ENR
#define RCC_APB1ENR_USART3EN  ((unsigned long)0x00040000)	   //!< register RCC_APB1ENR
#define RCC_APB1ENR_CANEN     ((unsigned long)0x02000000)	   //!< register RCC_APB1ENR
#define RCC_APB1ENR_BKPEN     ((unsigned long)0x08000000)	   //!< register RCC_APB1ENR
#define RCC_APB1ENR_PWREN     ((unsigned long)0x10000000)	   //!< register RCC_APB1ENR

/* register RCC_APB2ENR ------------------------------------------------------*/
#define RCC_APB2ENR_AFIOEN    ((unsigned long)0x00000001)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_IOPAEN    ((unsigned long)0x00000004)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_IOPBEN    ((unsigned long)0x00000008)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_IOPCEN    ((unsigned long)0x00000010)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_IOPDEN    ((unsigned long)0x00000020)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_IOPEEN    ((unsigned long)0x00000040)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_IOPFEN    ((unsigned long)0x00000080)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_IOPGEN    ((unsigned long)0x00000100)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_ADC1EN    ((unsigned long)0x00000200)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_ADC2EN    ((unsigned long)0x00000400)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_TIM1EN    ((unsigned long)0x00000800)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_SPI1EN    ((unsigned long)0x00001000)		//!< register RCC_APB2ENR
#define RCC_APB2ENR_USART1EN  ((unsigned long)0x00004000)		//!< register RCC_APB2ENR

/* register RCC_BCDR --------------------------------------------------------*/
#define RCC_BDCR_LSEON        ((unsigned long)0x00000001)	   //!< register RCC_BCDR
#define RCC_BDCR_LSERDY       ((unsigned long)0x00000002)	   //!< register RCC_BCDR
#define RCC_BDCR_RTCSEL       ((unsigned long)0x00000300)	   //!< register RCC_BCDR
#define RCC_BDCR_RTCEN        ((unsigned long)0x00008000)	   //!< register RCC_BCDR

/* register RCC_CSR ---------------------------------------------------------*/
#define RCC_CSR_LSION         ((unsigned long)0x00000001)	   //!< register RCC_CSR
#define RCC_CSR_LSIRDY        ((unsigned long)0x00000002)	   //!< register RCC_CSR

/*----------------------------------------------------------------------------
   RTC
 *----------------------------------------------------------------------------*/
/* register RTC_CR -----------------------------------------------------------*/
#define RTC_CRL_SECF          ((unsigned long)0x00000001)	   //!< register RTC_CR
#define RTC_CRL_ALRF          ((unsigned long)0x00000002)	   //!< register RTC_CR
#define RTC_CRL_OWF           ((unsigned long)0x00000004)	   //!< register RTC_CR
#define RTC_CRL_RSF           ((unsigned long)0x00000008)	   //!< register RTC_CR
#define RTC_CRL_CNF           ((unsigned long)0x00000010)	   //!< register RTC_CR
#define RTC_CRL_RTOFF         ((unsigned long)0x00000020)	   //!< register RTC_CR


/*----------------------------------------------------------------------------
   TIMX
 *----------------------------------------------------------------------------*/
/* register TIMX_CR1 ---------------------------------------------------------*/
#define TIMX_CR1_CEN         ((unsigned short)0x0001)		   //!< register TIMX_CR1

/* register TIMX_SR ----------------------------------------------------------*/
#define TIMX_SR_UIF          ((unsigned short)0x0001)		  //!< register TIMX_SR


/*----------------------------------------------------------------------------
   CAN
 *----------------------------------------------------------------------------*/
/* register CAN_MCR ----------------------------------------------------------*/
#define CAN_MCR_INRQ          ((unsigned long)0x00000001)	 //!< register CAN_MCR
#define CAN_MCR_NART          ((unsigned long)0x00000010)	 //!< register CAN_MCR

/* register CAN_FMR ----------------------------------------------------------*/
#define CAN_FMR_FINIT         ((unsigned long)0x00000001)	 //!< register CAN_FMR

/* register CAN_TSR ----------------------------------------------------------*/
#define CAN_TSR_RQCP0         ((unsigned long)0x00000001)	//!< register CAN_TSR
#define CAN_TSR_TME0          ((unsigned long)0x04000000)	//!< register CAN_TSR

/* register CAN_RF0R ---------------------------------------------------------*/
#define CAN_RF0R_FMP0         ((unsigned long)0x00000003)	 //!< register CAN_RF0R
#define CAN_RF0R_RFOM0        ((unsigned long)0x00000020)	 //!< register CAN_RF0R

/* register CAN_IER ----------------------------------------------------------*/
#define CAN_IER_TMEIE         ((unsigned long)0x00000001)	//!< register CAN_IER 
#define CAN_IER_FMPIE0        ((unsigned long)0x00000002)	//!< register CAN_IER 

/* register CAN_BTR ----------------------------------------------------------*/
#define CAN_BTR_SILM          ((unsigned long)0x80000000)	//!< register CAN_BTR_SILM
#define CAN_BTR_LBKM          ((unsigned long)0x40000000)   //!< register CAN_BTR_LBKM

/* register CAN_TIxR ----------------------------------------------------------*/
#define CAN_TIxR_TXRQ         ((unsigned long)0x00000001)	//!< register CAN_TIxR

/* register CAN_TDTxR ----------------------------------------------------------*/
#define CAN_TDTxR_DLC         ((unsigned long)0x0000000F)	//!< register CAN_TDTxR 

/*----------------------------------------------------------------------------
   ADC
 *----------------------------------------------------------------------------*/
/* register ADC_SR----------------------------------------------------------*/
#define ADC_SR_EOC            ((unsigned long)0x00000002)	//!< register ADC_SR

/* register ADC_DR -----------------------------------------------------------*/
#define ADC_DR_DATA           ((unsigned long)0x0000FFFF)	//!< register ADC_DR

/*----------------------------------------------------------------------------
   USART
 *----------------------------------------------------------------------------*/
/* register USART_CR1 --------------------------------------------------------*/
#define USART_CR1_RE          ((unsigned long)0x00000004)	  //!< register USART_CR1_RE
#define USART_CR1_TE          ((unsigned long)0x00000008)	  //!< register USART_CR1_TE
#define USART_CR1_UE          ((unsigned long)0x00002000)	  //!< register USART_CR1_UE

#endif
