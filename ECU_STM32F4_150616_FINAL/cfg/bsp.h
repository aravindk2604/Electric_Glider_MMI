/*
*********************************************************************************************************
*                                     MICIRUM BOARD SUPPORT PACKAGE
*
*                             (c) Copyright 2007; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                        BOARD SUPPORT PACKAGE
*
*                                     ST Microelectronics STM32
*                                              with the
*                                   STM3210B-EVAL Evaluation Board
*
* Filename      : bsp.h
* Version       : V1.10
* Programmer(s) : BAN
* modifications : deleted #include app_cfg.h & #include bsp.h (?!)
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP present pre-processor macro definition.
*********************************************************************************************************
*/

#ifndef  BSP_PRESENT
#define  BSP_PRESENT

/*
*********************************************************************************************************
*                                                 EXTERNS
*********************************************************************************************************
*/

#ifdef   BSP_MODULE
#define  BSP_EXT
#else
#define  BSP_EXT  extern
#endif

/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/



#include  <cpu.h>
#include  <ucos_ii.h>

/*
*********************************************************************************************************
*                                                 DEFINES
*********************************************************************************************************
*/

#define  BSP_PB_ID_KEY                                     1
#define  BSP_PB_ID_WAKEUP                                  2
#define  BSP_PB_ID_TAMPER                                  3

/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/

#define  BSP_INT_ID_WWDG                                   0    /* Window WatchDog Interrupt                            */
#define  BSP_INT_ID_PVD                                    1    /* PVD through EXTI Line detection Interrupt            */
#define  BSP_INT_ID_TAMPER                                 2    /* Tamper Interrupt                                     */
#define  BSP_INT_ID_RTC                                    3    /* RTC global Interrupt                                 */
#define  BSP_INT_ID_FLASH                                  4    /* FLASH global Interrupt                               */
#define  BSP_INT_ID_RCC                                    5    /* RCC global Interrupt                                 */
#define  BSP_INT_ID_EXTI0                                  6    /* EXTI Line0 Interrupt                                 */
#define  BSP_INT_ID_EXTI1                                  7    /* EXTI Line1 Interrupt                                 */
#define  BSP_INT_ID_EXTI2                                  8    /* EXTI Line2 Interrupt                                 */
#define  BSP_INT_ID_EXTI3                                  9    /* EXTI Line3 Interrupt                                 */
#define  BSP_INT_ID_EXTI4                                 10    /* EXTI Line4 Interrupt                                 */
#define  BSP_INT_ID_DMA1_CH1                              11    /* DMA1 Channel 1 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH2                              12    /* DMA1 Channel 2 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH3                              13    /* DMA1 Channel 3 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH4                              14    /* DMA1 Channel 4 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH5                              15    /* DMA1 Channel 5 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH6                              16    /* DMA1 Channel 6 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH7                              17    /* DMA1 Channel 7 global Interrupt                      */
#define  BSP_INT_ID_ADC1_2                                18    /* ADC1 et ADC2 global Interrupt                        */
#define  BSP_INT_ID_USB_HP_CAN_TX                         19    /* USB High Priority or CAN TX Interrupts               */
#define  BSP_INT_ID_USB_LP_CAN_RX0                        20    /* USB Low Priority or CAN RX0 Interrupts               */
#define  BSP_INT_ID_CAN_RX1                               21    /* CAN RX1 Interrupt                                    */
#define  BSP_INT_ID_CAN_SCE                               22    /* CAN SCE Interrupt                                    */
#define  BSP_INT_ID_EXTI9_5                               23    /* External Line[9:5] Interrupts                        */
#define  BSP_INT_ID_TIM1_BRK                              24    /* TIM1 Break Interrupt                                 */
#define  BSP_INT_ID_TIM1_UP                               25    /* TIM1 Update Interrupt                                */
#define  BSP_INT_ID_TIM1_TRG_COM                          26    /* TIM1 Trigger and Commutation Interrupt               */
#define  BSP_INT_ID_TIM1_CC                               27    /* TIM1 Capture Compare Interrupt                       */
#define  BSP_INT_ID_TIM2                                  28    /* TIM2 global Interrupt                                */
#define  BSP_INT_ID_TIM3                                  29    /* TIM3 global Interrupt                                */
#define  BSP_INT_ID_TIM4                                  30    /* TIM4 global Interrupt                                */
#define  BSP_INT_ID_I2C1_EV                               31    /* I2C1 Event Interrupt                                 */
#define  BSP_INT_ID_I2C1_ER                               32    /* I2C1 Error Interrupt                                 */
#define  BSP_INT_ID_I2C2_EV                               33    /* I2C2 Event Interrupt                                 */
#define  BSP_INT_ID_I2C2_ER                               34    /* I2C2 Error Interrupt                                 */
#define  BSP_INT_ID_SPI1                                  35    /* SPI1 global Interrupt                                */
#define  BSP_INT_ID_SPI2                                  36    /* SPI2 global Interrupt                                */
#define  BSP_INT_ID_USART1                                37    /* USART1 global Interrupt                              */
#define  BSP_INT_ID_USART2                                38    /* USART2 global Interrupt                              */
#define  BSP_INT_ID_USART3                                39    /* USART3 global Interrupt                              */
#define  BSP_INT_ID_EXTI15_10                             40    /* External Line[15:10] Interrupts                      */
#define  BSP_INT_ID_RTCAlarm                              41    /* RTC Alarm through EXTI Line Interrupt                */
#define  BSP_INT_ID_USBWakeUp                             42    /* USB WakeUp from suspend through EXTI Line Interrupt  */
#define  BSP_INT_ID_TIM8_BRK                              43    /* TIM8 Break Interrupt                                 */
#define  BSP_INT_ID_TIM8_UP                               44    /* TIM8 Update Interrupt                                */
#define  BSP_INT_ID_TIM8_TRG_COM                          45    /* TIM8 Trigger and Commutation Interrupt               */
#define  BSP_INT_ID_TIM8_CC                               46    /* TIM8 Capture Compare Interrupt                       */
#define  BSP_INT_ID_ADC3                                  47    /* ADC3 global Interrupt                                */
#define  BSP_INT_ID_FSMC                                  48    /* FSMC global Interrupt                                */
#define  BSP_INT_ID_SDIO                                  49    /* SDIO global Interrupt                                */
#define  BSP_INT_ID_TIM5                                  50    /* TIM5 global Interrupt                                */
#define  BSP_INT_ID_SPI3                                  51    /* SPI3 global Interrupt                                */
#define  BSP_INT_ID_UART4                                 52    /* UART4 global Interrupt                               */
#define  BSP_INT_ID_UART5                                 53    /* UART5 global Interrupt                               */
#define  BSP_INT_ID_TIM6                                  54    /* TIM6 global Interrupt                                */
#define  BSP_INT_ID_TIM7                                  55    /* TIM7 global Interrupt                                */
#define  BSP_INT_ID_DMA2_CH1                              56    /* DMA2 Channel 1 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH2                              57    /* DMA2 Channel 2 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH3                              58    /* DMA2 Channel 3 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH4_5                            59    /* DMA2 Channel 4 and DMA2 Channel 5 global Interrupt   */

/*
*********************************************************************************************************
*                                             PERIPH DEFINES
*********************************************************************************************************
*/

#define  BSP_PERIPH_ID_DMA1                                0
#define  BSP_PERIPH_ID_DMA2                                1
#define  BSP_PERIPH_ID_SRAM                                2
#define  BSP_PERIPH_ID_FLITF                               4
#define  BSP_PERIPH_ID_CRC                                 6
#define  BSP_PERIPH_ID_FSMC                                8
#define  BSP_PERIPH_ID_SDIO                               10

#define  BSP_PERIPH_ID_AFIO                               32
#define  BSP_PERIPH_ID_IOPA                               34
#define  BSP_PERIPH_ID_IOPB                               35
#define  BSP_PERIPH_ID_IOPC                               36
#define  BSP_PERIPH_ID_IOPD                               37
#define  BSP_PERIPH_ID_IOPE                               38
#define  BSP_PERIPH_ID_IOPF                               39
#define  BSP_PERIPH_ID_IOPG                               40
#define  BSP_PERIPH_ID_ADC1                               41
#define  BSP_PERIPH_ID_ADC2                               42
#define  BSP_PERIPH_ID_TIM1                               43
#define  BSP_PERIPH_ID_SPI1                               44
#define  BSP_PERIPH_ID_TIM8                               45
#define  BSP_PERIPH_ID_USART1                             46
#define  BSP_PERIPH_ID_ADC3                               47

#define  BSP_PERIPH_ID_TIM2                               64
#define  BSP_PERIPH_ID_TIM3                               65
#define  BSP_PERIPH_ID_TIM4                               66
#define  BSP_PERIPH_ID_TIM5                               67
#define  BSP_PERIPH_ID_TIM6                               68
#define  BSP_PERIPH_ID_TIM7                               69
#define  BSP_PERIPH_ID_WWDG                               75
#define  BSP_PERIPH_ID_SPI2                               78
#define  BSP_PERIPH_ID_SPI3                               79
#define  BSP_PERIPH_ID_USART2                             81
#define  BSP_PERIPH_ID_USART3                             82
#define  BSP_PERIPH_ID_USART4                             83
#define  BSP_PERIPH_ID_USART5                             84
#define  BSP_PERIPH_ID_I2C1                               85
#define  BSP_PERIPH_ID_I2C2                               86
#define  BSP_PERIPH_ID_USB                                87
#define  BSP_PERIPH_ID_CAN                                89
#define  BSP_PERIPH_ID_BKP                                91
#define  BSP_PERIPH_ID_PWR                                92
#define  BSP_PERIPH_ID_DAC                                93

/*
*********************************************************************************************************
*                                               DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                 MACRO'S
*********************************************************************************************************
*/

/************************************************************************\
*    Low-level LED- and EXTI - IO functions
\************************************************************************/

extern void           BSP_SetLED(unsigned int LED_bitpattern);

//extern void           BSP_SetEXTIPR( unsigned int EXTI_bitpattern );
//extern unsigned int   BSP_GetEXTIPR( void );


/************************************************************************\
*    Low-level single character IO to via stdin/stdout, e.g. USART#1
\************************************************************************/

//extern char   _putch (char c);// send char to console
extern char   _putch (int c);  // send char to console
extern char   _getch (void);   // read char from console
extern char   _keyhit(void);   // 0 if no input, else char from console 



/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void         BSP_Init                    (void);

void         BSP_IntDisAll               (void);

CPU_INT32U   BSP_CPU_ClkFreq             (void);

/*
*********************************************************************************************************
*                                           INTERRUPT SERVICES
*********************************************************************************************************
*/

void         BSP_IntInit                 (void);

void         BSP_IntEn                   (CPU_DATA       int_id);

void         BSP_IntDis                  (CPU_DATA       int_id);

void         BSP_IntClr                  (CPU_DATA       int_id);

void         BSP_IntVectSet              (CPU_DATA       int_id,
                                          CPU_FNCT_VOID  isr);

void         BSP_IntPrioSet              (CPU_DATA       int_id,
                                          CPU_INT08U     prio);

void         WWDG_IRQHandler           (void);
void         PVD_IRQHandler            (void);
void         TAMPER_IRQHandler         (void);
void         RTC_IRQHandler            (void);
void         FLASH_IRQHandler          (void);
void         RCC_IRQHandler            (void);
void         EXTI0_IRQHandler          (void);
void         EXTI1_IRQHandler          (void);
void         EXTI2_IRQHandler          (void);
void         EXTI3_IRQHandler          (void);
void         EXTI4_IRQHandler          (void);
void         DMAChannel1_IRQHandler    (void);
void         DMAChannel2_IRQHandler    (void);
void         DMAChannel3_IRQHandler    (void);
void         DMAChannel4_IRQHandler    (void);
void         DMAChannel5_IRQHandler    (void);
void         DMAChannel6_IRQHandler    (void);  
void         DMAChannel7_IRQHandler    (void);

void         ADC_IRQHandler            (void);
void         USB_HP_CAN_TX_IRQHandler  (void);
void         USB_LP_CAN_RX0_IRQHandler (void);
void         CAN_RX1_IRQHandler        (void);
void         CAN_SCE_IRQHandler        (void);
void         EXTI9_5_IRQHandler        (void);
void         TIM1_BRK_IRQHandler       (void);
void         TIM1_UP_IRQHandler        (void);
void         TIM1_TRG_COM_IRQHandler   (void);
void         TIM1_CC_IRQHandler        (void);
void         TIM2_IRQHandler           (void);
void         TIM3_IRQHandler           (void);
void         TIM4_IRQHandler           (void);
void         I2C1_EV_IRQHandler        (void);
void         I2C1_ER_IRQHandler        (void);
void         I2C2_EV_IRQHandler        (void);
void         I2C2_ER_IRQHandler        (void);

void         SPI1_IRQHandler           (void);
void         SPI2_IRQHandler           (void);
void         USART1_IRQHandler         (void);
void         USART2_IRQHandler         (void);
void         USART3_IRQHandler         (void);
void         EXTI15_10_IRQHandler      (void);
void         RTCAlarm_IRQHandler       (void);
void         USBWakeUp_IRQHandler      (void);

/*
*********************************************************************************************************
*                                     PERIPHERAL POWER/CLOCK SERVICES
*********************************************************************************************************
*/

CPU_INT32U   BSP_PeriphClkFreqGet        (CPU_DATA       pwr_clk_id);

void         BSP_PeriphEn                (CPU_DATA       pwr_clk_id);

void         BSP_PeriphDis               (CPU_DATA       pwr_clk_id);



/*
*********************************************************************************************************
*                                               MODULE END
*********************************************************************************************************
*/

#endif                                                          /* End of module include.                               */
