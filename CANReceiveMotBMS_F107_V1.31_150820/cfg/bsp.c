/**
*  ***********************************************************************
*  @file    BSP.c
*  @author  B. Mysliwetz
*  @version V2.00
*
*  @brief   Real-Time Systems Lab Course 
*
*  'Retarget' layer with TARGET-SPECIFIC functions for the Keil MCBSTM32
*  evalboard; contains low level serial character-IO and LED-IO functions. 
*
*  Changes:
*  16.11.2008 keil  V1.0
*  18.11.2008 wu    V1.1 Adaption of SVC Handler
*  03.03.2011 mys   V1.2 Serial IO functions for fputc(), fgetc() outsourced 
*  25.02.2013 mys   V2.0 Doxygen compatible & new project folder structure 
*
*  @date    25/02/2013
**************************************************************************
*/

#include  <stdio.h>

#include  <rt_misc.h>
//#include  <stm32f10x_lib.h>    // STM32F10x Library Definitions
#include  "stm32f10x_conf.h"
//#include  "STM32_Init.h"       // STM32 initialization package

#include  "platform_cfg.h"     // board specific definitions
void     GPIO_Configuration(void);               //!< Claim GPIO_Configuration 
void    RCC_Configuration(void);                 //!< Claim RCC_Configuration
void 	USART_Configuration( void );   // Config USART



void    BSP_SetLED ( unsigned char LED_bitpattern )
/**
*  ***********************************************************************
*  @brief  Write to IO port that controls LEDs (row of max 8 LEDs assumed).
*  @param  LED_bitpattern : 1 turns on, 0 turns off individual LED
*  @retval none
**************************************************************************
*/
{
  GPIO_LED->ODR = LED_bitpattern << 8;    // 1 turns on LED on MCBSTM32B
}


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



char  _putch (int ch)
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial port#1.
*  @param  ch : character to output
*  @retval output character
**************************************************************************
*/
{
  while ( !(USARTx->SR & USART_FLAG_TXE) );
  USARTx->DR = (ch & 0xFF);
  return (ch);
}


char  _getch (void)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial port#1 (BLOCKiNG MODE).
*  @retval input character
**************************************************************************
*/
{
  while ( !(USARTx->SR & USART_FLAG_RXNE) );
  return (USARTx->DR & 0xFF);
}


char  _keyhit (void)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial port#1 (NON-BLOCKiNG MODE).
*  @retval input character
**************************************************************************
*/
{
  if ( USARTx->SR & USART_FLAG_RXNE )
    return (USARTx->DR & 0xFF);
  else
    return (0);
}


int  fputc (int ch, FILE *f)
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial port#1.
*  @param  ch : character to output
*  @param  f  : filepointer (as yet ignored here)
*  @retval output character
**************************************************************************
*/
{
  return ( _putch(ch) );
}


int  fgetc (FILE *f)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial port#1 (BLOCKiNG MODE).
*  @param  f  : filepointer (as yet ignored here)
*  @retval input character
**************************************************************************
*/
{
  return ( _putch( _getch() ) );
}


void  _ttywrch (int ch)
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial port#1.
*  @param  ch : character to output
*  @retval none
**************************************************************************
*/
{
  _putch( ch );
}


int  ferror (FILE *f)
/**
*  ***********************************************************************
*  @brief  Return error status of file f, not yet implemented.
*  @param  f  : filepointer (as yet ignored here)
*  @retval error code (as yet const EOF)
**************************************************************************
*/
{
  // your implementation of ferror
  return EOF;
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


void  BSP_Init (void)
/**
*  ***********************************************************************
*  @brief  Initialize CPU/MCU and application relevant peripherals.
*  @retval none.
**************************************************************************
*/
{
  #if defined USE_MCBSTM32
  stm32_Init( );                  // STM32 peripherals initialization
 #elif defined USE_MCBSTM32C
  RCC_Configuration();			  // Config Clock Settings	
  GPIO_Configuration();			 // Config GPIOs and AFIO	
  USART_Configuration( );        // Config USART
  #endif		
}


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


__asm void SVC_Handler (void)
/**
*  ***********************************************************************
*  @brief  SVC_Handler.
*  @retval none.
**************************************************************************
*/
{
  PRESERVE8

  TST     LR,#4                         ; Called from Handler Mode ?
  MRSNE   R12,PSP                       ; Yes, use PSP
  MOVEQ   R12,SP                        ; No, use MSP
  LDR     R12,[R12,#24]                 ; Read Saved PC from Stack
  LDRH    R12,[R12,#-2]                 ; Load Halfword
  BICS    R12,R12,#0xFF00               ; Extract SVC Number

  PUSH    {R4,LR}                       ; Save Registers
  LDR     LR,=SVC_Count
  LDR     LR,[LR]
  CMP     R12,LR
  BHS     SVC_Dead                      ; Overflow
  LDR     LR,=SVC_Table
  LDR     R12,[LR,R12,LSL #2]           ; Load SVC Function Address
  BLX     R12                           ; Call SVC Function

  POP     {R4,LR}
  TST     LR,#4
  MRSNE   R12,PSP
  MOVEQ   R12,SP
  STM     R12,{R0-R3}                   ; Function return values
  BX      LR                            ; RETI

SVC_Dead
  B       SVC_Dead                      ; None Existing SVC

SVC_Cnt    EQU    (SVC_End-SVC_Table)/4
SVC_Count  DCD    SVC_Cnt

; Import user SVC functions here.
;*IMPORT  __SVC_0
;*IMPORT  __SVC_1
;*IMPORT  __SVC_2
;*IMPORT  __SVC_3

SVC_Table
; Insert user SVC functions here
;*DCD     __SVC_0                   ; SVC 0 Function Entry
;*DCD     __SVC_1                   ; SVC 1 Function Entry
;*DCD     __SVC_2                   ; SVC 2 Function Entry
;*DCD     __SVC_3                   ; SVC 2 Function Entry

  DCD    0                          ; dummy vectors
  DCD    0
  DCD    0
  DCD    0

SVC_End

  ALIGN
}

#ifdef      USE_MCBSTM32C

void RCC_Configuration(void)
/************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configure the Clocks of STM32.
* Input          : None
* Output         : None
* Return         : None
************************************************************************/
{ 
	ErrorStatus HSEStartUpStatus;
  
	 /* RCC system reset(for debug purpose) */
  	RCC_DeInit();

  	 /* Enable HSE */
  	RCC_HSEConfig(RCC_HSE_ON);

  	 /* Wait till HSE is ready */
  	HSEStartUpStatus = RCC_WaitForHSEStartUp();

  	if(HSEStartUpStatus == SUCCESS)
  	{
    	 /* Enable Prefetch Buffer */
    	//FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    	 /* Flash 2 wait state */
    	//FLASH_SetLatency(FLASH_Latency_2);
	
    	 /* HCLK = SYSCLK */
    	RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    	 /* PCLK2 = HCLK */
    	RCC_PCLK2Config(RCC_HCLK_Div1); 

    	 /* PCLK1 = HCLK/2 */
    	RCC_PCLK1Config(RCC_HCLK_Div2);

    	 /* ADCCLK = PCLK2/4 */
    	RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  
    	 /* Configure PLLs */
    	 /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    	RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    	RCC_PLL2Config(RCC_PLL2Mul_8);

    	 /* Enable PLL2 */
    	RCC_PLL2Cmd(ENABLE);

    	 /* Wait till PLL2 is ready */
    	while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    	{}

    	 /* PLL configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz */ 
    	RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    	RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);

    	 /* PLL3 configuration: PLL3CLK = (HSE / 5) * 11 => PLL3_VCO = 2 * PLL3CLK = 2 * 55 MHz = 110 MHz */
		//RCC_PLL3Config(RCC_PLL3Mul_11);
 	
   		 /* PLL3 configuration: PLL3CLK = (HSE / 5) * 9 => PLL3_VCO = 2 * PLL3CLK = 2 * 45 MHz = 90 MHz */
		RCC_PLL3Config(RCC_PLL3Mul_9);
    	
		 /* Enable PLL3 */
    	RCC_PLL3Cmd(ENABLE);    
    	 /* Wait till PLL3 is ready */
    	while (RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) == RESET)
    	{}

    	 /* Configure I2S clock source: On Connectivity-Line Devices, the I2S can be 
        	clocked by PLL3 VCO instead of SYS_CLK in order to guarantee higher 
        	precision */

    	RCC_I2S2CLKConfig(RCC_I2S2CLKSource_PLL3_VCO);  

    	 /* Enable PLL */ 
    	RCC_PLLCmd(ENABLE);

    	 /* Wait till PLL is ready */
    	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    	{
    	}

    	 /* Select PLL as system clock source */
    	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    	 /* Wait till PLL is used as system clock source */
    	while(RCC_GetSYSCLKSource() != 0x08)
    	{
    	}
	}  
  	else
  	{ /* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
  
	while (1);
  
    }  
 
  	 /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
  	RCC_APB2PeriphClockCmd(SD_SPI_GPIO_CLK | CODEC_I2S_GPIO_CLK | CODEC_I2C_GPIO_CLK |
                           CODEC_MCK_GPIO_CLK | USARTx_GPIO_CLK |
                           GPIO_LED_CLK | SD_DETECT_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
  

  	 /* Enable USARTx Clock */
  	RCC_APB1PeriphClockCmd(USARTx_CLK, ENABLE); 
  
  	 /*!< SD_SPI Periph clock enable */
  	RCC_APB2PeriphClockCmd(SD_SPI_CLK, ENABLE); 

  	 /* Enable the I2C1 APB1 clock */
  	//RCC_APB1PeriphClockCmd(CODEC_I2C_CLK, ENABLE);

  	 /* Enable the I2S2 APB1 clock */
  	RCC_APB1PeriphClockCmd(CODEC_I2S_CLK, ENABLE);

  	 /* Enable DMA1 */
  	RCC_AHBPeriphClockCmd(CODEC_I2S_DMA_CLK | SD_SPI_DMA_CLK, ENABLE);
}


/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configure the I2Ss NVIC channel.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

/************************************** NOT USED as yet
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	 // SPI2 IRQ Channel configuration
  	NVIC_InitStructure.NVIC_IRQChannel = CODEC_I2S_IRQ;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);	 

	 // DMA2 Channel5 IRQ Channel configuration
  	NVIC_InitStructure.NVIC_IRQChannel = CODEC_I2S_DMA_IRQ;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

}
*****************************************/



/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Initializes the USART Interface
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
 void USART_Configuration (void)
 {
	USART_InitTypeDef USART_InitStructure;			 
   	USART_StructInit(&USART_InitStructure);

   	 /* USART Settings*/
   	USART_InitStructure.USART_BaudRate = USARTx_BDRate;
   	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   	USART_InitStructure.USART_StopBits = USART_StopBits_1;
   	USART_InitStructure.USART_Parity = USART_Parity_No ;
   	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
   	 /* Configure the USARTx */ 
   	USART_Init(USARTx, &USART_InitStructure);
   	 /* Enable the USARTx */
   	USART_Cmd(USARTx, ENABLE);
 }
 
/*******************************************************************************
* Function Name  : GPIO_Config
* Description    : Initializes the GPIO pins used by the codec application.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_StructInit(&GPIO_InitStructure);
  
  /* Enable the USART2 Pins Software Remapping */
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);  

  /* Configure USARTy Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USARTx_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USARTx_GPIO, &GPIO_InitStructure);
    
  /* Configure USARTy Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = USARTx_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USARTx_GPIO, &GPIO_InitStructure);

  /* Configure GPIO Port E for LEDs*/
  /*PE0...PE7 as analog input	 */
  /*PE8...PE15 as push-pull output @ 50MHz */

  GPIO_InitStructure.GPIO_Pin = GPIO_LED_1 | GPIO_LED_2 | \
  		  						  GPIO_LED_3 | GPIO_LED_4 | \
								  GPIO_LED_5 | GPIO_LED_6 | \
								  GPIO_LED_7 | GPIO_LED_8;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIO_LED, &GPIO_InitStructure);

	/* I2S2 SD, CK and WS pins configuration */
  GPIO_InitStructure.GPIO_Pin = CODEC_I2S_SCLK | CODEC_I2S_SDIO | CODEC_I2S_CMD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(CODEC_I2S_PORT, &GPIO_InitStructure);

  /* I2S2 MCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = CODEC_MCK_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(CODEC_MCK_PORT, &GPIO_InitStructure);

	/* Enable the I2C-1 Pins Software Remapping */
  GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

  /* I2C-1 SCL PB8 and SDA PB9 pins configuration */
  GPIO_InitStructure.GPIO_Pin = CODEC_I2C_SCL_PIN | CODEC_I2C_SDA_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;  //GPIO_Mode_AF_OD;
  GPIO_Init(CODEC_I2C_PORT, &GPIO_InitStructure);

  /* Disable the SPI-1 Pins Software Remapping */
  /* No SPI1 remap (use PA5..PA7). */
  GPIO_PinRemapConfig(GPIO_Remap_SPI1, DISABLE);

  /* SPI-1 SCL PA..PA7 pins configuration */
  /* SPI1_NSS is GPIO, output set to high. */
  /* SPI1_SCK, SPI1_MISO, SPI1_MOSI are SPI pins. */
  GPIO_InitStructure.GPIO_Pin = SD_SPI_SCK_PIN | SD_SPI_MOSI_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SD_SPI_GPIO_PORT, &GPIO_InitStructure); 
	
  GPIO_InitStructure.GPIO_Pin = SD_SPI_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(SD_SPI_GPIO_PORT, &GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin = SD_CS_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SD_SPI_GPIO_PORT, &GPIO_InitStructure); 

  /* Card Sensor PE.0 input */
  /* 1 = NO Card, 0 = Card plugged. */
  GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);  
}

#endif


/************************************************************************\
*    END OF MODULE BSP.c for the Keil MCBSTM32 evalboard
\************************************************************************/
