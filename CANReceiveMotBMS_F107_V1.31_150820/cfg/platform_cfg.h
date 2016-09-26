/*
 ******************************************************************************
 * @file    platform_config.h 
 * @author  MCD Application Team
 * @version V3.4.0
 * @date    10/15/2010
 * @brief   Evaluation board specific configuration file.
 ******************************************************************************
 * @copy
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
 *
 * Modified:
 * 23.05.2011  Mooshammer/Maier for mp3-player project on MCBSTM32C board
 * 10.07.2011  Mys  Adaptations for RTSys-LabEx#2 sample code
*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Uncomment the line corresponding to the STMicroelectronics evaluation board
   used to run application code
*/

#if !defined (USE_MCBSTM32C) && !defined (USE_MCBSTM32)
//#define USE_MCBSTM32
 #define USE_MCBSTM32C 
#endif


#ifdef    USE_MCBSTM32C
  #define HW_TARGET  "STM32F107"               //!< target MCU string
  #define MESSAGE1   "STM32 Connectivity Line Device 107VC "
  #define MESSAGE2   "running on MCBSTM32C\r\n"
#elif defined (USE_MCBSTM32)
  #define HW_TARGET  "STM32F103"               //!< target MCU string
  #define MESSAGE1   "STM32 Medium Density Device 103RB "
  #define MESSAGE2   "running on MCBSTM32\r\n"
#endif


/* Define STM32F10x peripherals depending on evaluation board used */

#if defined USE_MCBSTM32

/*********************** UART (Terminal) Interface  ***************************/
  #define USARTx               USART1                 // USART1 for console SIO
  #define USARTx_GPIO          GPIOA
  #define USARTx_CLK           RCC_APB2Periph_USART1
  #define USARTx_GPIO_CLK      RCC_APB2Periph_GPIOA
  #define USARTx_RxPin         GPIO_Pin_10
  #define USARTx_TxPin         GPIO_Pin_9
  #define USARTx_BDRate        9600

/*********************** LEDs / Stepmotor Control *****************************/
  #define GPIO_LED             GPIOB                  // GPIOB used for LEDs
  #define GPIO_LED_CLK         RCC_APB2Periph_GPIOB   

/*********************** usec-TIMER as Stopwatch ******************************/
/* #endif USE_MCBSTM32 */


#elif defined USE_MCBSTM32C

/*********************** UART (Terminal) Interface  ***************************/
  #define USARTx               USART2                 // USART2 for console SIO
  #define USARTx_GPIO          GPIOD                  // GPIOD
  #define USARTx_CLK           RCC_APB1Periph_USART2
  #define USARTx_GPIO_CLK      RCC_APB2Periph_GPIOD
  
  #define USARTx_CTSPin        GPIO_Pin_3             // PD.03
  #define USARTx_RTSPin        GPIO_Pin_4             // PD.04
  #define USARTx_TxPin         GPIO_Pin_5             // PD.05
  #define USARTx_RxPin         GPIO_Pin_6             // PD.06
  #define USARTx_CKPin         GPIO_Pin_7             // PD.07
  #define USARTx_BDRate        9600   
//#define USARTx_BDRate        115200   

/*********************** LEDs / Stepmotor Control *****************************/

  #define GPIO_LED             GPIOE                  // GPIOE used for LEDs
  #define GPIO_LED_CLK         RCC_APB2Periph_GPIOE   

/*********************** usec-TIMER as Stopwatch ******************************/

 
  #define GPIO_LED_1           GPIO_Pin_8             // PE.08
  #define GPIO_LED_2           GPIO_Pin_9             // PE.09
  #define GPIO_LED_3           GPIO_Pin_10            // PE.10
  #define GPIO_LED_4           GPIO_Pin_11            // PE.11
  #define GPIO_LED_5           GPIO_Pin_12            // PE.12
  #define GPIO_LED_6           GPIO_Pin_13            // PE.13
  #define GPIO_LED_7           GPIO_Pin_14            // PE.14
  #define GPIO_LED_8           GPIO_Pin_15            // PE.15


/*********************** SD_CARD SPI Interface  *******************************/
/* ACHTUNG:
    SD_CARD Init muss noch in SPI_STM32F107.c entsprechend den hier gemachten
    Definitionen angepasst werden. Auch die dort vorgenommene Initialisierung  
    muss noch geprüft werden!
    Prüfen wo eine DMA Initialisierung für das Auslesen der SDCard möglich ist
*/
  
  #define SD_SPI               SPI1
  #define SD_SPI_CLK           RCC_APB2Periph_SPI1

  #define SD_SPI_GPIO_PORT     GPIOA                  // GPIOA
  #define SD_SPI_GPIO_CLK      RCC_APB2Periph_GPIOA
  
  #define SD_SPI_SCK_PIN       GPIO_Pin_5             // PA.05 
  #define SD_SPI_MISO_PIN      GPIO_Pin_6             // PA.06
  #define SD_SPI_MOSI_PIN      GPIO_Pin_7             // PA.07
  #define SD_CS_PIN            GPIO_Pin_4             // PA.04
  
  #define SD_DETECT_PIN        GPIO_Pin_0             // PE.00 
  #define SD_DETECT_GPIO_PORT  GPIOE                  // GPIOE 
  #define SD_DETECT_GPIO_CLK   RCC_APB2Periph_GPIOE

  #define CRCPolynomial                 7
  
 /* DMA for SDCARD-READ-SPI */
  #define USE_DMA_FOR_SDCARD_READ   

  #define SD_SPI_DMA            DMA1
  #define SD_SPI_DMA_CLK        RCC_AHBPeriph_DMA1
  #define SD_SPI_Rx_DMA_Channel DMA1_Channel2
  #define SD_SPI_Rx_DMA_FLAG    DMA1_FLAG_TC2
  #define SD_SPI_Tx_DMA_Channel DMA1_Channel3
  #define SD_SPI_Tx_DMA_FLAG    DMA1_FLAG_TC3  
  #define SD_SPI_DR_Base        0x4001300C 
  #define SD_SPI_Tx_DMA_Request SPI_I2S_DMAReq_Tx
  #define SD_SPI_Rx_DMA_Request SPI_I2S_DMAReq_Rx     

/********************** I2S AUDIO-CODEC Interface *****************************/
  /* I2S */
  #define CODEC_I2S             SPI2
  #define CODEC_I2S_PORT        GPIOB                 // GPIOB
  #define CODEC_I2S_CMD         GPIO_Pin_12           // PB.12
  #define CODEC_I2S_SCLK        GPIO_Pin_13           // PB.13
  #define CODEC_I2S_SDIO        GPIO_Pin_15           // PB.15
  #define CODEC_I2S_CLK         RCC_APB1Periph_SPI2
  #define CODEC_I2S_GPIO_CLK    RCC_APB2Periph_GPIOB

  /* Master-CLK for CODEC-I2S --> 256*Fs */
  #define CODEC_MCK_PORT        GPIOC                 // GPIOB
  #define CODEC_MCK_PIN         GPIO_Pin_6            // PC.06
  #define CODEC_MCK_GPIO_CLK    RCC_APB2Periph_GPIOC

  /* DMA for CODEC-I2S */
  #define CODEC_I2S_DMA            DMA1
  #define CODEC_I2S_DMA_CLK        RCC_AHBPeriph_DMA1
  #define CODEC_I2S_Rx_DMA_Channel DMA1_Channel4
  #define CODEC_I2S_Rx_DMA_FLAG    DMA1_FLAG_TC4
  #define CODEC_I2S_Tx_DMA_Channel DMA1_Channel5
  #define CODEC_I2S_Tx_DMA_FLAG    DMA1_FLAG_TC5  
  #define CODEC_I2S_DR_Base        0x4000380C 
  #define CODEC_I2S_Tx_DMA_Request SPI_I2S_DMAReq_Tx
 
  /* IRQ for DMA-Transfer CODEC-I2S */
  #define CODEC_I2S_DMA_IRQ                      DMA1_Channel5_IRQn
  #define CODEC_I2S_DMA_Tx_Complete_IRQ          DMA_IT_TC         
  
  #define CODEC_I2S_DMA_GLOBAL_IRQ_FLAG          DMA1_IT_GL5                    
  #define CODEC_I2S_DMA_Tx_COMPLETE_IRQ_FLAG     DMA1_IT_TC5                       
  #define CODEC_I2S_DMA_HALFTx_COMPLETE_IRQ_FLAG DMA1_IT_HT5                        
  #define CODEC_I2S_DMA_Tx_ERROR_IRQ_FLAG        DMA1_IT_TE5
                      
  /* IRQ for CODEC-I2S */
  #define CODEC_I2S_IRQ                          SPI2_IRQn
  #define CODEC_I2S_Tx_BUF_Empty_IRQ             SPI_I2S_IT_TXE

/********************** I2C AUDIO-CODEC Interface *****************************/
  /* I2C */
  #define CODEC_I2C            I2C1
  #define CODEC_I2C_PORT       GPIOB                  // GPIOB
  #define CODEC_I2C_SCL_PIN    GPIO_Pin_8             // PB.08
  #define CODEC_I2C_SDA_PIN    GPIO_Pin_9             // PB.09
  #define CODEC_I2C_CLK        RCC_APB1Periph_I2C1
  #define CODEC_I2C_GPIO_CLK   RCC_APB2Periph_GPIOB

  #define I2C_Speed            100000                 // max. 100kHz bus speed to CODEC allowed
  #define I2C_SLAVE_ADDRESS7   0xA0                   // I2C own address if in slave mode

#endif /* USE_MCBSTM32C */

/******************************************************************************/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
