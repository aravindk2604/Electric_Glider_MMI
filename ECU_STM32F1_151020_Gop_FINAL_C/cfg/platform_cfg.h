/*
 ******************************************************************************
 * @file    platform_cfg.h 
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
 * 10.07.2011  Mys  Adaptations for RTSys-LabEx#2 sample code
 * 12.11.2013  Imr  Defination added for porting to Discovery STM32F407VG
 * 25.09.2014  Janezic Sommerauer; added new porting for STM32F4 and STM321xx Families
 * 15.10.2014  Janezic Sommerauer: Added global variable base&offset address for CAN
 * 10.02.2014  Janezic Sommerauer: Added SD definitions for SD
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


#if !defined (USE_MCBSTM32C) && !defined (USE_MCBSTM32) && !defined (USE_DISCOVERY)
  #undef USE_MCBSTM32
  #define USE_MCBSTM32C 
  #undef USE_DISCOVERY                                  //!< STM32F4 board specific files
#endif


#ifdef    USE_MCBSTM32C
  #define HW_TARGET  "STM32F107"                        //!< target MCU string : STM32F107
  #define MESSAGE1   "STM32 Connectivity Line Device 107VC "
  #define MESSAGE2   "running on MCBSTM32C\r\n"
#elif defined (USE_MCBSTM32)
  #define HW_TARGET  "STM32F103"                        //!< target MCU string : STM32F103
  #define MESSAGE1   "STM32 Medium Density Device 103RB "
  #define MESSAGE2   "running on MCBSTM32\r\n"
#elif defined USE_DISCOVERY
  #define HW_TARGET  "STM32F407VG"                      //!< target MCU string : STM32F407VG
  #define MESSAGE1   "STM32F407VG Connectivity line device"
  #define MESSAGE2   "running on DISCOVERY board\r\n"
#else
  #error: "Illegal target board selection!"
#endif







/*********************** CAN address  ***************************/
#define BaseAdr       0x14A10000                          //!< Base address for all CAN messages
#define OffsetAdr_Bat 0x0101                              //!< Offset added for Battery specific address
#define OffsetAdr_Mot 0x0041                              //!< Offset added for Motor specific address (Config param P15)

/* Define STM32F10x peripherals depending on evaluation board used */

#if defined USE_MCBSTM32

/*********************** UART (Terminal) Interface  ***************************/
  #define USART_x              USART1                     //!< USART1 for console SIO
  #define USARTx_GPIO          GPIOA                      //!< GPIO A port for USART
  #define USARTx_CLK           RCC_APB2Periph_USART1      //!< USART clock definition
  #define USARTx_GPIO_CLK      RCC_APB2Periph_GPIOA       //!< GPIO clock definition
  #define USARTx_RxPin         GPIO_Pin_10                //!< Recieve Pin number
  #define USARTx_TxPin         GPIO_Pin_9                 //!< Transmission Pin number 
  #define USARTx_BDRate        9600                       //!< Baud rate

/*********************** LEDs / Stepmotor Control *****************************/
  #define GPIO_LED             GPIOB                      //!< GPIOB used for LEDs
  #define GPIO_LED_CLK         RCC_APB2Periph_GPIOB       //!< GPIO clock defintion 
                                                        
/*********************** usec-TIMER as Stopwatch ******************************/
/* #endif USE_MCBSTM32 */







#elif defined USE_MCBSTM32C
/************************** CAN bus interface *******************************/
  #define CAN_x                 CAN1                      //!< CAN1 
  #define CANx_GPIO             GPIOD                     //!< Using CAN1 at GPIOD
  #define CANx_RxPin            GPIO_Pin_0                //!< RX signal at PD.0
  #define CANx_TxPin            GPIO_Pin_1                //!< TX signal at PD.1
                                                        
                                                        
  #define CANx_CLK              RCC_APB1Periph_CAN1       //!< Using RCC to enable clock for CAN1
  #define CANx_GPIO_CLK         RCC_APB2Periph_GPIOD      //!< Using RCC to enable clock for GPIOD
  #define CANx_AF_CLK           RCC_APB2Periph_AFIO       //!< Remap alternate GPIO function to CAN
  #define CANx_REMAP1           GPIO_Remap1_CAN1          //!< Configures PD0-1
  #define CANx_REMAP2           GPIO_Remap2_CAN1          //!< Remaps PD0-1


/*********************** USART (Terminal) Interface  ***************************/
  #define USART_x              USART2                     //!< USART2 for console SIO
  #define USARTx_GPIO          GPIOD                      //!< GPIOD
  #define USARTx_CLK           RCC_APB1Periph_USART2      //!< USART6 used RCC_APB1, in case another USART is used, might might change to RCC_APBx
  #define USARTx_GPIO_CLK      RCC_APB2Periph_GPIOD       //!< Using RCC to enable clock for GPIOD
  #define USARTx_AF_CLK        RCC_APB2Periph_AFIO        //!< Using RCC to enable clock for GPIOD alternate function
  #define USARTx_AF_Remap      GPIO_Remap_USART2          //!< Alternate function USART mapping
                                                        
  #define USARTx_CTSPin        GPIO_Pin_3                 //!< PD.03
  #define USARTx_RTSPin        GPIO_Pin_4                 //!< PD.04
  #define USARTx_TxPin         GPIO_Pin_5                 //!< PD.05 
  #define USARTx_RxPin         GPIO_Pin_6                 //!< PD.06
  #define USARTx_CKPin         GPIO_Pin_7                 //!< PD.07
  #define USARTx_BDRate        9600                       //!< Baudrate = 9600

/************************** ADC Configuration ********************************/
  #define ADC_x                ADC1                       //!< ADC1 for conversion of poti value for throttle
  #define ADCx_GPIO            GPIOC                      //!< GPIOC
  #define ADCx_Pin             GPIO_Pin_4                 //!< Pin PC.4
                                                        
   #define ADCx_CLK            RCC_APB2Periph_ADC1        //!< Clock for ADCx
   #define ADCx_GPIO_CLK       RCC_APB2Periph_GPIOA       //!< Clock for GPIOx
   #define ADCx_AF             GPIO_Mode_AIN              //!< Alternate GPIO function used for AD conversion is ANALOG
   #define ADC_Channel_x       ADC_Channel_14             //!< ADC channel    
   #define ADC_SampleTime_x    ADC_SampleTime_55Cycles5   //!< ADC sample time


/*********************** LEDs and GPIOs ****************************************/
  #define GPIO_LED             GPIOE                      //!< GPIOE used for LEDs
  #define GPIO_LED_CLK         RCC_APB2Periph_GPIOE       //!< Clock for GPIO_LED

  #define GPIO_LED_1           GPIO_Pin_8                 //!< PE.08
  #define GPIO_LED_2           GPIO_Pin_9                 //!< PE.09
  #define GPIO_LED_3           GPIO_Pin_10                //!< PE.10
  #define GPIO_LED_4           GPIO_Pin_11                //!< PE.11
  #define GPIO_LED_5           GPIO_Pin_12                //!< PE.12
  #define GPIO_LED_6           GPIO_Pin_13                //!< PE.13
  #define GPIO_LED_7           GPIO_Pin_14                //!< PE.14
  #define GPIO_LED_8           GPIO_Pin_15                //!< PE.15
 
  #define STATUS_LED           GPIO_LED_1                 //!< Status Led = GPIO_LED_1
  #define UI_LED               GPIO_LED_3                 //!< UI     Led = GPIO_LED_2

  #define GPIO_UI_BTN         GPIOB                      //!< GPIOB used for UI_BTN
  #define GPIO_UI_BTN_CLK     RCC_APB2Periph_GPIOB       //!< Clock for GPIOx
  #define UI_BTN              GPIO_Pin_7                 //!< UI Button = Pin7
  #define UI_BTN_EXTI_PORTSRC GPIO_PortSourceGPIOB       //!< UI Button Port Source= GPIOB
  #define UI_BTN_EXTI_PINSRC  GPIO_PinSource7            //!< UI Button Pin Source= Pin7
  #define UI_BTN_EXTI_LINE    EXTI_Line7                 //!< PB7 is connected to EXTI_Line7

  #define GPIO_UI_CLK         RCC_APB2Periph_GPIOB       //!< UI alert state CLK
  #define UIx_GPIO            GPIOB                      //!< UI alert state GPIO 
  #define UIx_GPIO_pin        GPIO_Pin_8                 //!< UI alert state GPIO pin 


/*********************** SD_CARD SPI Interface  *******************************/
  #define SD_CARD_FILE_NAME    "data.csv"                 //!< Name of SDcard csv CAN data file. Name must not contain underscores, can contain A_z0_9  
  
  #define  FATFS_USE_SDIO      0                          //!< Do not use SDIO communication with SDCard - will use SPI
                                                                                                             
  //SPI and GPIO clocks
  #define GPIO_SPI_CLK         RCC_APB2Periph_GPIOA       //!< Clock for GPIOx
  #define SPI_CLK	             RCC_APB2Periph_SPI1        //!< Clock for SPIx
  
  //SPI initialization specific definitions
  #define SPI_SDx				       SPI1                       //!< SPI USED
  #define GPIO_SDx			       GPIOA                      //!< GPIO used for SPIx 
  #define GPIO_AF_SPIx  	     GPIO_AF_SPI1               //!< Alternate function used on GPIOx SPIx 
  
  //SPI GPIO pin values
  #define GPIO_SPI_SCK_pin	   GPIO_Pin_5                 //!< SCK pin  = PA.5
  #define GPIO_SPI_MISO_pin	   GPIO_Pin_6                 //!< MISO pin = PA.6
  #define GPIO_SPI_MOSI_pin	   GPIO_Pin_7                 //!< MOSI pin = PA.7
  
  //SPI GPIO pin source values
  #define GPIO_SPI_SCK  	     GPIO_PinSource5            //!< Pin source adress for PA.5
  #define GPIO_SPI_MISO		     GPIO_PinSource6            //!< Pin source adress for PA.6
  #define GPIO_SPI_MOSI		     GPIO_PinSource7            //!< Pin source adress for PA.7

  // CS pin for SPI communication                 
  #define SPI_CS_CLK           RCC_APB2Periph_GPIOA       //!< Using RCC to enable clock for GPIOA
  #define SPI_CS_PORT          GPIOA                      //!< ChipSelect port GPIOA
  #define SPI_CS_PIN           GPIO_Pin_4                 //!< ChipSelect pin 4

/*#endif USE_MCBSTM32C */





#elif defined USE_DISCOVERY

/************************** CAN bus interface *******************************/
  #define CAN_x                CAN1                       //!< CAN1 
  #define CANx_GPIO            GPIOB                      //!< Using CAN1 at GPIOB
  #define CANx_RxPin           GPIO_Pin_8                 //!< RX signal at PB.8
  #define CANx_TxPin           GPIO_Pin_9                 //!< TX signal at PB.9
  #define CANx_GPIO_SRC_RX     GPIO_PinSource8            //!< Pin source adress for PD.8
  #define CANx_GPIO_SRC_TX     GPIO_PinSource9            //!< Pin source adress for PD.9
                                                        
  #define CANx_CLK             RCC_APB1Periph_CAN1        //!< Using RCC to enable clock for CAN1
  #define CANx_GPIO_CLK        RCC_AHB1Periph_GPIOB       //!< Using RCC to enable clock for GPIOB
  #define CANx_AF              GPIO_AF_CAN1               //!< Remap alternate GPIO function to CAN
  

/*********************** UART (Terminal) Interface  **************************/
  #define USART_x              USART6                     //!< USART6 for console SIO
  #define USARTx_GPIO          GPIOC                      //!< GPIOC
  #define USARTx_TxPin         GPIO_Pin_6                 //!< TX signal at PC.6
  #define USARTx_RxPin         GPIO_Pin_7                 //!< RX signal at PC.7
  #define USARTx_GPIO_SRC_TX   GPIO_PinSource6            //!< Pin source adress for PC.6
  #define USARTx_GPIO_SRC_RX   GPIO_PinSource7            //!< Pin source adress for PC.7
                                                        
  #define USARTx_CLK           RCC_APB2Periph_USART6      //!< USART6 used RCC_APB2, in case another USART ist used might change to RCC_APBx
  #define USARTx_GPIO_CLK      RCC_AHB1Periph_GPIOC       //!< Using RCC to enable clock for GPIOC
  #define USARTx_AF            GPIO_AF_USART6             //!< Remap alternate GPIO function to USART

  #define USARTx_BDRate        9600                       //!< Baudrate = 9600
  
  
/************************** ADC Configuration ********************************/
   #define ADC_x               ADC1                       //!< ADC1 for conversion of poti value for throttle
   #define ADCx_GPIO           GPIOA                      //!< GPIOA
   #define ADCx_Pin            GPIO_Pin_1                 //!< Pin PA.1
   
   #define ADCx_CLK            RCC_APB2Periph_ADC1        //!< Clock for ADCx
   #define ADCx_GPIO_CLK       RCC_AHB1Periph_GPIOA       //!< Clock for GPIOx
   #define ADCx_AF             GPIO_Mode_AN               //!< Alternate GPIO function used for AD conversion is ANALOG
   #define ADC_Channel_x       ADC_Channel_1              //!< ADC channel    
   #define ADC_SampleTime_x    ADC_SampleTime_56Cycles    //!< ADC sample time
  

/*********************** LEDs and GPIOs ****************************************/
   #define GPIO_LED            GPIOD                      //!< GPIOD used for LEDs
   #define GPIO_LED_CLK        RCC_AHB1Periph_GPIOD       //!< Clock for GPIOx
   #define STATUS_LED          LED4                       //!< Status Led = LED4 
   #define UI_LED              LED3                       //!< UI     Led = LED3
       
   #define GPIO_UI_BTN         GPIOA                      //!< GPIOA used for UI_BTN
   #define GPIO_UI_BTN_CLK     RCC_AHB1Periph_GPIOA       //!< Clock for GPIOx
   #define UI_BTN              GPIO_Pin_0                 //!< UI Button = Pin0
   #define UI_BTN_EXTI_PORTSRC EXTI_PortSourceGPIOA       //!< UI Button Port Source= GPIOA
   #define UI_BTN_EXTI_PINSRC  EXTI_PinSource0            //!< UI Button Pin Source= Pin0
   #define UI_BTN_EXTI_LINE    EXTI_Line0                 //!< PA0 is connected to EXTI_Line0

   #define GPIO_UI_CLK         RCC_AHB1Periph_GPIOA       //!< UI alert state CLK
   #define UIx_GPIO            GPIOA                      //!< UI alert state GPIO 
   #define UIx_GPIO_pin        GPIO_Pin_2                 //!< UI alert state GPIO pin 



/*********************** SD_CARD SPI Interface  *******************************/
  #define SD_CARD_FILE_NAME    "data.csv"                 //!< Name of SDcard csv CAN data file. Name must not contain underscores, can contain A_z0_9  
  
  #define  FATFS_USE_SDIO      0                          //!< Do not use SDIO communication with SDCard - will use SPI
                                                                                                             
  //SPI and GPIO clocks
  #define GPIO_SPI_CLK         RCC_AHB1Periph_GPIOA       //!< Clock for GPIOx
  #define SPI_CLK	             RCC_APB2Periph_SPI1        //!< Clock for SPIx
  
  //SPI initialization specific definitions
  #define SPI_SDx				       SPI1                       //!< SPI USED
  #define GPIO_SDx			       GPIOA                      //!< GPIO used for SPIx 
  #define GPIO_AF_SPIx  	     GPIO_AF_SPI1               //!< Alternate function used on GPIOx SPIx 
  
  //SPI GPIO pin values
  #define GPIO_SPI_SCK_pin	   GPIO_Pin_5                 //!< SCK pin  = PA.5
  #define GPIO_SPI_MISO_pin	   GPIO_Pin_6                 //!< MISO pin = PA.6
  #define GPIO_SPI_MOSI_pin	   GPIO_Pin_7                 //!< MOSI pin = PA.7
  
  //SPI GPIO pin source values
  #define GPIO_SPI_SCK  	     GPIO_PinSource5            //!< Pin source adress for PA.5
  #define GPIO_SPI_MISO		     GPIO_PinSource6            //!< Pin source adress for PA.6
  #define GPIO_SPI_MOSI		     GPIO_PinSource7            //!< Pin source adress for PA.7

  // CS pin for SPI communication                 
  #define SPI_CS_CLK           RCC_AHB1Periph_GPIOB       //!< Using RCC to enable clock for GPIOB
  #define SPI_CS_PORT          GPIOB                      //!< ChipSelect port GPIOB
  #define SPI_CS_PIN           GPIO_Pin_5                 //!< ChipSelect pin 5
 
  /*# endif USE_DISCOVERY */
#else
  #error: "Illegal target board selection!"
#endif


/******************************************************************************/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
