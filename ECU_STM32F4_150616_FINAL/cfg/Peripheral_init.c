/**
*  ***********************************************************************
*  @file    Peripheral_init.c
*  @author  Janezic Sommerauer
*  @version V1.00
*
*  @brief   Initialization of all peripherals.  
*          
*  Changes:
*
*  @date    08/10/2014
**************************************************************************
*/



#ifdef USE_MCBSTM32
    #include  "stm32f10x_conf.h"      //!< STM32F10x Configuration File
    #include  "STM32_Init.h"           //!< STM32 initialization package  
#elif defined USE_MCBSTM32C
    #include  "stm32f10x_conf.h"       //!< STM32F10x Configuration File
    #include  "STM32_Init.h"            //!<  STM32 initialization package  
#elif defined USE_DISCOVERY
    #include <stm32f4xx_conf.h>
    #include "stm32f4_discovery.h"     //!< Functions specific to discovery board
    #include "stm32f4xx_conf.h"         //!< Periph. library configuration file
    #include "LcdDefs.h"           //!< LCD periph. library
    #include "SDlib.h"         //!<  added for SD SPI purposes over 1289LCD myTouch One peripheral, SPI protocols and file system
    #include "stm32f4xx_tim.h"         //!<  added for SD SPI purposes over 1289LCD myTouch One peripheral, timers   
#endif




#include  <stdio.h>                  //!<  needed here for defs of FILE, EOF
#include  <rt_misc.h>
#include  "platform_cfg.h"           //!<  hardware selection & abstraction defs
#include   "CanDefs.h"           //!<  Structure definition for CAN data parameters and prototypes



/************************************************************************\
*        LOCAL FUNCTION PROTOTYPES
\************************************************************************/

void  RCC_Configuration( void );     // Config Clock Settings                       
void  GPIO_Configuration( void );    // Config GPIOs and AFIO    

void  USART_Configuration( void );   // Config USART
void  LED_Configuration( void );     // Config LEDs
void  CAN_Configuration( void );     // Config CAN 
void  ADC_Configuration( void );      // Config ADC
void  SD_Configuration( void );      // Config SD
void  UI_Configuration( void );      // Config User interrupt button
void  Timer_Configuration(void);     // Configuration of Timer 


void  Periph_Init( void )
/**
*  ***********************************************************************
*  @brief  Initialize CPU/MCU and application relevant peripherals.
*  @retval none.
**************************************************************************
*/
{
   #if defined USE_MCBSTM32
        stm32_Init( );                 // 'classical' STM32 peripherals initialization
   #elif defined USE_MCBSTM32C
        LED_Configuration( );          // Config Clock Settings  
        USART_Configuration( );        // Config USART
        CAN_Configuration( );           // Config CAN
        ADC_Configuration( );          // Config ADC
        LCD_Init();  
   #elif defined USE_DISCOVERY
        Timer_Configuration();         // Config Timer   
        LED_Configuration( );          // Config Clock Settings                       
        USART_Configuration( );        // Config USART
        CAN_Configuration( );           // Config CAN 
        ADC_Configuration( );          // Config ADC
        SD_Configuration();             // Config SD
        UI_Configuration();             // Config User interrupt button
        LCD_Init();                     // Config LCD (uses SD1289)
        Initialize_LCDStructure();     //  Initialize the Structure holding the data to be displayed 
    #else
   // PRINT COMPILER ERROR MSG
  #endif
}          

/************************************************************************\
*        USE MCBSTM32C
\************************************************************************/

#ifdef      USE_MCBSTM32C
void LED_Configuration(void)
/************************************************************************
*  @brief  Configures the used status LEDs.
*  @param  data: user defined parameters from platform_cfg
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
*           and functions from:
*           stm32f10x_rcc.c and stm32f10x_gpio.c  
************************************************************************/
{ 
   GPIO_InitTypeDef GPIO_InitStructure;   // Define GPIO structure
   GPIO_StructInit(&GPIO_InitStructure);   // Initialize GPIO structure
  
  
   RCC_APB2PeriphClockCmd(GPIO_LED_CLK);// Enable GPIOC clock 
  
  // Configure GPIO Port E for LEDs
    // PE0...PE7 as analog input   
    // PE8...PE15 as push-pull output @ 50MHz 

   GPIO_InitStructure.GPIO_Pin = GPIO_LED_1 | GPIO_LED_2 | \
                                GPIO_LED_3 | GPIO_LED_4 | \
                                GPIO_LED_5 | GPIO_LED_6 | \
                                GPIO_LED_7 | GPIO_LED_8;

   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIO_LED, &GPIO_InitStructure);
} // END of LED_Configuration

/*******************************************************************************
*  @brief  Initializes the USART.
*  @param  data: user defined parameters from platform_cfg
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
*           and functions from:
*           stm32f10x_rcc.c, stm32f10x_gpio.c and  stm32f10x_usart.c
*******************************************************************************/
void USART_Configuration (void)
 {
  // defining confguration structures for USART 
   GPIO_InitTypeDef GPIO_InitStruct;                          // this is for the GPIO pins used as TX and RX configuration
   USART_InitTypeDef USART_InitStructure;                    // this is for the USART specific configuration
   
  // Enable GPIOC clock for USART
   RCC_APB2PeriphClockCmd(USARTx_GPIO_CLK, ENABLE);          // Enable clock for selected GPIO
   RCC_APB1PeriphClockCmd(USARTx_CLK, ENABLE);               // Enable clock for selected USART

  // USARTx GPIO config 
   GPIO_InitStructure.GPIO_Pin = USARTx_TxPin;                // TX Pin is configured
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;          // this defines the IO speed and has nothing to do with the baudrate!
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;            // the pin is configured as alternate function so the USART peripheral has access to them
   GPIO_Init(USARTx_GPIO, &GPIO_InitStructure);              // now all the values are passed to the GPIO_Init() function which sets the GPIO registers
   
   GPIO_InitStructure.GPIO_Pin = USARTx_RxPin;                // RX Pin is configured
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;      // Input floating
   GPIO_Init(USARTx_GPIO, &GPIO_InitStructure);              // now all the values are passed to the GPIO_Init() function which sets the GPIO registers
    
  // Set RX and TX pins AF 
   GPIO_PinRemapConfig(USARTx_AF_Remap, ENABLE);              // Remap alternate function for used GPIO pin as USARTx
 
  // USARTx Settings
   USART_StructInit(&USART_InitStructure);                    // Fills each ADC_InitStructure member with its default value
   USART_InitStructure.USART_BaudRate = USARTx_BDRate;                // Set Baud rate
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;          // Set Word length
   USART_InitStructure.USART_StopBits = USART_StopBits_1;            // Set number of stop bits
   USART_InitStructure.USART_Parity = USART_Parity_No ;            //Disable parity
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;      // Disable Flowcontrol
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;          // Set Tx and Rx modes on USART port
   USART_Init(USART_x, &USART_InitStructure);                // Initializes the USART peripheral according to the settings above
  
  
  USART_Cmd(USART_x, ENABLE);    // Enable the USARTx 

} // END of USART_Configuration

 void CAN_Configuration(void)
/************************************************************************
*  @brief  Configures the used status CAN bus.
*  @param  data: user defined parameters from platform_cfg
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
*           and functions from:
*           stm32f10x_rcc.c, stm32f10x_gpio.c and  stm32f10x_can.c
************************************************************************/
{ 
   CAN_InitTypeDef         CAN_InitStructure;       // Define CAN structure
   CAN_FilterInitTypeDef   CAN_FilterInitStructure; // Define CAN filter structure
   GPIO_InitTypeDef        GPIO_InitStructure;     // Define GPIO structure
   
   
   RCC_APB1PeriphClockCmd(CANx_CLK, ENABLE);                  // enable clock for CAN
   RCC_APB2PeriphClockCmd(CANx_GPIO_CLK, ENABLE);            // enable clock for GPIO used for CAN   
   RCC_APB2PeriphClockCmd(CANx_AF, ENABLE);              // Alternate Function clock enable 
   RCC_APB2PeriphClockCmd(CANx_REMAP1, ENABLE);            // Set CAN1 remap 


   // Configure CAN pin: RX 
   GPIO_InitStructure.GPIO_Pin = CANx_RxPin;              // Pin assignemet for Rx
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;            // GPIO mode assignement
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(CANx_GPIO, &GPIO_InitStructure);
   
  // Configure CAN pin: TX 
   GPIO_InitStructure.GPIO_Pin = CANx_TxPin;              // Pin assignemet for Rx
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;            // GPIO mode assignement
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(CANx_GPIO, &GPIO_InitStructure);   
   
   GPIO_PinRemapConfig(CANx_REMAP2, ENABLE);                  //GPIO_Remap2_CAN1 remaps PD0-1

   CAN_DeInit(CAN_x);                                         //Deinitializes the CAN peripheral registers to their 
                                                            //default reset values
   CAN_InitStructure.CAN_TTCM=DISABLE;                        // time-triggered communication mode = DISABLED
   CAN_InitStructure.CAN_ABOM=DISABLE;                        // automatic bus-off management mode = DISABLED                
   CAN_InitStructure.CAN_AWUM=DISABLE;                        // automatic wake-up mode = DISABLED
   CAN_InitStructure.CAN_NART=DISABLE;                       // non-automatic retransmission mode = DISABLED
   CAN_InitStructure.CAN_RFLM=DISABLE;                       // receive FIFO locked mode = DISABLED
   CAN_InitStructure.CAN_TXFP=DISABLE;                         // transmit FIFO priority = DISABLED
   CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;                    // normal CAN mode                                             
 
   CAN_InitStructure.CAN_SJW = CAN_SJW_4tq;                  //Initialize the CAN_SJW member          1-4
   CAN_InitStructure.CAN_BS1 = CAN_BS1_12tq;                  //Initialize the CAN_BS1 member        1-16
   CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;                  //Initialize the CAN_BS2 member        1-8
   CAN_InitStructure.CAN_Prescaler = 4;                      //Initialize the CAN_Prescaler member
   CAN_Init(CAN_x, &CAN_InitStructure);                        // Initializes the CAN peripheral according to the settings above
  

   CAN_FilterInitStructure.CAN_FilterNumber=0; 
   CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
   CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; 
   CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
   CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
   CAN_FilterInit(&CAN_FilterInitStructure);                // Initializes the CAN filter according to the settings above

} // END of CAN_Configuration

 void ADC_Configuration(void)
/************************************************************************
*  @brief  Configures the ADC used for throttle conversion.
*  @param  data: user defined parameters from platform_cfg
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*                  Constants defined in platform_cfg.h
*                  and functions from:
*                  stm32f10x_rcc.c and stm32f10x_adc.c
************************************************************************/
{
   GPIO_InitTypeDef GPIO_InitStructure;                      // Structure to initialize GPIO
   ADC_InitTypeDef  ADC_InitStructure;                        // Structure to define ADC functionallity
    
  // Enable ADC1 Clock 
   RCC_APB2PeriphClockCmd(ADCx_CLK , ENABLE);              // enable clock for ADC
   RCC_APB2PeriphClockCmd(ADCx_GPIO_CLK, ENABLE);            // enable clock for GPIO used 
  
  // ADC config for throttle
   GPIO_InitStructure.GPIO_Pin = ADCx_Pin;                    // Select pin at specific GPIO at which the ADC is used
   GPIO_InitStructure.GPIO_Mode = ADCx_AF;                    // the pins are configured as alternate function 
   GPIO_Init(ADCx_GPIO, &GPIO_InitStructure);                     // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

   ADC_DeInit();                                               // Deinitializes the ADC peripheral registers to their
                                                            //default reset values
   ADC_StructInit(&ADC_InitStructure);                        // Fills each ADC_InitStructure member with its default value.
   ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
   ADC_InitStructure.ADC_ScanConvMode = ENABLE;           // ENABLE ScanConversion Mode
   ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;       // Scan on Demand
   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  // no external triggering
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;    // Data align to the right
   ADC_InitStructure.ADC_NbrOfChannel = 1;
   ADC_Init(ADC_x, &ADC_InitStructure);                        // Initializes the ADC peripheral according to the settings above

  // ADC1 regular channel1 configuration 
  ADC_RegularChannelConfig(ADC_x, ADC_Channel_x, 1, ADC_SampleTime_x);  //Configures for the selected ADC regular channel

  
  ADC_Cmd(ADC_x, ENABLE);// Enable ADC1 

  
  ADC_ResetCalibration(ADC_x);// Enable ADC1 reset calibaration register 

  
  while(ADC_GetResetCalibrationStatus(ADC_x));// Check the end of ADC1 reset calibration register 

  
  ADC_StartCalibration(ADC_x); // Start ADC1 calibaration 
  
  while(ADC_GetCalibrationStatus(ADC_x));// Check the end of ADC1 calibration 
} // END of ADC_Configuration



/************************************************************************\
*        USE DISCOVERY - STM32F4
\************************************************************************/
#elif defined USE_DISCOVERY

void LED_Configuration(void)
/************************************************************************
*  @brief  Configures the used status LEDs.
*  @param  data: user defined parameters from platform_cfg
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*                  Constants defined in platform_cfg.h
*                  and functions from:
*                  stm32f4xx_rcc.c and stm32f4_discovery.c
************************************************************************/
{ 
   // Enable GPIOD clock 
   RCC_AHB1PeriphClockCmd(GPIO_LED_CLK, ENABLE);
   // LED GPIO config   
   STM_EVAL_LEDInit(STATUS_LED);  
   STM_EVAL_LEDInit(UI_LED);// initialize LED port
   STM_EVAL_LEDOff(UI_LED);//toggle LED state
                         
} // END of LED_Configuration


void USART_Configuration (void)
/*******************************************************************************
*  @brief  Initializes the USART.
*  @param  data: user defined parameters from platform_cfg
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*                  Constants defined in platform_cfg.h
*                  and functions from:
*                  stm32f4xx_gpio.h and stm32f4xx_usart.h
*******************************************************************************/
 {
  // defining confguration structures for USART 
   GPIO_InitTypeDef GPIO_InitStruct;       // this is for the GPIO pins used as TX and RX configuration
   USART_InitTypeDef USART_InitStructure;  // this is for the USART specific configuration
   
  // Enable GPIOC clock for USART
   RCC_AHB1PeriphClockCmd(USARTx_GPIO_CLK, ENABLE); // Enable clock for selected GPIO
   RCC_APB2PeriphClockCmd(USARTx_CLK, ENABLE);      // Enable clock for selected USART
   
  // USARTx GPIO config 
   GPIO_InitStruct.GPIO_Pin = USARTx_TxPin;           // TX Pin is configured
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;          // the pin is configured as alternate function so the USART peripheral has access to them
   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;     // this defines the IO speed and has nothing to do with the baudrate!
   GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;        // this defines the output type as push pull mode (as opposed to open drain)
   GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;          // this activates the pullup resistor on the IO pins
   GPIO_Init(USARTx_GPIO, &GPIO_InitStruct);          // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

   GPIO_InitStruct.GPIO_Pin = USARTx_RxPin;                 // RX Pin is configured
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;               // the pin is configured as alternate function so the USART peripheral has access to them
   GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;              // this defines the output type as push pull mode (as opposed to open drain)
   GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;            // this activates the pullup resistor on the IO pins
   GPIO_Init(USARTx_GPIO, &GPIO_InitStruct);              // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

  // Set RX and TX pins AF 
   GPIO_PinAFConfig(USARTx_GPIO, USARTx_GPIO_SRC_TX, USARTx_AF); // Remap alternate function for used GPIO pin as USARTx
   GPIO_PinAFConfig(USARTx_GPIO, USARTx_GPIO_SRC_RX, USARTx_AF); // Remap alternate function for used GPIO pin as USARTx
   
     
  // USARTx Settings
   USART_StructInit(&USART_InitStructure);                  // Fills each ADC_InitStructure member with its default value
   USART_InitStructure.USART_BaudRate = USARTx_BDRate;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No ;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_Init(USART_x, &USART_InitStructure);                // Initializes the USART peripheral according to the settings above
  
  // Enable the USARTx 
  USART_Cmd(USART_x, ENABLE);

} // END of USART_Configuration
 
 void CAN_Configuration(void)
/************************************************************************
*  @brief  Configures the used status CAN bus.
*  @param  data: user defined parameters from platform_cfg
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
*           and functions from:
*           stm32f4xx_gpio.h and stm32f4xx_can.h
************************************************************************/
{ 
   // Initialization of CAN1
   CAN_InitTypeDef        CAN_InitStructure;                                // CAN init structure
   CAN_FilterInitTypeDef CAN_FilterInitStructure;                      // Can Filter structure definition
   
   GPIO_InitTypeDef GPIO_InitStructureCAN_TX;                           // Structure definition for CAN1 GPIO Pins 
   GPIO_InitTypeDef GPIO_InitStructureCAN_RX;                           // Structure definition for CAN1 GPIO Pins 
 
   RCC_APB1PeriphClockCmd(CANx_CLK, ENABLE);                          // enable clock for CAN
   RCC_AHB1PeriphClockCmd(CANx_GPIO_CLK, ENABLE);                      // enable clock for GPIO used for CAN
    
   
   // GPIO CAN Tx Config
   GPIO_InitStructureCAN_TX.GPIO_Pin        = CANx_TxPin;             // Pin assignemet for Tx
   GPIO_InitStructureCAN_TX.GPIO_Mode       = GPIO_Mode_AF ;           // GPIO mode assignement
   GPIO_InitStructureCAN_TX.GPIO_OType      = GPIO_OType_PP;            // Configring push pull mode
   GPIO_InitStructureCAN_TX.GPIO_PuPd       = GPIO_PuPd_UP;            // this activates the pullup resistors on the IO pins
   GPIO_InitStructureCAN_TX.GPIO_Speed      = GPIO_Speed_50MHz;
   GPIO_Init(CANx_GPIO, &GPIO_InitStructureCAN_TX);
   GPIO_PinAFConfig(CANx_GPIO, CANx_GPIO_SRC_TX, CANx_AF);
   
   
   // GPIO CAN Rx Config
   GPIO_InitStructureCAN_RX.GPIO_Pin        = CANx_RxPin;               // Pin assignemet for Rx
   GPIO_InitStructureCAN_RX.GPIO_Mode       = GPIO_Mode_AF ;             // GPIO mode assignement
   GPIO_InitStructureCAN_RX.GPIO_OType      = GPIO_OType_PP;            // Configring push pull mode
   GPIO_InitStructureCAN_RX.GPIO_PuPd       = GPIO_PuPd_UP;              // this activates the pullup resistors on the IO pins
   GPIO_InitStructureCAN_RX.GPIO_Speed      = GPIO_Speed_50MHz;
   GPIO_Init(CANx_GPIO, &GPIO_InitStructureCAN_RX);
   GPIO_PinAFConfig(CANx_GPIO, CANx_GPIO_SRC_RX, CANx_AF);
      
   CAN_DeInit(CAN_x);                              //Deinitializes the CAN peripheral registers to their 
                                                  //default reset values
   CAN_StructInit(&CAN_InitStructure);  // Fills each CAN_InitStruct member with its default value
   CAN_InitStructure.CAN_TTCM = DISABLE;// time-triggered communication mode = DISABLED
   CAN_InitStructure.CAN_ABOM = DISABLE;// automatic bus-off management mode = DISABLED
   CAN_InitStructure.CAN_AWUM = DISABLE;// automatic wake-up mode = DISABLED
   CAN_InitStructure.CAN_NART = DISABLE;// non-automatic retransmission mode = DISABLED
   CAN_InitStructure.CAN_RFLM = DISABLE;// receive FIFO locked mode = DISABLED
   CAN_InitStructure.CAN_TXFP = DISABLE;// transmit FIFO priority = DISABLED
   CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;// normal CAN mode
   CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;     // synchronization jump width = 1
   CAN_InitStructure.CAN_BS1 = CAN_BS1_14tq;    //14
   CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;     //6
   CAN_InitStructure.CAN_Prescaler = 4;         //4 -> baudrate 500 kbps
   CAN_Init(CAN_x, &CAN_InitStructure);         // Initializes the CAN peripheral according to the settings above
   
   
   
   /// specified parameters for the CAN message filter
   CAN_FilterInitStructure.CAN_FilterNumber=0; 
   CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
   CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; 
   CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
   CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
   CAN_FilterInit(&CAN_FilterInitStructure);      // Initializes the CAN filter according to the settings above

} // END of CAN_Configuration

 void ADC_Configuration(void)
/************************************************************************
*  @brief  Configures the ADC used for throttle conversion.
*  @param  data: user defined parameters from platform_cfg
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
*           and functions from:
*           stm32f4xx_gpio.h and stm32f4xx_adc.h
************************************************************************/
{
   GPIO_InitTypeDef GPIO_InitStruct;                      // Structure to initialize GPIO
   ADC_InitTypeDef ADC_InitStructure;                    // Structure to define ADC functionallity
  
    // Enable ADC1 Clock 
   RCC_APB2PeriphClockCmd(ADCx_CLK, ENABLE);      // enable clock for ADC
   RCC_AHB1PeriphClockCmd(ADCx_GPIO_CLK, ENABLE); // enable clock for GPIO used 
  
  // ADC config for throttle
   GPIO_InitStruct.GPIO_Pin = ADCx_Pin;           // Select pin at sepcific GPIO at which the ADC isw used
   GPIO_InitStruct.GPIO_Mode = ADCx_AF;           // the pins are configured as alternate function 
   GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;  // No Pull
   GPIO_Init(ADCx_GPIO, &GPIO_InitStruct);        // now all the values are passed to the GPIO_Init() function which sets the GPIO registers
  
   ADC_DeInit();                                  //Deinitializes the ADC peripheral registers to their
                                                        //default reset values
   ADC_StructInit(&ADC_InitStructure);            // Fills each ADC_InitStructure member with its default value.
   ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                    //ADC resolution 12Bit
   ADC_InitStructure.ADC_ScanConvMode = ENABLE;                              // ENABLE ScanConversion Mode
   ADC_InitStructure.ADC_ExternalTrigConvEdge= ADC_ExternalTrigConvEdge_None;// no external triggering
   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                    // Data align to the right
   ADC_InitStructure.ADC_NbrOfConversion = 1;                                // Number of Conversions = 1
   ADC_Init(ADC_x,&ADC_InitStructure);                                       // Initializes the ADC peripheral according to the settings above

   ADC_Cmd(ADC_x, ENABLE);
   ADC_RegularChannelConfig(ADC_x, ADC_Channel_x, 1, ADC_SampleTime_x);      //Configures for the selected ADC regular channel

} // END of ADC_Configuration



void SD_Configuration(void)
  /************************************************************************
*  @brief  Configures the SD card used for data storage
*  @param  data: user defined parameters from platform_cfg
*  @retval  none
* Additionnal   : Uses functions found within ff.c and definitions found within platform_cfg.h
************************************************************************/ 
{

  FATFS FatFs; // Fatfs object
  FIL fil;     // File object
  FRESULT res; // Variable for error handling

  GPIO_InitTypeDef GPIO_InitStruct;
  SPI_InitTypeDef SPI_InitStruct;

  /*CS Init*/
  RCC_AHB1PeriphClockCmd(SPI_CS_CLK, ENABLE);  // CHIP SELECT PIN, defined in defines.h GPIOB - pin 5
  GPIO_InitStruct.GPIO_Pin = SPI_CS_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(SPI_CS_PORT, &GPIO_InitStruct);
   
  
  //Common settings for all pins
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
 
  //Enable clock for GPIOA
  RCC_AHB1PeriphClockCmd(GPIO_SPI_CLK, ENABLE);
  //Pinspack nr. 1        SCK          MISO         MOSI
  GPIO_InitStruct.GPIO_Pin = GPIO_SPI_SCK_pin | GPIO_SPI_MISO_pin | GPIO_SPI_MOSI_pin;
  GPIO_Init(GPIO_SDx, &GPIO_InitStruct);

  GPIO_PinAFConfig(GPIO_SDx, GPIO_SPI_SCK, GPIO_AF_SPIx);
  GPIO_PinAFConfig(GPIO_SDx, GPIO_SPI_MISO, GPIO_AF_SPIx);
  GPIO_PinAFConfig(GPIO_SDx, GPIO_SPI_MOSI, GPIO_AF_SPIx);
  RCC_APB2PeriphClockCmd(SPI_CLK, ENABLE);

  SPI_StructInit(&SPI_InitStruct);
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; //BAUDRATE
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;                  //Specify datasize   
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;                 //Specify which bit is first
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;                      //Specify mode of operation, clock polarity and clock phase
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;                   //Specify mode of operation, clock polarity and clock phase
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_Init(SPI_SDx, &SPI_InitStruct);

  SPI_Cmd(SPI_SDx, ENABLE);

// Try to mount the drive and write header 

  if (f_mount(&FatFs, "", 1) == FR_OK) {
    res = f_open(&fil, SD_CARD_FILE_NAME, FA_OPEN_ALWAYS|FA_READ|FA_WRITE) ;   // Open file 'name', sets up GPIOS, Pins and Clocks for SPI, Try to open  FA_OPEN_ALWAYS
    if ((res) == FR_OK) {           // test if file is functioning correctly
      f_lseek(&fil, f_size(&fil));// replace write pointer to end of file in order to append new data to already existing data   
      if (f_size(&fil) == 0) {  //if empty create header
              f_puts("Battery Voltage;Battery Current;Motor RPM;Odometer;Temperature Controller;Battery Temperature;Motor Temperature;Warning Bits;Error Bits;Battery Total Capacity;Current;Total Voltage;Highest Temperature;Notification;Last Error Address;Internal Error Code;Lowest Voltage;Lowest Voltage Address;HighestVoltage;Highest Voltage Address;Lowest Temperature\n", &fil);
          }
          f_close(&fil);              //Close file, don't forget this!
      }  
    f_mount(0, "", 1);              //Unmount drive, don't forget this!
    }//End of mount and initialization
} // END of SD_Configuration

void UI_Configuration(void)
/************************************************************************
*  @brief  Configures User Interrupt GPIO and the Interrupt handler.
*  @param  data: user defined parameters from platform_cfg
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
************************************************************************/
{ 
  GPIO_InitTypeDef GPIO_InitStruct;    // Define GPIO structure
    EXTI_InitTypeDef EXTI_InitStruct;    // Define External interrupt structure 
    NVIC_InitTypeDef NVIC_InitStruct;    // Define NVIC structure
    
    // Enable clock for GPIOB
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    // Enable clock for SYSCFG 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    // Set pin as input 
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;      //Input mode
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // Push pull
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;      // Pin0
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   //GPIO_PuPd_UP
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  //
    GPIO_Init(GPIOA, &GPIO_InitStruct);        // Initialize Structure
    
    // Connect EXTI Line0 to PA0 pin 
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
    
    // PA0 is connected to EXTI_Line0
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    // Enable interrupt 
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    // Interrupt mode 
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    // Triggers on rising and falling edge 
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    // Add to EXTI 
    EXTI_Init(&EXTI_InitStruct);
 
    // Add IRQ vector to NVIC 
    // PA0 is connected to EXTI_Line0, EXTI0_IRQn 
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    // Set priority 
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
    //Set sub priority 
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    // Enable interrupt 
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    // Add to NVIC
    NVIC_Init(&NVIC_InitStruct);
  
    // BUTTON_USER and BUTTON_MODE_EXTI are defined within respective structures
    //STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);         
  //Enable clock for GPOIA used as User interrupt signal (set to HIGH if button pressed)
  RCC_AHB1PeriphClockCmd(GPIO_UI_CLK, ENABLE);
  //Pin PA.2
  GPIO_InitStruct.GPIO_Pin = UIx_GPIO_pin;
  //Mode output
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  //Output type push-pull
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  //Without pull resistors
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  //50MHz pin speed
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

  //Initialize pins on GPIOA port
  GPIO_Init(UIx_GPIO, &GPIO_InitStruct);
  
  // set output to LOW initally
  GPIO_ResetBits(GPIOA, GPIO_Pin_2);         

} // END of UI_Configuration

void  Timer_Configuration(void)
/************************************************************************
*  @brief   Configures a timer to be used for various purposes.
*  @param   data: user defined parameters from platform_cfg
*  @retval none
* Additionnal   : Uses structure definitions found in stm32f4xx_tim.h
************************************************************************/
{
  TIM_TimeBaseInitTypeDef timerInitStructure;          // Create timer structure
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    // Set RCC clock for TIMER entity
    
    timerInitStructure.TIM_Prescaler = 40000;          // Set prescaler
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  // Set couting direction >
    timerInitStructure.TIM_Period = 5000;            // Set timer tick period
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;    // Set clock divions, from fundemental clock
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);         // Initialize timer structure on port TIM2  
  TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);      // Define mode
}

#endif
