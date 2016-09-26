/**
*  ***********************************************************************
*  @file    Peripheral_init.c
*  @author  Aravind Krishnan, Madhan Kumar Gopinath
*  @version V2.00
*  @brief   Initialization of all peripherals.           
*          
*  Changes:  02.09.2015 Madhan and Aravind: Included RCC_Configuration() 
*                       function which was missing. USART_Configuration()
*                       and CAN_Configuration() functions were modified from
*                       a previous working code
*
*  @date    27/09/2015
**************************************************************************
*/


#ifdef USE_MCBSTM32
#include  "stm32f10x_conf.h"        //!< STM32F10x Configuration File
#include  "STM32_Init.h"            //!< STM32 initialization package  
#elif defined USE_MCBSTM32C
#include  "stm32f10x_conf.h"        //!< STM32F10x Configuration File
#include  "STM32_Init.h"            //!< STM32 initialization package     
#elif defined USE_DISCOVERY
#include "stm32f4_discovery.h"      //!< Functions specific to discovery board
#include "stm32f4xx_conf.h"         //!< Periph. library configuration file
#else
#error: "Illegal target board selection!"
#endif

#include  <stdio.h>                 //!< needed here for defs of FILE, EOF
#include  <rt_misc.h>
#include  "platform_cfg.h"          //!< hardware selection & abstraction defs
#include  "LcdDefs.h"               //!< LCD periph. library
#include  "CanDefs.h"               //!< Structure definition for CAN data parameters and prototypes
#include  "SDlib.h"                 //!< added for SD SPI purposes over 1289LCD myTouch One peripheral, SPI protocols and file system


/************************************************************************\
*        LOCAL FUNCTION PROTOTYPES
\************************************************************************/

void  RCC_Configuration( void );     // Config Clock Settings                       
void  GPIO_Configuration( void );    // Config GPIOs and AFIO    
void  USART_Configuration( void );   // Config USART
void  LED_Configuration( void );     // Config LEDs
void  CAN_Configuration( void );     // Config CAN 
void  ADC_Configuration( void );     // Config ADC
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
  RCC_Configuration();   
  Timer_Configuration();         // Config Timer
  LED_Configuration( );          // Config Clock Settings  
  USART_Configuration( );        // Config USART
  CAN_Configuration( );          // Config CAN
  ADC_Configuration( );          // Config ADC
  UI_Configuration();            // Config User interrupt button
  SD_Configuration();            // Config SD
  LCD_Init();                    // Config LCD
  
#elif defined USE_DISCOVERY
  Timer_Configuration();         // Config Timer   
  LED_Configuration( );          // Config Clock Settings                       
  USART_Configuration( );        // Config USART
  CAN_Configuration( );          // Config CAN 
  ADC_Configuration( );          // Config ADC
  SD_Configuration();            // Config SD
  UI_Configuration();            // Config User interrupt button
  LCD_Init();                    // Config LCD (uses SD1289)
  Initialize_LCDStructure();     // Initialize the Structure holding the data to be displayed 
#else
#error: "Illegal target board selection!"
#endif
}          

/************************************************************************\
********************         USE MCBSTM32C        ************************
\************************************************************************/

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
  RCC_APB2PeriphClockCmd(USARTx_GPIO_CLK |GPIO_LED_CLK | RCC_APB2Periph_AFIO, ENABLE);


  /* Enable USARTx Clock */
  RCC_APB1PeriphClockCmd(USARTx_CLK, ENABLE); 


  /* Enable the I2C1 APB1 clock */
  //RCC_APB1PeriphClockCmd(CODEC_I2C_CLK, ENABLE);

}

void  Timer_Configuration(void)
  /************************************************************************
  *  @brief   Configures a timer to be used for various purposes.
  *  @param   data: user defined parameters from platform_cfg
  *  @retval  none
  * Additionnal   : Uses structure definitions found in stm32f4xx_tim.h
  ************************************************************************/
{
  TIM_TimeBaseInitTypeDef timerInitStructure;               // Create timer structure
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);      // Set RCC clock for TIMER entity

  timerInitStructure.TIM_Prescaler = 40000;                 // Set prescaler
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  // Set couting direction >
  timerInitStructure.TIM_Period = 5000;                     // Set timer tick period
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;      // Set clock divions, from fundemental clock
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timerInitStructure);              // Initialize timer structure on port TIM2  
  TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);          // Define mode
}

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
  GPIO_InitTypeDef GPIO_InitStructure;           // Define GPIO structure
  GPIO_StructInit(&GPIO_InitStructure);          // Initialize GPIO structure

  RCC_APB2PeriphClockCmd(GPIO_LED_CLK, ENABLE);  // Enable GPIOE clock 

  GPIO_InitStructure.GPIO_Pin = GPIO_LED_1 | GPIO_LED_2 | \
    GPIO_LED_3 | GPIO_LED_4 | \
    GPIO_LED_5 | GPIO_LED_6 | \
    GPIO_LED_7 | GPIO_LED_8;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     // PE8...PE15 as push-pull output @ 50MHz
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_Init(GPIO_LED, &GPIO_InitStructure);             // Configure GPIO Port E for LEDs
}                                                       // END of LED_Configuration


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
  GPIO_InitTypeDef GPIO_InitStruct;                         // this is for the GPIO pins used as TX and RX configuration
  USART_InitTypeDef USART_InitStructure;                    // this is for the USART specific configuration

  // Enable GPIOC clock for USART
  RCC_APB2PeriphClockCmd(USARTx_GPIO_CLK | USARTx_AF_CLK, ENABLE);   // Enable clock for selected GPIO
  RCC_APB1PeriphClockCmd(USARTx_CLK, ENABLE);                        // Enable clock for selected USART

  // USARTx GPIO config 
  GPIO_InitStruct.GPIO_Pin = USARTx_TxPin;               // TX Pin is configured
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;         // this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;           // the pin is configured as alternate function so the USART peripheral has access to them
  GPIO_Init(USARTx_GPIO, &GPIO_InitStruct);              // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

  GPIO_InitStruct.GPIO_Pin = USARTx_RxPin;               // RX Pin is configured
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;     // Input floating
  GPIO_Init(USARTx_GPIO, &GPIO_InitStruct);              // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

  // Set RX and TX pins AF 
  GPIO_PinRemapConfig(USARTx_AF_Remap, ENABLE);              // Remap alternate function for used GPIO pin as USARTx

  // USARTx Settings
  USART_StructInit(&USART_InitStructure);                    // Fills each ADC_InitStructure member with its default value
  USART_InitStructure.USART_BaudRate = USARTx_BDRate;               // Set Baud rate
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;       // Set Word length
  USART_InitStructure.USART_StopBits = USART_StopBits_1;            // Set number of stop bits
  USART_InitStructure.USART_Parity = USART_Parity_No ;              //Disable parity
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Disable Flowcontrol
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // Set Tx and Rx modes on USART port
  USART_Init(USART_x, &USART_InitStructure);                                      // Initializes the USART peripheral according to the settings above


  USART_Cmd(USART_x, ENABLE);   // Enable the USARTx 

}                               // END of USART_Configuration

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

  CAN_InitTypeDef CAN_InitStruct;
  CAN_FilterInitTypeDef   Can_filter;                  // Define CAN filter structure
  GPIO_InitTypeDef        GPIO_InitStructure;          // Define GPIO structure


  /*Initialization of CAN1*/
  RCC_APB1PeriphClockCmd(CANx_CLK, ENABLE);            // enable clock for CAN
  //Note: MCBSTM32C CAN1 uses PD0 and PD1

  RCC_APB2PeriphClockCmd(CANx_AF_CLK, ENABLE);         // Alternate Function clock enable
  GPIO_PinRemapConfig(CANx_REMAP2, ENABLE);            // GPIO_Remap2_CAN1 remaps PD0-1  
  RCC_APB2PeriphClockCmd(CANx_GPIO_CLK, ENABLE);       // enable clock for GPIO used for CAN 


  // Configure CAN pin: RX 
  GPIO_InitStructure.GPIO_Pin = CANx_RxPin;            // Pin assignemet for Rx
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;        // GPIO mode assignement
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CANx_GPIO, &GPIO_InitStructure);

  // Configure CAN pin: TX 
  GPIO_InitStructure.GPIO_Pin = CANx_TxPin;            // Pin assignemet for Rx
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      // GPIO mode assignement
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CANx_GPIO, &GPIO_InitStructure); 

  CAN_DeInit(CAN_x);                             // Deinitializes the CAN peripheral registers to their 
  //default reset values
  CAN_StructInit(&CAN_InitStruct);               // Fills each CAN_InitStruct member with its default value.
  CAN_InitStruct.CAN_SJW = CAN_SJW_4tq;          // Initialize the CAN_SJW member          1-4
  CAN_InitStruct.CAN_BS1 = CAN_BS1_12tq;         // Initialize the CAN_BS1 member        1-16
  CAN_InitStruct.CAN_BS2 = CAN_BS2_5tq;          // Initialize the CAN_BS2 member        1-8
  CAN_InitStruct.CAN_Prescaler = 4;              // Initialize the CAN_Prescaler member
  /*Clk=72MHz 
  HCLK = SYSCLK 
  PCLK2 = HCLK 
  PCLK1 = HCLK/2 
  BaudRate=36/Prescaler/(sjw+bs1+bs2)  = 9/(1+12+5) = 500K
  sample point = (1+bs1)/(1+bs1+bs2)  = 13/18 = 72%
  */
  CAN_Init(CAN_x, &CAN_InitStruct);              // Initializes the CAN peripheral according to the 
  //specified parameters in the CAN_InitStruct
  Can_filter.CAN_FilterNumber=0; 
  Can_filter.CAN_FilterMode=CAN_FilterMode_IdMask; 
  Can_filter.CAN_FilterScale=CAN_FilterScale_16bit; 
  Can_filter.CAN_FilterFIFOAssignment=0;
  Can_filter.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&Can_filter);
}                                                // END of CAN_Configuration

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
  GPIO_InitTypeDef GPIO_InitStructure;                 // Structure to initialize GPIO
  ADC_InitTypeDef  ADC_InitStructure;                  // Structure to define ADC functionallity

  // Enable ADC1 Clock 
  RCC_APB2PeriphClockCmd(ADCx_CLK , ENABLE);           // enable clock for ADC
  RCC_APB2PeriphClockCmd(ADCx_GPIO_CLK, ENABLE);       // enable clock for GPIO used 

  // ADC config for throttle
  GPIO_InitStructure.GPIO_Pin = ADCx_Pin;              // Select pin at specific GPIO at which the ADC is used
  GPIO_InitStructure.GPIO_Mode = ADCx_AF;              // the pins are configured as alternate function 
  GPIO_Init(ADCx_GPIO, &GPIO_InitStructure);           // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

  ADC_DeInit(ADC_x);                                   // Deinitializes the ADC peripheral registers to their
  //default reset values
  ADC_StructInit(&ADC_InitStructure);                  // Fills each ADC_InitStructure member with its default value.
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;         // ENABLE ScanConversion Mode
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // Scan on Demand
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  // no external triggering
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               // Data align to the right
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC_x, &ADC_InitStructure);                 // Initializes the ADC peripheral according to the settings above

  // ADC1 regular channel1 configuration
  ADC_RegularChannelConfig(ADC_x, ADC_Channel_x, 1, ADC_SampleTime_x);  // Configures for the selected ADC regular channel

  ADC_Cmd(ADC_x, ENABLE);                              // Enable ADC1 

  ADC_ResetCalibration(ADC_x);                         // Enable ADC1 reset calibaration register 

  while(ADC_GetResetCalibrationStatus(ADC_x));         // Check the end of ADC1 reset calibration register 

  ADC_StartCalibration(ADC_x);                         // Start ADC1 calibaration 

  while(ADC_GetCalibrationStatus(ADC_x));              // Check the end of ADC1 calibration 

} // END of ADC_Configuration


void UI_Configuration(void)
  /************************************************************************
  *  @brief  Configures User Interrupt GPIO and the Interrupt handler.
  *  @param  data: user defined parameters from platform_cfg
  *  @retval  none
  * Additionnal    : Conditionnal compile flags, Global variables and 
  *                  Constants defined in platform_cfg.h
  ************************************************************************/
{ 
  GPIO_InitTypeDef GPIO_InitStruct;    // Define GPIO structure
  EXTI_InitTypeDef EXTI_InitStruct;    // Define External interrupt structure 
  NVIC_InitTypeDef NVIC_InitStruct;    // Define NVIC structure

  // Enable clock for GPIOB
  RCC_APB2PeriphClockCmd(GPIO_UI_BTN_CLK, ENABLE);

  // Set pin as input 
  GPIO_InitStruct.GPIO_Pin = UI_BTN;                                // Pin B.7
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;                // Input mode
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                    // 50Mhz
  GPIO_Init(GPIO_UI_BTN, &GPIO_InitStruct);                         // Initialize Structure


  GPIO_EXTILineConfig(UI_BTN_EXTI_PORTSRC, UI_BTN_EXTI_PINSRC);     // Connect EXTI to PB7 pin

  EXTI_InitStruct.EXTI_Line = UI_BTN_EXTI_LINE;                 // PB7 is connected to EXTI_Line7
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;              // Interrupt mode 
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;           // Triggers on rising edge 
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;                        // Enable interrupt
  EXTI_Init(&EXTI_InitStruct);                                  // Add to EXTI

  // Add IRQ vector to NVIC 
  NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;               // PB7 is connected to EXTI_Line7, EXTI9_5_IRQn 
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;     // Set priority 
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;            // Set sub priority 
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;                  // Enable interrupt 
  NVIC_Init(&NVIC_InitStruct);                                  // Add to NVIC


  // Enable clock for GPOIB used as User interrupt signal (set to HIGH if button pressed)
  RCC_APB2PeriphClockCmd(GPIO_UI_CLK, ENABLE);

  GPIO_InitStruct.GPIO_Pin = UIx_GPIO_pin;                      // Pin PB.8
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;                 // Mode output
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                // 50MHz pin speed

  //Initialize pins on GPIOA port
  GPIO_Init(UIx_GPIO, &GPIO_InitStruct);

  // set output to LOW initally
  GPIO_ResetBits(UIx_GPIO, UIx_GPIO_pin);         

}                                                               // END of UI_Configuration

void SD_Configuration(void)
  /************************************************************************
  *  @brief  Configures the SD card used for data storage
  *  @param  data: user defined parameters from platform_cfg
  *  @retval  none
  * Additionnal   : Uses definitions found within platform_cfg.h
  ************************************************************************/ 
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable SPI1 and GPIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | SPI_CS_CLK, ENABLE);


  /*CS Init*/
  GPIO_InitStructure.GPIO_Pin = SPI_CS_PIN;                             // SPI_CS pin  =  PA.4
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                      // Set to output mode type PushPull
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                     // GPIO speed 50Mhz
  GPIO_Init(SPI_CS_PORT, &GPIO_InitStructure);                          // Initializes the GPIO


  /* Configure SPIx pins: SCK and MOSI with default alternate function (not remapped) push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_SPI_SCK_pin | GPIO_SPI_MOSI_pin;   // SPI_CLK pin  = PA.5 , SPI_MOSI pin = PA.7
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                     // GPIO speed 50Mhz
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                       // Alternate function
  GPIO_Init(GPIO_SDx, &GPIO_InitStructure);                             // Initializes the GPIO

  /* Configure MISO as Input with internal pull-up */
  GPIO_InitStructure.GPIO_Pin = GPIO_SPI_MISO_pin;                      // SPI_MISO pin = PA.6
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                     // GPIO speed 50Mhz
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                         // Set as Input (with Pull Up)
  GPIO_Init(GPIO_SDx, &GPIO_InitStructure);


  /* SPI1 configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;    // Fullduplex direction
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                         // Set MCU to be the SPI Master
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                     // Specify datasize
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                            // Specify mode of operation, clock polarity and clock phase
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  // 72000kHz/128=281kHz < 400Hz BAUDRATE
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                    // Specify which bit is first
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);                                   // Initializes the SPI peripheral according to the settings above

  /* Enables or disables the CRC value calculation of the transfered bytes */
  SPI_CalculateCRC(SPI_SDx, DISABLE);

  /* Enable SPIx */
  SPI_Cmd(SPI_SDx, ENABLE);

} // END of SD_Configuration

/************************************************************************\
********************         USE DISCOVERY        ************************
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
  RCC_AHB1PeriphClockCmd(GPIO_LED_CLK, ENABLE);       // Enable GPIOD clock 
  // LED GPIO config   
  STM_EVAL_LEDInit(STATUS_LED);  
  STM_EVAL_LEDInit(UI_LED);                           // initialize LED port
  STM_EVAL_LEDOff(UI_LED);                            // toggle LED state
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
  GPIO_InitTypeDef GPIO_InitStruct;                // this is for the GPIO pins used as TX and RX configuration
  USART_InitTypeDef USART_InitStructure;           // this is for the USART specific configuration

  // Enable GPIOC clock for USART
  RCC_AHB1PeriphClockCmd(USARTx_GPIO_CLK, ENABLE); // Enable clock for selected GPIO
  RCC_APB2PeriphClockCmd(USARTx_CLK, ENABLE);      // Enable clock for selected USART

  // USARTx GPIO config 
  GPIO_InitStruct.GPIO_Pin = USARTx_TxPin;         // TX Pin is configured
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        // the pin is configured as alternate function so the USART peripheral has access to them
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;   // this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // this defines the output type as push pull mode (as opposed to open drain)
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;        // this activates the pullup resistor on the IO pins
  GPIO_Init(USARTx_GPIO, &GPIO_InitStruct);        // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

  GPIO_InitStruct.GPIO_Pin = USARTx_RxPin;         // RX Pin is configured
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        // the pin is configured as alternate function so the USART peripheral has access to them
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // this defines the output type as push pull mode (as opposed to open drain)
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;    // this activates the pullup resistor on the IO pins
  GPIO_Init(USARTx_GPIO, &GPIO_InitStruct);        // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

  // Set RX and TX pins AF 
  GPIO_PinAFConfig(USARTx_GPIO, USARTx_GPIO_SRC_TX, USARTx_AF); // Remap alternate function for used GPIO pin as USARTx
  GPIO_PinAFConfig(USARTx_GPIO, USARTx_GPIO_SRC_RX, USARTx_AF); // Remap alternate function for used GPIO pin as USARTx


  // USARTx Settings
  USART_StructInit(&USART_InitStructure);          // Fills each ADC_InitStructure member with its default value
  USART_InitStructure.USART_BaudRate = USARTx_BDRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART_x, &USART_InitStructure);       // Initializes the USART peripheral according to the settings above

  // Enable the USARTx 
  USART_Cmd(USART_x, ENABLE);

}                                                  // END of USART_Configuration

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
  CAN_InitTypeDef        CAN_InitStructure;                      // CAN init structure
  CAN_FilterInitTypeDef CAN_FilterInitStructure;                 // Can Filter structure definition

  GPIO_InitTypeDef GPIO_InitStructureCAN_TX;                     // Structure definition for CAN1 GPIO Pins 
  GPIO_InitTypeDef GPIO_InitStructureCAN_RX;                     // Structure definition for CAN1 GPIO Pins 

  RCC_APB1PeriphClockCmd(CANx_CLK, ENABLE);                      // enable clock for CAN
  RCC_AHB1PeriphClockCmd(CANx_GPIO_CLK, ENABLE);                 // enable clock for GPIO used for CAN


  // GPIO CAN Tx Config
  GPIO_InitStructureCAN_TX.GPIO_Pin        = CANx_TxPin;         // Pin assignemet for Tx
  GPIO_InitStructureCAN_TX.GPIO_Mode       = GPIO_Mode_AF ;      // GPIO mode assignement
  GPIO_InitStructureCAN_TX.GPIO_OType      = GPIO_OType_PP;      // Configring push pull mode
  GPIO_InitStructureCAN_TX.GPIO_PuPd       = GPIO_PuPd_UP;       // this activates the pullup resistors on the IO pins
  GPIO_InitStructureCAN_TX.GPIO_Speed      = GPIO_Speed_50MHz;
  GPIO_Init(CANx_GPIO, &GPIO_InitStructureCAN_TX);
  GPIO_PinAFConfig(CANx_GPIO, CANx_GPIO_SRC_TX, CANx_AF);


  // GPIO CAN Rx Config
  GPIO_InitStructureCAN_RX.GPIO_Pin        = CANx_RxPin;         // Pin assignemet for Rx
  GPIO_InitStructureCAN_RX.GPIO_Mode       = GPIO_Mode_AF ;      // GPIO mode assignement
  GPIO_InitStructureCAN_RX.GPIO_OType      = GPIO_OType_PP;      // Configring push pull mode
  GPIO_InitStructureCAN_RX.GPIO_PuPd       = GPIO_PuPd_UP;       // this activates the pullup resistors on the IO pins
  GPIO_InitStructureCAN_RX.GPIO_Speed      = GPIO_Speed_50MHz;
  GPIO_Init(CANx_GPIO, &GPIO_InitStructureCAN_RX);
  GPIO_PinAFConfig(CANx_GPIO, CANx_GPIO_SRC_RX, CANx_AF);

  CAN_DeInit(CAN_x);                                             // Deinitializes the CAN peripheral registers to their 
  // default reset values
  CAN_StructInit(&CAN_InitStructure);                            // Fills each CAN_InitStruct member with its default value
  CAN_InitStructure.CAN_TTCM = DISABLE;                          // time-triggered communication mode = DISABLED
  CAN_InitStructure.CAN_ABOM = DISABLE;                          // automatic bus-off management mode = DISABLED
  CAN_InitStructure.CAN_AWUM = DISABLE;                          // automatic wake-up mode = DISABLED
  CAN_InitStructure.CAN_NART = DISABLE;                          // non-automatic retransmission mode = DISABLED
  CAN_InitStructure.CAN_RFLM = DISABLE;                          // receive FIFO locked mode = DISABLED
  CAN_InitStructure.CAN_TXFP = DISABLE;                          // transmit FIFO priority = DISABLED
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;                  // normal CAN mode
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;                       // synchronization jump width = 1
  CAN_InitStructure.CAN_BS1 = CAN_BS1_14tq;                      // 14
  CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;                       // 6
  CAN_InitStructure.CAN_Prescaler = 4;                           // 4 -> baudrate 500 kbps
  CAN_Init(CAN_x, &CAN_InitStructure);                           // Initializes the CAN peripheral according to the settings above



  /// specified parameters for the CAN message filter
  CAN_FilterInitStructure.CAN_FilterNumber=0; 
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit; 
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);                      // Initializes the CAN filter according to the settings above

}                                                                // END of CAN_Configuration

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
  ADC_InitTypeDef ADC_InitStructure;                     // Structure to define ADC functionallity

  // Enable ADC1 Clock 
  RCC_APB2PeriphClockCmd(ADCx_CLK, ENABLE);              // enable clock for ADC
  RCC_AHB1PeriphClockCmd(ADCx_GPIO_CLK, ENABLE);         // enable clock for GPIO used 

  // ADC config for throttle
  GPIO_InitStruct.GPIO_Pin = ADCx_Pin;                   // Select pin at sepcific GPIO at which the ADC isw used
  GPIO_InitStruct.GPIO_Mode = ADCx_AF;                   // the pins are configured as alternate function 
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;          // No Pull
  GPIO_Init(ADCx_GPIO, &GPIO_InitStruct);                // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

  ADC_DeInit();                                          // Deinitializes the ADC peripheral registers to their
  // default reset values
  ADC_StructInit(&ADC_InitStructure);                    // Fills each ADC_InitStructure member with its default value.
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // ADC resolution 12Bit
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;           // ENABLE ScanConversion Mode
  ADC_InitStructure.ADC_ExternalTrigConvEdge= ADC_ExternalTrigConvEdge_None; // no external triggering
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                     // Data align to the right
  ADC_InitStructure.ADC_NbrOfConversion = 1;                                 // Number of Conversions = 1
  ADC_Init(ADC_x,&ADC_InitStructure);                                        // Initializes the ADC peripheral according to the settings above

  ADC_Cmd(ADC_x, ENABLE);
  ADC_RegularChannelConfig(ADC_x, ADC_Channel_x, 1, ADC_SampleTime_x);       //Configures for the selected ADC regular channel

}                                                        // END of ADC_Configuration

void SD_Configuration(void)
  /************************************************************************
  *  @brief  Configures the SD card used for data storage
  *  @param  data: user defined parameters from platform_cfg
  *  @retval  none
  * Additionnal   : Uses definitions found within platform_cfg.h
  ************************************************************************/ 
{
  GPIO_InitTypeDef GPIO_InitStruct;
  SPI_InitTypeDef SPI_InitStruct;

  /*CS Init*/
  RCC_AHB1PeriphClockCmd(SPI_CS_CLK, ENABLE);      // CHIP SELECT PIN, defined in defines.h GPIOB - pin 5
  GPIO_InitStruct.GPIO_Pin = SPI_CS_PIN;           // GPIO pin for SPI chip select 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;       // Set to output mode
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // Set to type PushPull
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;      // Pull Down
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  // GPIO speed 100Mhz
  GPIO_Init(SPI_CS_PORT, &GPIO_InitStruct);


  //Common settings for all pins
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // Set to type PushPull
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;    // Set to No Pull up Pull Down
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  // Speed 100Mhz
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        // Alternate Function mode

  //Enable clock for GPIOA
  RCC_AHB1PeriphClockCmd(GPIO_SPI_CLK, ENABLE);
  //                              SCK                   MISO              MOSI
  GPIO_InitStruct.GPIO_Pin = GPIO_SPI_SCK_pin | GPIO_SPI_MISO_pin | GPIO_SPI_MOSI_pin;
  GPIO_Init(GPIO_SDx, &GPIO_InitStruct);

  GPIO_PinAFConfig(GPIO_SDx, GPIO_SPI_SCK, GPIO_AF_SPIx);           // Remap alternate function for used GPIO pin as SPI
  GPIO_PinAFConfig(GPIO_SDx, GPIO_SPI_MISO, GPIO_AF_SPIx);          // Remap alternate function for used GPIO pin as SPI
  GPIO_PinAFConfig(GPIO_SDx, GPIO_SPI_MOSI, GPIO_AF_SPIx);          // Remap alternate function for used GPIO pin as SPI
  RCC_APB2PeriphClockCmd(SPI_CLK, ENABLE);                          // Enable RCC clock for SPI 

  SPI_StructInit(&SPI_InitStruct);
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128; // BAUDRATE
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;                    // Specify datasize   
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   // Fullduplex direction
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;                   // Specify which bit is first
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;                        // Specify mode of operation, clock polarity and clock phase
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;                           // Specify mode of operation, clock polarity and clock phase
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_Init(SPI_SDx, &SPI_InitStruct);                               // Initializes the SPI peripheral according to the settings above

  /* Enable SPIx */
  SPI_Cmd(SPI_SDx, ENABLE);


} // END of SD_Configuration

void UI_Configuration(void)
  /************************************************************************
  *  @brief  Configures User Interrupt GPIO and the Interrupt handler.
  *  @param  data: user defined parameters from platform_cfg
  *  @retval  none
  * Additionnal    : Conditionnal compile flags, Global variables and 
  *                  Constants defined in platform_cfg.h
  ************************************************************************/
{ 
  GPIO_InitTypeDef GPIO_InitStruct;    // Define GPIO structure
  EXTI_InitTypeDef EXTI_InitStruct;    // Define External interrupt structure 
  NVIC_InitTypeDef NVIC_InitStruct;    // Define NVIC structure

  // Enable clock for GPIOB
  RCC_AHB1PeriphClockCmd(GPIO_UI_BTN_CLK, ENABLE);
  // Enable clock for SYSCFG 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  // Set pin as input 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;        // Input mode
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // Push pull
  GPIO_InitStruct.GPIO_Pin = UI_BTN;               // Pin0
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;    // GPIO_PuPd_UP
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  // 100Mhz
  GPIO_Init(GPIO_UI_BTN, &GPIO_InitStruct);        // Initialize Structure


  SYSCFG_EXTILineConfig(UI_BTN_EXTI_PORTSRC, UI_BTN_EXTI_PINSRC);   // Connect EXTI Line0 to PA0 pin
  EXTI_InitStruct.EXTI_Line = UI_BTN_EXTI_LINE;                     // PA0 is connected to EXTI_Line0
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;                            // Enable interrupt 
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;                  // Interrupt mode 
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;               // Triggers on rising edge 
  EXTI_Init(&EXTI_InitStruct);                                      // Add to EXTI

  // Add IRQ vector to NVIC 
  NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;                 // PA0 is connected to EXTI_Line0, EXTI0_IRQn 
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;     // Set priority 
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;            // Set sub priority 
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;                  // Enable interrupt 
  NVIC_Init(&NVIC_InitStruct);                                  // Add to NVIC


  // BUTTON_USER and BUTTON_MODE_EXTI are defined within respective structures
  // STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);         

  // Enable clock for GPOIA used as User interrupt signal (set to HIGH if button pressed)
  RCC_AHB1PeriphClockCmd(GPIO_UI_CLK, ENABLE);

  GPIO_InitStruct.GPIO_Pin = UIx_GPIO_pin;                      // Pin PA.2
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;                    // Mode output
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;                   // Output type push-pull
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;                 // Without pull resistors
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;                // 50MHz pin speed

  //Initialize pins on GPIOA port
  GPIO_Init(UIx_GPIO, &GPIO_InitStruct);

  // set output to LOW initally
  GPIO_ResetBits(UIx_GPIO, UIx_GPIO_pin);         

} // END of UI_Configuration

void  Timer_Configuration(void)
  /************************************************************************
  *  @brief   Configures a timer to be used for various purposes.
  *  @param   data: user defined parameters from platform_cfg
  *  @retval  none
  * Additionnal   : Uses structure definitions found in stm32f4xx_tim.h
  ************************************************************************/
{
  TIM_TimeBaseInitTypeDef timerInitStructure;               // Create timer structure
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);      // Set RCC clock for TIMER entity

  timerInitStructure.TIM_Prescaler = 40000;                 // Set prescaler
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  // Set couting direction >
  timerInitStructure.TIM_Period = 5000;                     // Set timer tick period
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;      // Set clock divions, from fundemental clock
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timerInitStructure);              // Initialize timer structure on port TIM2  
  TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);          // Define mode
}

#else
#error: "Illegal target board selection!"
#endif
