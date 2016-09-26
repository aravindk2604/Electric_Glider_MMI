/**
*  ***********************************************************************
*  @file    CommMotConBMS.c
*  @author  R&P / Mys
*  @version V1.31
*
*  @brief   Communication functions for HBC25063 motor controller & BMS-2
*           via CAN-bus.
*
*  Changes:
*  14.05.2014  Mys target independent version
*  20.08.2014  R&P Correction of MotCon/BMS parameter scaling factors
*  17.03.2015  mys Begin of streamlining and removal of R&P's bullsh..
*  18.03.2015  mys Adaptation to new 'shifted+1' CAN -IDs for new
*                  HBC25063 motor controller from MGM
*  20.08.2015  mys Cont'd Streamlining, CanLib.x -> CommMotConBMS.x
*
*  @date    20/08/2015
**************************************************************************
*/

//#define PRINT_CAN_MESSAGES                   //!< enable debug output

/************************************************************************\
*        HEADERFILES
\************************************************************************/

#include "stm32f10x.h"

#include "bsp.h"                               //!< low level IO
#include "..\BasicIO\BasicIO.h"                //!< mid level formatted IO

#include "CommMotConBMS.h"                     //!< communication related defs

/************************************************************************\
*        GLOBAL VARIABLES
\************************************************************************/

CanRxMsg                     CanRecMsg;        //!< CAN Rx message structure 
CanTxMsg                     CanSendMsg;       //!< CAN Tx message structure
CAN_FilterInitTypeDef        Can_filter;       //!< Can Filter structure definition

MotConParamBlock             MotorParams;      //!< Contains Motor Parameters
BMSParamBlock                BatteryParams;    //!< Contains RT-BMS master parameters

/************************************************************************\
*        EXTERNAL VARIABLES
\************************************************************************/

extern CAN_InitTypeDef    Can_struct;          //!< CAN init structure


/************************************************************************\
*    local functions
\************************************************************************/

// CAN_OpStruct WAS/IS NOT used ... !!!      old R&P bullsh..

void CAN_UserInit(CAN_TypeDef* CAN_OpStruct, CAN_InitTypeDef* CAN_InitStruct)  
/**
*  ***********************************************************************
*  @brief  Initialize selected CAN controller w 500kbit/s (@ 36MHz).
*  @param  CAN_OpStruct   : IN  buffer contains information regarding the mail boxes.
*  @param  CAN_InitStruct : IN  buffer contains CAN init structure definition.
*  @retval none
**************************************************************************
*/
{
#if defined USE_MCBSTM32C  

  /*Initialization of CAN1*/
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;           //enable clock for CAN1
                                                   //Note: MCBSTM32C CAN1 uses PD0 and PD1
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;           //enable clock for Alternate Function
  AFIO->MAPR   &= 0xFFFF9FFF;                   //reset CAN1 remap
  AFIO->MAPR   |= 0x00006000;                   //set CAN1 remap, use PD0, PD1
                            
  RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;           //enable clock for GPIOD
  GPIOD->CRL &= ~(0x0F<<0);
  GPIOD->CRL |=  (0x08<<0);                     //CAN RX pin PD.0 input push pull 
    
  GPIOD->CRL &= ~(0x0F<<4);
  GPIOD->CRL |=  (0x0B<<4);                     //CAN TX pin PD.1 alternate output push pull
    
  CAN_DeInit(CAN1);                             //Deinitializes the CAN peripheral registers to their 
                                             //default reset values
  CAN_StructInit(CAN_InitStruct);            //Fills each CAN_InitStruct member with its default value.
  CAN_InitStruct->CAN_SJW = CAN_SJW_4tq;        //Initialize the CAN_SJW member          1-4
  CAN_InitStruct->CAN_BS1 = CAN_BS1_12tq;        //Initialize the CAN_BS1 member        1-16
  CAN_InitStruct->CAN_BS2 = CAN_BS2_5tq;        //Initialize the CAN_BS2 member        1-8
  CAN_InitStruct->CAN_Prescaler = 4;          //Initialize the CAN_Prescaler member

/*Clk=72MHz 
  HCLK = SYSCLK 
  PCLK2 = HCLK 
  PCLK1 = HCLK/2 
  BaudRate=36/Prescaler/(sjw+bs1+bs2)  = 9/(1+12+5) = 500K
  sample point = (1+bs1)/(1+bs1+bs2)  = 13/18 = 72%
*/

  CAN_Init(CAN1, CAN_InitStruct);              //Initializes the CAN peripheral according to the 
                                           //specified parameters in the CAN_InitStruct
  Can_filter.CAN_FilterNumber=0; 
  Can_filter.CAN_FilterMode=CAN_FilterMode_IdMask; 
  Can_filter.CAN_FilterScale=CAN_FilterScale_16bit; 
  Can_filter.CAN_FilterFIFOAssignment=0;
  Can_filter.CAN_FilterActivation=ENABLE;
//Can_filter.CAN_FilterIdHigh=0x0000; 
//Can_filter.CAN_FilterIdLow=0x0000; 
//Can_filter.CAN_FilterMaskIdHigh=0x0000; 
//Can_filter.CAN_FilterMaskIdLow=0x0000; 

  CAN_FilterInit(&Can_filter);


  /*Initialization of CAN2*/
  RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;           //enable clock for CAN2
                                          //Note: MCBSTM32C CAN2 uses PB5 and PB6
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;           //enable clock for Alternate Function

  AFIO->MAPR   &= 0xFFBFFFFF;                   //reset CAN remap
  AFIO->MAPR   |= 0x00400000;                   //set CAN2 remap, use PB.5, PB.6

  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;           //enable clock for GPIO B
  GPIOB->CRL &= ~(0x0F<<20);
  GPIOB->CRL |=  (0x08<<20);                     //CAN RX pin PB.5 input push pull 

  GPIOB->CRL &= ~(0x0F<<24);
  GPIOB->CRL |=  (0x0B<<24);              //CAN TX pin PB.6 alternate output push pull

  CAN_DeInit(CAN2);                             //Deinitializes the CAN peripheral registers to their 
  CAN_StructInit(CAN_InitStruct);            //Fills each CAN_InitStruct member with its default value.
  CAN_InitStruct->CAN_SJW = CAN_SJW_4tq;        //Initialize the CAN_SJW member          1-4
  CAN_InitStruct->CAN_BS1 = CAN_BS1_12tq;        //Initialize the CAN_BS1 member        1-16
  CAN_InitStruct->CAN_BS2 = CAN_BS2_5tq;        //Initialize the CAN_BS2 member        1-8
  CAN_InitStruct->CAN_Prescaler = 4;          //Initialize the CAN_Prescaler member

/*Clk=72MHz 
  HCLK = SYSCLK 
  PCLK2 = HCLK 
  PCLK1 = HCLK/2 
  BaudRate=36/Prescaler/(sjw+bs1+bs2)  = 9/(1+12+5) = 500K
  sample point = (1+bs1)/(1+bs1+bs2)  = 13/18 = 72%
*/

  CAN_Init(CAN2, CAN_InitStruct);              //Initializes the CAN peripheral according to the 
                                          //specified parameters in the CAN_InitStruct
  Can_filter.CAN_FilterNumber=14; 
  Can_filter.CAN_FilterMode=CAN_FilterMode_IdMask; 
  Can_filter.CAN_FilterScale=CAN_FilterScale_16bit; 
  Can_filter.CAN_FilterFIFOAssignment=0;
  Can_filter.CAN_FilterActivation=ENABLE;
  Can_filter.CAN_FilterIdHigh=0x0000; 
  Can_filter.CAN_FilterIdLow=0x0000; 
  Can_filter.CAN_FilterMaskIdHigh=0x0000; 
  Can_filter.CAN_FilterMaskIdLow=0x0000; 

  CAN_FilterInit(&Can_filter);

#elif defined USE_MCBSTM32

//unsigned int brp = BSP_PeriphClkFreqGet(1);        // unsigned int brp = stm32_GetPCLK1();     
  unsigned int brp = BSP_CPU_ClkFreq() / 2;

  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;                 /// enable clock for CAN
                                                      /// Note: MCBSTM32 uses PB8 and PB9 for CAN
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                 /// enable clock for Alternate Function
  AFIO->MAPR   &= 0xFFFF9FFF;                         /// reset CAN remap
  AFIO->MAPR   |= 0x00004000;                         /// set CAN remap, use PB8, PB9
                          
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                 /// enable clock for GPIO B
  GPIOB->CRH &= ~(0x0F<<0);
  GPIOB->CRH |=  (0x08<<0);                           /// CAN RX pin PB.8 input push pull 
  
  GPIOB->CRH &= ~(0x0F<<4);
  GPIOB->CRH |=  (0x0B<<4);                           /// CAN TX pin PB.9 alternate output push pull 
  
  CAN_DeInit(CAN1);                                   /// Deinitializes the CAN peripheral registers to their 
                                                  /// default reset values
  CAN_StructInit(CAN_InitStruct);                /// Fills each CAN_InitStruct member with its default value.
  CAN_InitStruct->CAN_SJW = CAN_SJW_4tq;              /// Initialize the CAN_SJW member          1-4
  CAN_InitStruct->CAN_BS1 = CAN_BS1_12tq;            /// Initialize the CAN_BS1 member        1-16
  CAN_InitStruct->CAN_BS2 = CAN_BS2_5tq;            /// Initialize the CAN_BS2 member        1-8
  CAN_InitStruct->CAN_Prescaler = 4;              /// Initialize the CAN_Prescaler member

/*Clk=72MHz 
  HCLK = SYSCLK 
  PCLK2 = HCLK 
  PCLK1 = HCLK/2 
  BaudRate=36/Prescaler/(sjw+bs1+bs2)  = 9/(1+12+5) = 500K
  sample point = (1+bs1)/(1+bs1+bs2)  = 13/18 = 72%
*/
    
  CAN_Init(CAN1, CAN_InitStruct);                  /// Initializes the CAN peripheral according to the 
  /// specified parameters in the CAN_InitStruct
  Can_filter.CAN_FilterNumber=0; 
  Can_filter.CAN_FilterMode=CAN_FilterMode_IdMask; 
  Can_filter.CAN_FilterScale=CAN_FilterScale_16bit; 
  Can_filter.CAN_FilterFIFOAssignment=0;
  Can_filter.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&Can_filter);

#else
  #error: Unknown Target Board Selection !
#endif

}  // end of CAN_UserInit()                         


void CAN_FillThrottlemsg(CAN_TypeDef* CAN_OpStruct , CanTxMsg* CanSendMsg, UINT32 throttle_value)
/**
*  ********************************************************************************
  * @brief  Transmit Scaled ADC value via CAN
  * @param  CAN_OpStruct: IN Contains CAN_typedef parameters.
  * @param  CanSendMsg: OUT Contains CAN transmission parameters.
  * @param  throttle_value: IN value Contains the smoothened throttle value.  
  * @retval None
***********************************************************************************
*/ 
{
  unsigned short  i;
//UINT16  throttle_lo, throttle_hi;

  CanSendMsg->ExtId = 0x14A10041;           // ID for throttle command value {0..1023}         
  CanSendMsg->DLC   = 8;                    // frame length
  CanSendMsg->RTR   = CAN_RTR_DATA ;        // frame type
  CanSendMsg->IDE   = CAN_ID_EXT;           // use extended 29 bit identifier

//throttle_lo = throttle_value & 0x00FF;    // low  8 byte of scaled throttle value
//throttle_hi = throttle_value >> 8;        // high 4 bits of the scaled throttle value that is to be transmitted

  for (i = 0; i < 8; i++) 
     CanSendMsg->Data[i] = 0;               /// clear tx msg
   
  /// fill in msg payload
  CanSendMsg->Data[0] = throttle_value & 0x00FF; // low  byte of scaled throttle value
  CanSendMsg->Data[1] = throttle_value >> 8;     // high byte of scaled throttle value

  CAN_Transmit(CAN_OpStruct, CanSendMsg);
    
} // end of CAN_FillThrottlemsg()


void CAN_ReceiveMessage(CAN_TypeDef* CANx, CanRxMsg* CanReceiveMessage)
/**
*  ********************************************************************************
  * @brief  Receive CAN messages on CANx.
  * @param  CANx: where x can be 1 or 2 to to select the CAN peripheral.
  * @param  CanReceiveMessage: OUT buffer that stores CAN message
  * @retval None
***********************************************************************************
*/  
{     
//CAN_Receive(CAN1, CAN_FIFO0, CanReceiveMessage);

  CAN_Receive(CANx, CAN_FIFO0, CanReceiveMessage);

#ifdef PRINT_CAN_MESSAGES 
  CAN_PrintMsgDetails(CanReceiveMessage);
#endif  

} // end of CAN_ReceiveMessage()


void CAN_PrintMsgDetails(CanRxMsg* CanRecMsg )
/**
*  ********************************************************************************
  * @brief  Print details of a CAN message.
  * @param  CanRecMsg:OUT buffer that stores CAN message 
  * @retval None
***********************************************************************************
*/   
{
  UINT16 i;
  
  print_string("  ID");  print_hex32(CanRecMsg->ExtId); 
  print_string("  IDE"); print_hexbyte(CanRecMsg->IDE);
  print_string("  RTR"); print_hexbyte(CanRecMsg->RTR);
  print_string("  DLC"); print_int(CanRecMsg->DLC);
  print_string("  FMI"); print_hexbyte(CanRecMsg->FMI);
  print_string("  DATA ");

  for(i=0; i<CanRecMsg->DLC; i++)     ///data[] depends on DLC
  {   
    print_hexbyte(CanRecMsg->Data[i]);
    _putch(' ');
  }
  print_string("  \r");

} // end of CAN_PrintMsgDetails()


#define  MOTCON_CAN_BASE    0x14A10000
#define  ADRESS_OFFSET      0x41

void CAN_ReceiveMotorParameters (CanRxMsg* CanRecMsg, MotConParamBlock* MotorParam)
/**
*  ********************************************************************************
  * @brief  Converts motor controller CAN data into scaled display output values
  * @param  CanRecMsg:  IN  CAN RX packet with IDs 0x14A10041 .. 0x14A10043 
  * @param  MotorParam: OUT scaled motor controller values for decimal output
  * @retval None
***********************************************************************************
*/

// Note: packet-IDs & scaling factors are documented in file 'CAN_ESC_140221b.pdf'
{
//if ( (CanRecMsg->ExtId) == 0x14A10041 ) // OLD
  if ( (CanRecMsg->ExtId) == (MOTCON_CAN_BASE+ADRESS_OFFSET+1) ) // packet#1
  {
     MotorParam->BatVoltage = (UINT16) ((CanRecMsg->Data[0] + CanRecMsg->Data[1]*256 ) / 5.745);              /// scaled in 0.1[V}
     MotorParam->BatCurrent = (CanRecMsg->Data[2] + CanRecMsg->Data[3]*256 );                                 /// scaled in 0.1[A]
     MotorParam->MotorRPM   = (CanRecMsg->Data[4] + CanRecMsg->Data[5]*256 + CanRecMsg->Data[6]*65536 )*10;   /// scaled in [rpm]
  }
    
  if ((CanRecMsg->ExtId) == 0x14A10042+1) // OLD
  {
    MotorParam->Odometer       = (CanRecMsg->Data[0] +(CanRecMsg->Data[1] << 8)+(CanRecMsg->Data[2] << 16)+(CanRecMsg->Data[3] << 24));
    MotorParam->ControllerTemp = (CanRecMsg->Data[4]) ;
    MotorParam->MotorTemp      = (CanRecMsg->Data[5]) ;
    MotorParam->BatteryTemp    = (CanRecMsg->Data[6]) ;
  }  

  if ((CanRecMsg->ExtId) == 0x14A10043+1) // OLD
  {
    MotorParam->WarningBits = CanRecMsg->Data[4] +(CanRecMsg->Data[5] << 8);
/*****************************
  print_hex16(WarningBits);
  if( (Warnings & 0x001) == 1)
  print_string("Low Voltage"); 
  if(Warnings == 2)
  print_string("High Current");
  if(Warnings == 4)
  print_string("Controller Overheated");
  if(Warnings == 8)
  print_string("Battery Overheated");
  if(Warnings == 16)
  print_string("Motor Overheated");
************************/
    MotorParam->ErrorBits = CanRecMsg->Data[6] +(CanRecMsg->Data[7] << 8);
/****************
  print_hex16(ErrorBits);
  if(Error == 1)
  print_string("Input signal out of range/not present"); 
  if(Error == 2)
  print_string("Waiting for throttle idle position"); 
  if(Error == 4)
  print_string("Power off request"); 
  if(Error == 8)
  print_string("Memory error"); 
  if(Error == 10)
  print_string("Power off request (settings change)"); 
  if(Error == 20)
  print_string("Motor error (HAL sensors)"); 
  if(Error == 40 )
  print_string("Internal power source error"); 
  if(Error == 80)
  print_string("Power off request – HAL position learning procedure completed sucessfully"); 
  if(Error == 100)
  print_string("Battery temperature sensor error"); 
  if(Error == 200)
  print_string("Motor temperature sensor error"); 
  if(Error == 8000)
  print_string("Other HW error");
************************/ 
  }
	
} //end of CAN_ReceiveMotorParameters() 

#define  BMS_CAN_BASE       0x14A10100


void CAN_ReceiveBatteryParameters(CanRxMsg* CanRecMsg, BMSParamBlock* BatteryParam)
/**
*  ********************************************************************************
  * @brief  Converts BMS CAN data into scaled display output values
  * @param  CanRecMsg:  IN  one CAN RX packet with IDs 0x14A10101, 0x14A10102.
  * @param  BatteryParam:OUT scaled BMS values for decimal output
  * @retval None
***********************************************************************************
*/
// Note: packet-IDs & scaling factors are documented in file 'CAN-BMS.pdf'
{
//if((CanRecMsg->ExtId) == 0x14A10101)   // 1st BMS CAN packet
  if( (CanRecMsg->ExtId) == (BMS_CAN_BASE+1) ) // 1st BMS CAN packet
  {
    BatteryParam->BatteryTotalCapacity =  CanRecMsg->Data[0];                           // scaled in % units
    BatteryParam->Current              = (CanRecMsg->Data[1]-128)/6;                   // scaled in A units 
    BatteryParam->TotalVoltage         = (CanRecMsg->Data[2]+(CanRecMsg->Data[3] << 8)); // scaled in 0.1[V] units
    BatteryParam->HighestTemperature   =  CanRecMsg->Data[4];
    BatteryParam->Notification         =  CanRecMsg->Data[5];
    BatteryParam->lastErrorAddress     =  CanRecMsg->Data[6];
    BatteryParam->InternalErrorCode    =  CanRecMsg->Data[7];
  }

//if((CanRecMsg->ExtId) == 0x14A10102) // 2nd BMS CAN packet
  if( (CanRecMsg->ExtId) == (BMS_CAN_BASE+2) ) // 2nd BMS CAN packet
  {
    BatteryParam->LowestVoltage        = (CanRecMsg->Data[0]+(CanRecMsg->Data[1] << 8)); // scaled in 0.01[V] units
    BatteryParam->LowestVoltageAddr    =  CanRecMsg->Data[2];
    BatteryParam->HighestVoltage       = (CanRecMsg->Data[3]+(CanRecMsg->Data[4] << 8)); // scaled in 0.01[V] units
    BatteryParam->HighestVoltageAddr   =  CanRecMsg->Data[5];
    BatteryParam->LowestTemperature    =  CanRecMsg->Data[6];
  }
	
} // end of CAN_ReceiveBatteryParameters()

/************************************************************************\
*      Ende of Module CommMotConBMS.c                                   
\************************************************************************/
