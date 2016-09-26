/**
*  ***********************************************************************
* @file    CanLib.c
* @author  Madhan and Aravind
* @version V2.0
*
* @brief   Collection of functions for STM32F4xx CAN-IO.  
*           
* Changes:
* 14.05.2014  Mys target independent version
* 20.08.2014  Correction of MotCon/BMS parameter scaling factors
* 26.09.2014  Janezic Sommerauer: Adaption to Discovery board
* 05.10.2014  Janezic Sommerauer: Adjusted display patern and consistance with value and MMI screen name
* 15.10.2014  Janezic Sommerauer: Added non local base&offset address for CAN
* 15.12.2014  Janezic Sommerauer: Added further lcdparam values to be displayed on LCD
* 08.02.2015  Janezic Sommerauer: Added Debug option for LCD
* 24.03.2015  Aravind & Madhan  : New CAN address implementation for Motor parameters
* 16.06.2015  Aravind & Madhan  : added float to string function for conversion of power and voltage values of the battery
* 15.10.2015  Aravind & Madhan  : added fixedpoint function for inserting a dot "." at desired place in an integer
* @date       15/10/2015
**************************************************************************
*/

/************************************************************************\
*    Headerfiles 
\************************************************************************/

#ifdef USE_DISCOVERY
  #include "stm32f4xx.h" 
  #include "stm32f4xx_can.h"
#elif defined USE_MCBSTM32C
  #include "stm32f10x.h"                    
  #include "stm32f10x_can.h"
#else
  #error: "Illegal target board selection!"
#endif

#include "bsp.h"
#include "BasicIO.h" 
#include "mctDefs.h"
#include "CanDefs.h"
#include "platform_cfg.h"

/************************************************************************\
*                            NUMERICAL CONSTANTS
\************************************************************************/

#define CAN_CONV_BATTV  57.45
#define CAN_CONV_BATTC  10
#define CAN_CONV_MOTRPM 10
#define KW_CONV         1000.0

/************************************************************************\
*        GLOBAL VARIABLES
\************************************************************************/

CanRxMsg                     CanRecMsg;               //!< CAN Rx message structure 
CanTxMsg                     CanSendMsg;              //!< CAN Tx message structure
CanDisplayMotorParams        MotorParams;             //!< Contains Motor Parameters
CanDisplayBatteryParams      BatteryParams;           //!< Contains RT-BMS master parameters
LcdDisplayParams             LCDParam;                //!< Contains parameters that are to be displayed on the LCD screen

CanDisplayMotorParams        MotorParamsCondition;    //!< Motor structure for color display
CanDisplayBatteryParams      BatteryParamsCondition;  //!< Battery structure for color display

/************************************************************************\
*    local functions
\************************************************************************/

void Initialize_LCDStructure(void)
  /**
  *  ********************************************************************************
  * @brief  Initializes the LCDParam structures with zeros 
  * @param  None
  * @retval None
  ***********************************************************************************
  */   
{
  UINT8 ii;

  LCDParam.BatChargeCap=0.0;
  LCDParam.Throttle_Status=0.0;
  for(ii=0;ii<MAXDigits+1;ii++)
  {
    LCDParam.BatVoltage_str[ii]=0;              
    LCDParam.BatCurrent_str[ii]=0;        
    LCDParam.RPM_str[ii]=0;                
    LCDParam.Power_str[ii]=0;              
    LCDParam.TMotor_str[ii]=0;            
    LCDParam.TBattHigh_str[ii]=0;          
    LCDParam.TBattLow_str[ii]=0;          
    LCDParam.TCont_str[ii]=0;              
  }
}

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
  UINT16  throttle_lo, throttle_hi;

  LCDParam.Throttle_Status = (float)throttle_value/MAXThrottle;      // Update throttle status value for LCD 

  CanSendMsg->ExtId = BaseAdr + OffsetAdr_Mot ;   // Specifies the extended identifier         
  CanSendMsg->DLC   = 8;                          // Specifies the length of the frame that will be transmitted
  CanSendMsg->RTR   = CAN_RTR_DATA ;              // Specifies the type of frame for the message that will be transmitted
  CanSendMsg->IDE   = CAN_ID_EXT;                 // Specifies the type of identifier for the message that will be transmitted
  throttle_lo = throttle_value & 0x00FF;          // Lower 4 bits of the scaled throttle value that is to be transmitted
  throttle_hi = throttle_value >> 8;              // Higher 4 bits of the scaled throttle value that is to be transmitted
  for (i = 0; i < 8; i++) 
    CanSendMsg->Data[i] = 0;                      // clear tx msg

  /* fill in msg payload */
  CanSendMsg->Data[0] = throttle_lo;
  CanSendMsg->Data[1] = throttle_hi;

  /* Send throttle value via can to motor controller */
  CAN_Transmit(CAN_OpStruct, CanSendMsg);         // Actual sending via can message

} // end of CAN_FillThrottlemsg()

void CAN_ReceiveMessage(CAN_TypeDef* CANx_used, CanRxMsg* CanReceiveMessage)
  /**
  *  ********************************************************************************
  * @brief  Receive CAN messages on CANx.
  * @param  CANx: where x can be 1 or 2 to to select the CAN peripheral.
  * @param  CanReceiveMessage: OUT buffer that stores CAN message
  * @retval None
  ***********************************************************************************
  */  
{     
  CAN_Receive(CANx_used, CAN_FIFO0, CanReceiveMessage);

#ifdef PRINT_CAN_Messages 
  CAN_PrintMsgDetails(CanReceiveMessage);
#endif  
}                                                 // end of CAN_ReceiveMessage()

void CAN_ReceiveMotorParameters (CanRxMsg* CanRecMsg, CanDisplayMotorParams* MotorParam)
  /**
  *  ********************************************************************************
  * @brief  Converts motor controller CAN data into scaled display output values
  * @param  CanRecMsg:  IN  one CAN RX packet with IDs 0x14A10041 .. 0x14A10043 
  * @param  MotorParam: OUT scaled motor controller values for decimal output
  * @param  User defined parameters BaseAdr + OffsetAdr_Bat from platform_cfg
  * @retval None
  ***********************************************************************************
  */

  // Note: packet-IDs & scaling factors are documented in file 'CAN_ESC_140221b.pdf'
{
  if ((CanRecMsg->ExtId) == BaseAdr + OffsetAdr_Mot + 1 )
  {
    /* Convert motor controller CAN data (selecting and scaling data) */
    MotorParam->BatVoltage = (float) ((CanRecMsg->Data[0] + CanRecMsg->Data[1]*256 )/CAN_CONV_BATTV);                     // scaled in [V}
    MotorParam->BatCurrent = (CanRecMsg->Data[2] + CanRecMsg->Data[3]*256 )/CAN_CONV_BATTC;                               // scaled in [A]
    MotorParam->MotorRPM   = (CanRecMsg->Data[4] + CanRecMsg->Data[5]*256 + CanRecMsg->Data[6]*65536 )*CAN_CONV_MOTRPM;   // scaled in [rpm]
    MotorParam->Power      = ((MotorParam->BatVoltage * MotorParam->BatCurrent)/KW_CONV);                                 // Scaled in KW

    MotorParamsCondition.BatVoltage  = MotorParam->BatVoltage;  //Structure for conditional color display 
    MotorParamsCondition.BatCurrent  = MotorParam->BatCurrent;  //Structure for conditional color display 
    MotorParamsCondition.MotorRPM  = MotorParam->MotorRPM;      //Structure for conditional color display 

    /* Store parameters to be displayed on lcd in structure */
    ftostr(MotorParam->BatVoltage, LCDParam.BatVoltage_str, 1);
    uint2str(MotorParam->BatCurrent, LCDParam.BatCurrent_str, DISABLE);
    uint2str(MotorParam->MotorRPM, LCDParam.RPM_str, DISABLE);
    ftostr(MotorParam->Power, LCDParam.Power_str, 1);          
  }

#ifdef LCDDEBUG  // Test LCD if no Can data
  uint2str(1000, LCDParam.BatVoltage_str, DISABLE);                        
  uint2str(200, LCDParam.BatCurrent_str, DISABLE);
  uint2str(504, LCDParam.RPM_str, DISABLE);
  uint2str(30040, LCDParam.Power_str, DISABLE);
#else
#endif

  if ((CanRecMsg->ExtId) == BaseAdr + OffsetAdr_Mot + 2)
  {
    /* Convert motor controller CAN data (selecting and scaling data) */
    MotorParam->Odometer       = (CanRecMsg->Data[0] +(CanRecMsg->Data[1] << 8)+(CanRecMsg->Data[2] << 16)+(CanRecMsg->Data[3] << 24));
    MotorParam->ControllerTemp = (CanRecMsg->Data[4]) ;
    MotorParam->MotorTemp      = (CanRecMsg->Data[5]) ;
    MotorParam->BatteryTemp    = (CanRecMsg->Data[6]) ;

    MotorParamsCondition.MotorTemp  = MotorParam->MotorTemp;           //Structure for conditional color display
    MotorParamsCondition.ControllerTemp  = MotorParam->ControllerTemp; //Structure for conditional color display

    /* Store parameters to be displayed on lcd in structure */
    uint2str(MotorParam->MotorTemp, LCDParam.TMotor_str, DISABLE);
    uint2str(MotorParam->ControllerTemp, LCDParam.TCont_str, DISABLE);
  }

#ifdef LCDDEBUG                                                        // Test LCD if no Can data
  uint2str(122, LCDParam.TMotor_str, DISABLE);
  uint2str(60, LCDParam.TCont_str, DISABLE);
#else
#endif  

  if ((CanRecMsg->ExtId) == BaseAdr + OffsetAdr_Mot + 3)
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


void CAN_ReceiveBatteryParameters(CanRxMsg* CanRecMsg, CanDisplayBatteryParams* BatteryParam)
/**
*  ********************************************************************************
* @brief  Converts BMS CAN data into scaled display output values
* @param  CanRecMsg:    IN  one CAN RX packet with IDs 0x14A10101, 0x14A10102.
* @param  BatteryParam: OUT scaled BMS values for decimal output
* @param  User defined parameters BaseAdr + OffsetAdr_Bat from platform_cfg
* @retval None
***********************************************************************************
*/
// Note: packet-IDs & scaling factors are documented in file 'CAN-BMS.pdf'
{
  if((CanRecMsg->ExtId) == BaseAdr + OffsetAdr_Bat)                                      // 1st BMS CAN packet
  {
    /* Convert BMS CAN data (selecting and scaling data) */
    BatteryParam->BatteryTotalCapacity =  CanRecMsg->Data[0];         // scaled in [%]
    BatteryParam->Current              = (CanRecMsg->Data[1]-128)/6;  // scaled in [A]
    BatteryParam->TotalVoltage         = (CanRecMsg->Data[2]+(CanRecMsg->Data[3] << 8));   // scaled in 0.1[V]
    BatteryParam->HighestTemperature   =  CanRecMsg->Data[4];         // scaled in [oC]
    BatteryParam->Notification         =  CanRecMsg->Data[5];
    BatteryParam->lastErrorAddress     =  CanRecMsg->Data[6];
    BatteryParam->InternalErrorCode    =  CanRecMsg->Data[7];

    BatteryParamsCondition.HighestTemperature   = BatteryParam->HighestTemperature;      // for conditional color display
    BatteryParamsCondition.BatteryTotalCapacity = BatteryParam->BatteryTotalCapacity;    // for conditional color display
    /* Store parameters to be displayed on lcd in structure */
    LCDParam.BatChargeCap=(float)BatteryParam->BatteryTotalCapacity/100;
    uint2str(BatteryParam->HighestTemperature, LCDParam.TBattHigh_str, DISABLE);
    uint2str(BatteryParam->BatteryTotalCapacity, LCDParam.batchargecap_str, DISABLE);    // for conditional color display
    fixedpoint(BatteryParam->TotalVoltage, LCDParam.BattTot_vol, 1);  
  }
  
#ifdef LCDDEBUG
  LCDParam.BatChargeCap = (float)70/100;  // 0.7 ???
  uint2str(223, LCDParam.TBattHigh_str, DISABLE);
#else
#endif  

  if((CanRecMsg->ExtId) == BaseAdr + OffsetAdr_Bat + 1)                                  // 2nd BMS CAN packet
  {
    /* Convert BMS CAN data (selecting and scaling data) */
    BatteryParam->LowestVoltage        = (CanRecMsg->Data[0]+(CanRecMsg->Data[1] << 8)); // scaled in 0.01[V]
    fixedpoint(BatteryParam->LowestVoltage, LCDParam.BattLow_vol, 2);
    BatteryParam->LowestVoltageAddr    =  CanRecMsg->Data[2];
    BatteryParam->HighestVoltage       = (CanRecMsg->Data[3]+(CanRecMsg->Data[4] << 8)); // scaled in 0.01[V]
    BatteryParam->HighestVoltageAddr   =  CanRecMsg->Data[5];
    BatteryParam->LowestTemperature    =  CanRecMsg->Data[6];
    fixedpoint(BatteryParam->LowestTemperature, LCDParam.TBattLow_str, DISABLE);
    
    BatteryParamsCondition.LowestTemperature  = BatteryParam->LowestTemperature;         //Structure for conditional color display
    BatteryParamsCondition.LowestVoltageAddr  = BatteryParam->LowestVoltageAddr;
    BatteryParamsCondition.HighestVoltageAddr  = BatteryParam->HighestVoltageAddr;
    /* Store parameters to be displayed on lcd in structure */ 
    
    fixedpoint(BatteryParam->HighestVoltage, LCDParam.BattHigh_vol, 2);
    fixedpoint(BatteryParam->LowestVoltageAddr, LCDParam.TBattLVA_str, DISABLE);
    fixedpoint(BatteryParam->HighestVoltageAddr, LCDParam.TBattHVA_str, DISABLE);

  }
  
#ifdef LCDDEBUG
  uint2str(21, LCDParam.TBattLow_str, DISABLE);
#else
#endif
  
}                                                                                        // end of CAN_ReceiveBatteryParameters()


void  CAN_PrintMsgDetails(CanRxMsg* CanRecMsg )
  /**
  *  ********************************************************************************
  * @brief  Print details of a CAN message.
  * @param  CanRecMsg: OUT buffer that stores CAN message 
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


/************************************************************************\
*      Ende of Module CanLib.c                                   
\************************************************************************/
