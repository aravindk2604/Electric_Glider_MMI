/**
*  ***********************************************************************
*  @file      CommMotConBMS.h
*  @author    Mys
*  @version   V2.00
*  @brief     Defines for HBC25063 motor controller & BMS-2 communication  
*             via CAN bus.
*  Changes:
*  20.08.2015 Mys Cont'd Streamlining, CanLib.x -> CommMotConBMS.x
*
*  @date      20/08/2015
**************************************************************************
*/

/************************************************************************\
*     HEADERFILES USED 
\************************************************************************/

#include    <stm32f10x_can.h>
#include    "..\BasicIO\mctDefs.h"             // needed for type defs


/************************************************************************\
*     HBC25063 MOTOR CONTROLLER DATA STRUCTURE
\************************************************************************/
//    controller data are received via CAN, throttle_cmd sent via CAN

typedef struct
{
  u16 BatVoltage;         //!< Battery voltage in [0.1 V]          
  u32 BatCurrent;         //!< Battery current in [0.1 A]
  u32 MotorRPM;           //!< Motor speed     in [RPM]
  u32 Odometer;           //!< Electrical rpms since last power on 
  u16 ControllerTemp;     //!< Controller temperature in deg. C 
  u16 MotorTemp;          //!< Motor      temperature in deg. C
  u16 BatteryTemp;        //!< Battery    Temperature in deg. C
  u16 WarningBits;        //!< 0x0001 = low voltage   0x0002=high current  0x0004=controller overheated  0x0008=battery overheated 0x0010=motor overheated
  u16 ErrorBits;          //!< 0x0001 = input signal out of range/not present
                          //!< 0x0002 = waiting for throttle idle position
                          //!< 0x0004 = power off request
                          //!< 0x0008 = memory error
                          //!< 0x0010 = power off request (settings change)
                          //!< 0x0020 = motor error (HAL sensors)
                          //!< 0x0040 = internal power source error
                          //!< 0x0080 = power off request. HAL position learning procedure completed sucessfully 
                          //!< 0x0100 = battery temperature sensor
                          //!< 0x0200 = motor temperature sensor
                          //!< 0x8000 = other HW error

} MotConParamBlock;       //!< mot controller data received via CAN bus

/************************************************************************\
*     BMS-2 BATTERY MANAGEMENT SYSTEM DATA STRUCTURE 
\************************************************************************/
//    BMS data are received via CAN bus

typedef struct
{                 
  u8  BatteryTotalCapacity;//!< Battery pack total capacity (0 .. 100 %)
  s16 Current;             //!< Current (-127 … +126, value * 5 = current in Ampere )
  u16 TotalVoltage;        //!< Total Voltage = Value/10 in Volts
  u8  HighestTemperature;  //!< Highest Temperature on the cells in deg.C
  u8  Notification;        //!< Bit 0 - reading, Bit 1 - waiting, Bit 2 - balanicing , Bit 3 - running, Bit 4 - low voltage, Bit 5 - error (not serious), Bit 6 - error (serious)
  u8  lastErrorAddress;    //!< The address of the CPU causing last error.
  u8  InternalErrorCode;   //!< Internal error code (only for service purposes)
  u16 LowestVoltage;       //!< Lowest voltage of the cell  = Value/100 in Volts
  u8  LowestVoltageAddr;   //!< The address of the cell with the lowest voltage
  u16 HighestVoltage;      //!< Highest voltage of the cell  = Value/100 in Volts
  u8  HighestVoltageAddr;  //!< The address of the cell with the highest voltage
  u8  LowestTemperature;   //!< Lowest temperature on the cells

} BMSParamBlock;           //!< BMS data block received via CAN bus

/************************************************************************\
*        GLOBAL VARIABLES
\************************************************************************/

extern CanRxMsg    CanRecMsg;                 //!< CAN Rx message structure
extern CanTxMsg    CanSendMsg;                //!< CAN Tx message structure

//extern MotConParamBlock   MotorParams;      //!< Motor Controlle Parameters
//extern BMSParamBlock     BatteryParams;     //!< Battery Parameters

/************************************************************************\
*    CAN related functions
\************************************************************************/

void CAN_UserInit(CAN_TypeDef* CAN_OpStruct, CAN_InitTypeDef* CAN_InitStruct);                 //!< init CAN controller (500kbit/s @ 36MHz).
void CAN_FillThrottlemsg(CAN_TypeDef* CAN_OpStruct, CanTxMsg* CanSendMsg, UINT32 throttle_cmd);//!< send throttle_cmd via CAN
void CAN_ReceiveMessage(CAN_TypeDef* CANx, CanRxMsg* CanReceiveMessage);                       //!< read CAN messages on CANx
void CAN_PrintMsgDetails(CanRxMsg* CanRecMsg);                                                 //!< print details of CAN message

void CAN_ReceiveMotorParameters(CanRxMsg* CanRecMsg, MotConParamBlock* MotorParam);  //!< read motcon data via CAN bus
void CAN_ReceiveBatteryParameters(CanRxMsg* CanRecMsg, BMSParamBlock* BatteryParam); //!< read BMS data that via CAN bus


/************************************************************************\
*    END OF MODULE CommMotConBMS.h
\************************************************************************/
