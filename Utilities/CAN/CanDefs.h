/**
*  ***********************************************************************
*  @file    CanDefs.h
*  @author  Madhan and Aravind
*  @version V2.00
*  @brief   Can bus related structures defines and function prototypes. 
*           
*
*  Changes: 25.09.2014 Janezic Sommerauer: File adaptation for STM32F4 Board
*           08.02.2015 Janezic Sommerauer: Added LCD Debug option 
*           16.06.2015 Aravind & Madhan  : Used float to string function for conversion of power and voltage values of the battery
*  @date    27/09/2015
**************************************************************************
*/


/************************************************************************\
*    Headerfiles 
\************************************************************************/
#ifdef USE_DISCOVERY
#include    <stm32f4xx_can.h>
#include    <stm32f4xx_conf.h>
#elif defined USE_MCBSTM32C
#include    <stm32f10x_can.h>
#else
#error: "Illegal target board selection!"
#endif

#include    "BasicIO.h" 


/************************************************************************\
*    Defines 
\************************************************************************/
# define MAXDigits 5                           //!< maximum digits of the String
# define MAXThrottle 1024                      //!< Throttle command maximum value 

#ifndef LCDDEBUG
#undef LCDDEBUG                                //!< Debug option do display dummy values on LCD w/o Canbus connection 
#endif


/************************************************************************\
*    MOTOR DEFINITIONS 
\************************************************************************/
typedef struct                          //!< Motor parameters sent via CAN bus    
{
  float BatVoltage;                   //!< Battery Voltage in volts          
  u32 BatCurrent;                     //!< Battery Current in Ampere
  float Power;                        //!< Power in Watts
  u32 MotorRPM;                       //!< Motor speed in rpm
  u32 Odometer;                       //!< Electrical rpms since last power on 
  u16 ControllerTemp;                 //!< Controller Temperature in deg.C 
  u16 MotorTemp;                      //!< Motor Temperature in deg.C
  u16 BatteryTemp;                    //!< Battery Temperature in deg.C
  u16 WarningBits;                    //!< 0x0001 =low voltage   0x0002=high current  0x0004=controller overheated  0x0008=battery overheated 0x0010=motor overheated
  u16 ErrorBits;                      //!< 0x0001 = input signal out of range/not present
  //!< 0x0002 = waiting for throttle idle position
  //!< 0x0004 = power off request
  //!< 0x0008 = memory error
  //!< 0x0010 = power off request (settings change)
  //!< 0x0020 = motor error (HAL sensors)
  //!< 0x0040 = internal power source error
  //!< 0x0080 = power off request . HAL position learning procedure completed sucessfully 
  //!< 0x0100 = battery temperature sensor
  //!< 0x0200 = motor temperature sensor
  //!< 0x8000 = other HW error 
}CanDisplayMotorParams;              //!< Motor parameters sent via CAN bus

/************************************************************************\
*   BATTERY DEFINITIONS 
\************************************************************************/
typedef struct                       //!< Battery parameters sent via CAN bus
{                 
  u8  BatteryTotalCapacity;          //!< Battery pack total capacity (0 .. 100 %)
  s16 Current;                       //!< Current (-127 … +126, value * 5 = current in Ampere )
  u16 TotalVoltage;                  //!< Total Voltage = Value/10 in Volts
  u8  HighestTemperature;            //!< Highest Temperature on the cells in deg.C
  u8  Notification;                  //!< Bit 0 - reading, Bit 1 - waiting, Bit 2 - balanicing , Bit 3 - running, Bit 4 - low voltage, Bit 5 - error (not serious), Bit 6 - error (serious)
  u8  lastErrorAddress;              //!< The address of the CPU causing last error.
  u8  InternalErrorCode;             //!< Internal error code (only for service purposes)
  u16 LowestVoltage;                 //!< Lowest voltage of the cell  = Value/100 in Volts
  u8  LowestVoltageAddr;             //!< The address of the cell with the lowest voltage
  u16 HighestVoltage;                //!< Highest voltage of the cell  = Value/100 in Volts
  u8  HighestVoltageAddr;            //!< The address of the cell with the highest voltage
  u8  LowestTemperature;             //!< Lowest temperature on the cells
  
}CanDisplayBatteryParams;            //!< Battery parameters sent via CAN bus


/************************************************************************\
*   LCD DISPLAY PARAMETERS 
\************************************************************************/
typedef struct                              //!< Parameters that are to be displayed on the LCD screen
{
  float Throttle_Status;                    //!< Throttle status in % (values between 0..1)
  float BatChargeCap;                       //!< Current Battery Charge (0 .. 100 %)
  UINT8 batchargecap_str[MAXDigits+1];      //!< newly added for battery percentage   
  char  BatVoltage_str[MAXDigits+1];        //!< Battery Voltage (String) in volts      
  UINT8 BatCurrent_str[MAXDigits+1];        //!< Battery Current(String) in Ampere
  UINT8 RPM_str[MAXDigits+1];               //!< Motor speed (String) in rpm
  char  Power_str[MAXDigits+2];             //!< Power (String) in watts
  UINT8 TMotor_str[MAXDigits+1];            //!< Motor Temperature String in deg.C
  UINT8 TBattHigh_str[MAXDigits+1];         //!< Highest Temperature on the cells in deg.C (String)
  char  TBattLow_str[MAXDigits+1];          //!< Lowest temperature on the cells(String)
  UINT8 TCont_str[MAXDigits+1];             //!< Controller Temperature (String)in deg.C
  char  TBattLVA_str[MAXDigits+1];          //!< Adress of Cell having lowest temperature(String)
  char  TBattHVA_str[MAXDigits+1];          //!< Adress of Cell having highest temperature(String)
  char  BattTot_vol[MAXDigits];             //!< Array for Total Voltage of Battery
  char  BattLow_vol[MAXDigits];             //!< Array for Low Voltage of Battery
  char  BattHigh_vol[MAXDigits];            //!< Array for High Voltage of Battery
}LcdDisplayParams;                          //!< parameters that are to be displayed on the LCD screen


/************************************************************************\
*        GLOBAL VARIABLES
\************************************************************************/
extern CanRxMsg    CanRecMsg;                 //!< CAN Rx message structure
extern CanTxMsg    CanSendMsg;                //!< CAN Tx message structure

extern CanDisplayMotorParams   MotorParamsCondition;   //!< Newly Addded for color display
extern CanDisplayBatteryParams BatteryParamsCondition; //!< Newly Addded for color display

/************************************************************************\
*    CAN related functions
\************************************************************************/
void CAN_UserInit(CAN_TypeDef* CAN_OpStruct, CAN_InitTypeDef* CAN_InitStruct);                 //!< Initialize CAN controller (500kbit/s @ 36MHz).
void CAN_FillThrottlemsg(CAN_TypeDef* CAN_OpStruct , CanTxMsg* CanSendMsg, UINT32 adc_value);  //!< Transmit Scaled ADC value via CAN
void CAN_ReceiveMessage(CAN_TypeDef* CANx_used, CanRxMsg* CanReceiveMessage);                  //!< Receive CAN messages on CANx
void CAN_PrintMsgDetails(CanRxMsg* CanRecMsg);                                                 //!< Print details of a CAN message.  
void CAN_ReceiveMotorParameters(CanRxMsg* CanRecMsg, CanDisplayMotorParams* MotorParam);       //!< Function that receives the motor data that are sent via CAN bus.
void CAN_ReceiveBatteryParameters(CanRxMsg* CanRecMsg, CanDisplayBatteryParams* BatteryParam); //!< Function that receives the battery data that are sent via CAN bus.
void Initialize_LCDStructure(void);                                                            //!< Function that initializes the LCDParam structures with zeros
