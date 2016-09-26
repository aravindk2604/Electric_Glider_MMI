/**
*  ***********************************************************************
*  @file    CANReceiveMotBMS.c
*  @author  B. Mysliwetz
*  @version V1.30
*
*  @brief   Main module of basic control/MMI unit, initializes peripherals,
*           uC/OS and user tasks.
*
*  TO BE ADAPTED
*  The 'StartupTask' creates two periodic user tasks that toggle two
*  pins of digital IO-port GPIO_B with different repeat rates.
*  The GPIO_B signals are accessible via connector pins PB8 and PB9.
*  A third user task (sieve) can optionally be activated via keyboard
*  input (key 's') on the terminal PC as an additional CPU load.
*  Period variations (jitter) under different CPU loads can thus be
*  analyzed by an oscilloscope or logic analyzer.
* 
*  Peripherals used:
*  - USART#1 or #2 for keyboard/terminal IO via RS-232/COM port
*    with 9600 Bd, 8N1 setting
*  - CAN1 for communication w HBC25063 motor controller and BMS-2
*    at 500 kBit/s and 29 bit ID field length
*  - ADC1 channel#1 or #14 for reading poti -> throttle_cmd
*
*  Changes:
*  15.01.2013 mys     Doxygen conformal headers and commenting style
*  24.02.2013 mys     Target independent version
*  17.03.2015 mys     Begin of streamlining and removal of R&P's bullsh..
*  18.03.2015 mys     ADC channel selection made target board specific
*  10.04.2015 mys     Smoothing of BatCurrent -> Ibat for terminal output
*  20.08.2015 mys     Comments streamlining & CanLib.x -> CommMotConBMS.x
*                      
*  @date    20/08/2015
**************************************************************************
*/

/************************************************************************\
*        VERSION & TARGET STRINGS
\************************************************************************/

#define     SW_VERSION "V1.31_150820"          //!< version number string
#define     HW_TARGET  "STM32F107"             //!< target MCU string

/************************************************************************\
*        CONDITIONAL COMPILATION FLAGS
\************************************************************************/

//#define PRINT_ADC_VALUES                     //!< enable debug output
//#define PRINT_MOTCON_VALUES                  //!< enable debug output 
//#define PRINT_BMS_VALUES                     //!< enable debug output 

/************************************************************************\
*        HEADERFILES
\************************************************************************/

#include    <stdio.h>                          //!< sprintf(), scanf() etc
// used with MCBSTM32 board & STM32F103 MCU & MDK 3.80 (?) 
//#include    <stm32f10x_lib.h>                //<!       OLD STUFF
//#include    <stm32f10x_can.h>                //<!       NOT USED ANYMORE !!
//#include    "STM32_Init.h"                   //!< STM32 peripherals init

// used with MCBSTM32C board & STM32F107 MCU & MDK 4.71
#include    <stm32f10x.h>

//#include  "platform_cfg.h"                   //!< as yet NOT USED HERE !!
#include    "bsp.h"                            //!< low level IO, clock & int related defs for uC/OS
#include    "..\BasicIO\BasicIO.h"             //!< mid level formatted IO

#include    "CommMotConBMS.h"                  //!< MotCon & BMS communication (via CAN)


/************************************************************************\
*        NUMERICAL CONSTANTS
\************************************************************************/

#define     K_LPF_IBAT        16               //!< Ibat    low pass filter const
#define     K_LPF_THROTTLECMD 16               //!< ThrtCmd low pass filter const

#define     AD_ON      0x00000001              //!< ADC start conversion bitmask
#define     ADC_EOC    0x00000002              //!< ADC end of conversion bitmask

#define     TASK_STK_SIZE     128              //!< task's stack size (in 16-bit WORDs)
#define     N_TASKS           3                //!< number of user/application tasks      
#define     USER_BASEPRIO     5                //!< priority of first application task
#define     STARTUPTASK_PRIO  USER_BASEPRIO-1  //!< priority of startup task

/************************************************************************\
*        GLOBAL VARIABLES
\************************************************************************/

OS_STK  UserTaskStk[N_TASKS][TASK_STK_SIZE];   //!< user task stacks
OS_STK  StartUpTaskStk[TASK_STK_SIZE];         //!< startup task stack
char    UserTaskData[N_TASKS+1];               //!< parameters for tasks

CAN_InitTypeDef    CAN_InitStruct;             //!< CAN operating mode, length of time quantum, xmit FIFO prio
CAN_TypeDef        CAN_OpStruct;               //!< CAN structure including mailboxes

MotConParamBlock   MotorParam;                 //!< Motor params eg. current, voltage, temperature...
BMSParamBlock      BatteryParam;               //!< BMS   params eg. Battery Capacity, Temperature, errors...

UINT16             Potivalue;                  //!< raw poti value from ADC channel 1
UINT32             ThrottleCmdLPF;             //!< smoothened throttle command value

/************************************************************************\
*    local functions
\************************************************************************/

void     Periph_Init(void);                    //!< init USART, GPIO & CAN 

void     adc_Init (void);                      //!< init ADC channel 1
UINT16   Get_Potvalue(void);                   //!< get raw poti value from ADC
void     adc_Printvalues(UINT16 Potvalue);     //!< print poti value in scale(0 to 100), (-1024 to 1024) and in hex.
 
void     StartUpTask(void *data);              //!< startup task initializes user tasks
void     UserTask_CANMsgSend(void *data);      //!< prototypes of user task for transmitting CAN message
void     UserTask_CANMsgRec(void *data);       //!< prototypes of user tasks for reeciving CAN message
void     UserTask_TerminalOutput(void *data);  //!< prototypes of user tasks for displaying the values on the terminal.
void     PrintOSerr( UINT8 uCOSerrNo, char *pFunctionName );

void     delay(UINT32 delaycount);             //!< simple runtime delay loop

/************************************************************************\
*        MAIN
\************************************************************************/

int main(void)
/**
*  ***********************************************************************
*  @brief  Initializes MCU peripherals, uCOS and creates StartUp Task.
*  @retval none : uC/OS applications NEVER return !
**************************************************************************
*/
{
  BYTE   err;                                  // µC/OS error return code

  BSP_Init( );                                 // basic UART, LED-GPIO initialization
  Periph_Init( );                              // app specific CAN, ADC initialization
  OSInit( );

  BSP_SetLED( 0x55 );                          // On/Off pattern aof the LEDs via GPIO-PB#8..PB#15

  //'clear' screen and print hello message
  print_string("\033[2J\033[;H");              // clear screen and go to home position          
  print_string( "CAN-ReceiveMotBMS  " SW_VERSION  );
  print_string( " on " HW_TARGET " Target using uC/OS V." );
  print_uint( OSVersion( ) ); 
      
  OSTaskCreate( StartUpTask, (void *)0,
   (void *) &StartUpTaskStk[TASK_STK_SIZE-1], STARTUPTASK_PRIO );

  OSTaskNameSet( STARTUPTASK_PRIO, "StartUpTask", &err );
   
  OSStart( );                                  // start multitasking

} // end of main()    


void StartUpTask ( void *data )
/**
*  ***********************************************************************
*  @brief  Initializes uC/OS system timer and creates all user tasks.
*  @param  data : pointer to optional input parameter block
*  @retval none
**************************************************************************
*/

{
  BYTE  err;                                   /// µC/OS error return code
  BYTE  iTask;                                 /// task number

  BSP_SetLED ( 0x55 );                         /// set initial LED on/off pattern via PB8..PB15 
//print_string( "\r\n\nHit any key to start uC/OS - multitasking ...\r\n\n\n" );
//_getch( );                                   /// wait for any key

  delay( 0xF00000 );                           //  time to show LED pattern

  BSP_SetLED ( 0x0 );                          /// clear initial LED pattern 
  OS_CPU_SysTickInit();                        /// set desired uC/OS tick rate according
                                               /// to parameter OS_TICKS_PER_SEC in os_cfg.h

  iTask = 1;
  OSTaskCreate( UserTask_CANMsgSend, "CANMsgSend", 
   (void *)&UserTaskStk[iTask-1][TASK_STK_SIZE-1], USER_BASEPRIO+iTask);
  OSTaskNameSet(USER_BASEPRIO+iTask, "CANMsgSend", &err);

  iTask = 2;
  OSTaskCreate( UserTask_CANMsgRec, "CANMsgRecv", 
   (void *)&UserTaskStk[iTask-1][TASK_STK_SIZE-1], USER_BASEPRIO+iTask);
  OSTaskNameSet(USER_BASEPRIO+iTask, "CANMsgRecv", &err);

  iTask = 3;
  OSTaskCreate(UserTask_TerminalOutput, "TerminalOut", 
   (void *)&UserTaskStk[iTask-1][TASK_STK_SIZE-1], USER_BASEPRIO+iTask);
  OSTaskNameSet(USER_BASEPRIO+iTask, "TerminalOut", &err);



  while (TRUE)                                 /// DO FOREVER loop, StartUpTask NEVER returns
  {
    _keyhit();   
    OSCtxSwCtr = 0;
    OSTimeDly(OS_TICKS_PER_SEC);               /// suspend for 1 sec,
  }                                            /// i.e. execution rate is 1 Hz

} // end of StartUpTask()  
      

/************************************************************************\
*        PERIODIC USER TASKS
\************************************************************************/

void UserTask_CANMsgSend (void *data)
/**
*  ***********************************************************************
*  @brief   LED#0 is switched ON and smoothened throttle command value is 
*           passed to function CAN_FillThrottlemsg() to xmit it via CAN bus.
*  @param  data : pointer to optional input parameter block (NOT USED HERE)
*  @retval none
***************************************************************************
*/
{
  BOOL on_off = FALSE;
                                    
  UINT16 ThrottleCmd1024;
  UINT16 ThrottleCmdOld = 0;

  while ( TRUE )                               // DO FOREVER loop, uC/OS task NEVER returns
  {
    if ( on_off )
    {
      LED_on( 0 );                              // toggle 1st LED for each cycle
      on_off = FALSE;
    }
    else
    {
      LED_off( 0 );
      on_off = TRUE;
    }

    Potivalue       = Get_Potvalue();          // raw poti position value [0 .. 4095]
    ThrottleCmd1024 = Potivalue / 4;
    ThrottleCmdLPF  = ((K_LPF_THROTTLECMD-1) * ThrottleCmdOld + ThrottleCmd1024) / K_LPF_THROTTLECMD;
    ThrottleCmdOld  = ThrottleCmdLPF;

#ifdef PRINT_ADValues
    {
    UINT16 PotiScale100;                       // poti value in the range of 0 to 100%
    UINT8 hi, lo;
    UINT8 k = 16;
    PotiScale100    = (Potivalue*100) / 4096;  // scale to % units

    print_string( "AD=" );                     // print scaled ADC values
    print_uint(Potivalue);
    print_string( " S100=" );
    print_uint(PotiScale100);

    print_string( "% S1024=" );
    print_int(ThrottleCmdLPF);

    print_string( " Hx32= " );                 // printing ThrottleCmdLPF in HEX
    print_hex32(ThrottleCmdLPF );

    hi= ThrottleCmdLPF >> 8 ;                  // hi byte in hex
    lo= ThrottleCmdLPF & 0x00ff ;              // lo byte in hex
    
    print_string( "  hi=" );
    print_hexbyte(hi);                         // print high byte
    print_string( "  lo= " );
    print_hexbyte(lo);                         // print low byte
    }
#endif

    CAN_FillThrottlemsg(CAN1, &CanSendMsg, ThrottleCmdLPF);  /// send throttle command to motor controller

    OSTimeDly( 20 );                           /// suspend for 100ms, i.e. execution rate is 10 Hz
  }

}  // end of UserTask_CANMsgSend()



void UserTask_CANMsgRec (void *data)
/**
*  ***********************************************************************
*  @brief  The messages that are sent by moto and Battery via CAN bus is received through functions
*          CAN_ReceiveMotorParameters, CAN_ReceiveMotorParameters respectively.
*  @param  data : pointer to optional input parameter block
*  @retval none
***************************************************************************
*/
{
  while(TRUE)
 {
 CAN_ReceiveMessage(CAN1, &CanRecMsg);                      /// read available MotorController - Msgs
 CAN_ReceiveMotorParameters(&CanRecMsg, &MotorParam);     ///Function that receives the motor data that are sent via CAN bus
 CAN_ReceiveBatteryParameters(&CanRecMsg, &BatteryParam); ///Function that receives the battery data that are sent via CAN bus
 OSTimeDly( 20 );                                          /// suspend for 100ms, i.e. execution rate is 10 Hz
 }
}  // end of UserTask_CANMsgRec()

    
void UserTask_TerminalOutput (void *data)
/**
*  ***********************************************************************
*  @brief  Print the received motor and battery parameters and the throttle command value on the screen
*  @param  data : pointer to optional input parameter block
*  @retval none
***************************************************************************
*/
{
//static UINT8 kLPF  = K_LPF_IBAT;
//static UINT8 kLPF1 = K_LPF_IBAT - 1;
   static INT16 IbatLPF;   
   static INT16 IbatLPFold = 0;   

  while(TRUE)
  { 
    print_string("\033[3;0HThrt\033[3;6HUbat\033[3;13HIbat\033[3;19HRPM\033[3;25HOdom\033[3;36HTcont\033[3;42HTbat\033[3;48HTmot\033[3;53HWarnBits\033[3;62HErrBits ");
    print_string("\033[4;0H                                                              ");
    print_string("\033[4;0H"); 
    print_int(ThrottleCmdLPF);                                         ///Printing of Throttle command value

    IbatLPF    = ((K_LPF_IBAT-1) * IbatLPFold + MotorParam.BatCurrent) / K_LPF_IBAT;
    IbatLPFold = IbatLPF;   

    print_string("\033[4;6H");  print_uint( MotorParam.BatVoltage );    
    print_string("\033[4;13H"); print_uint( IbatLPF) ;               // was MotorParam.BatCurrent          
    print_string("\033[4;19H"); print_uint( MotorParam.MotorRPM );       
    print_string("\033[4;25H"); print_hex32( MotorParam.Odometer );         
    print_string("\033[4;36H"); print_uint( MotorParam.ControllerTemp ); 
    print_string("\033[4;42H"); print_uint( MotorParam.BatteryTemp);   
    print_string("\033[4;48H"); print_uint( MotorParam.MotorTemp);       
    print_string("\033[4;53H"); print_hex16( MotorParam.WarningBits);   
    print_string("\033[4;62H"); print_hex16( MotorParam.ErrorBits);
   
    print_string ("\033[6;0HCapa\033[6;6HCurr\033[6;11HVolt\033[6;16HHighT\033[6;23HStatus\033[6;30HErrAdd\033[6;037HErrCod\033[6;44HLowV\033[6;50HAddr\033[6;56HHighV\033[6;62HAddr\033[6;68HLowT");
    print_string("\033[7;0H                                                                      "); 
    print_string("\033[7;0H");  print_uint( BatteryParam.BatteryTotalCapacity); 
    print_string("\033[7;6H");  print_int( BatteryParam.Current);                
    print_string("\033[7;11H");   print_uint( BatteryParam.TotalVoltage);         
    print_string("\033[7;16H"); print_uint( BatteryParam.HighestTemperature);      
    print_string("\033[7;23H"); print_hexbyte( BatteryParam.Notification);         
    print_string("\033[7;30H"); print_uint( BatteryParam.lastErrorAddress);     
    print_string("\033[7;37H"); print_hexbyte( BatteryParam.InternalErrorCode);     
    print_string("\033[7;44H"); print_uint( BatteryParam.LowestVoltage);         
    print_string("\033[7;50H"); print_uint( BatteryParam.LowestVoltageAddr);        
    print_string("\033[7;56H"); print_uint( BatteryParam.HighestVoltage);        
    print_string("\033[7;62H"); print_uint( BatteryParam.HighestVoltageAddr);    
    print_string("\033[7;68H"); print_uint( BatteryParam.LowestTemperature);        

    OSTimeDly( 20 );                                 //!< suspend for 250ms, i.e. execution rate is 4 Hz
  }
}// end of UserTask_TerminalOutput()
 
 
void Periph_Init(void)
/**
*  ***********************************************************************
*  @brief  CAN and ADC are initialized 
*  @retval none
***************************************************************************
*/
{
  CAN_UserInit(&CAN_OpStruct, &CAN_InitStruct); // init CAN controller
  adc_Init( );                                  // init ADC#1 for single channel
                                                // conversion
} // end of function ...


void adc_Init (void)
/**
*  ***********************************************************************
*  @brief  Initialize ADC channel 1.
*  @retval none
**************************************************************************
*/
{
#if defined USE_MCBSTM32C               // init for STM32F107xx
  
  RCC->APB2ENR |=  1 <<  9;             /* Enable ADC1 clock                  */
  GPIOC->CRL   &= 0xFFF0FFFF;           /* Configure PC4 as ADC.14 input      */
  ADC1->SQR1    = 0x00000000;           /* Regular channel 1 conversion       */
  ADC1->SQR2    = 0x00000000;           /* Clear register                     */
  ADC1->SQR3    = 14 <<  0;             /* SQ1 = channel 14                   */
  ADC1->SMPR1   =  5 << 12;             /* Channel 14 sample time is 55.5 cyc */
  ADC1->SMPR2   = 0x00000000;           /* Clear register                     */

//ADC1->CR1    &=  0xFFBFFFFF;          /* Scan mode off                      */      //add
  ADC1->CR1     =  1 <<  8;             /* Scan mode on                       */
  
  ADC1->CR2     = /*(1 << 20) |            Enable external trigger            */
                  (7 << 17) |           /* EXTSEL = SWSTART                   */
                  (1 <<  1) |           /* Continuous conversion              */
                  (1 <<  0) ;           /* ADC enable                         */

  ADC1->CR2    |=  1 <<  3;             /* Initialize calibration registers   */
  while (ADC1->CR2 & (1 << 3));         /* Wait for initialization to finish  */
  ADC1->CR2    |=  1 <<  2;             /* Start calibration                  */
  while (ADC1->CR2 & (1 << 2));         /* Wait for calibration to finish     */
  ADC1->CR2    |=  1 << 22;             /* Start first conversion           */
  ADC1->CR2    &=  0xFFFFFFFE;          /* Disable ADC conversion/calibration          //add
                                                     and go to power down mode */

#elif defined USE_MCBSTM32

//GPIOA->CRL &= ~0x0000000F;       // configure PA1 as ADC.1 analog input
                                   // see good old stm32_Init.c

  RCC->APB2ENR |= (1<<9);          // enable peripheral clock for ADC1

  ADC1->CR1   = 0x00000000;        // no interrupts, no watchdog enabled
  ADC1->CR2   = 0x000E0001;        // data right aligned, single conversion
                                   // no DMA, EXTSEL = SWSTART
  ADC1->SQR1  = 0x00000000;        // ONE conversion
  ADC1->SQR3  = (1<<0);            // select channel#1 in conversion sequence

#else
  #error: Unknown Target Board Selection !
#endif

}  // end of function adc_Init( )



UINT16 Get_Potvalue(void)   
/**  
*  ***********************************************************************
*  @brief  Getting the pot value from the ADC        
*  @retval Potvalue corresponding to Poti position in the range 0 to 4095
***************************************************************************
*/
{
  while (1)  // do forever
  {
   UINT16  Potvalue = 0xFFF;                 // ADC Value
    // Start ADC Conversion
   ADC1->CR2 = ADC1->CR2 B_OR AD_ON;         // start conversion v. ADON bit

    // wait till conversion is finished
   (ADC1->SR B_AND ADC_EOC) == 0;            // wait for end of conversion
   
   Potvalue = ADC1->DR;                      // Potvalue is obtained from ADC Channel#1
   return Potvalue;                         // Potvalue is returned
  } // end while
}   // end of function ...




void    delay(UINT32 delaycount)
/**
*  ***********************************************************************
*
*  @brief  Simple active waiting loop,
*          effective delay time depends on CPU type & clock frequency !
*
*  @param  delaycount : 32-Bit delay loop starting value
*  @retval none
**************************************************************************
*/
{
  while ( delaycount-- );                    // decrement until 0
}    // end of function ...


void PrintOSerr( UINT8 uCOSerrNo, char *pFunctionName )
/**
*  ***********************************************************************
*
*  @brief  Check uc/OS error code and output with name of error causing
*          function to terminal if <> 0
*
*  @param  uCOSerrNo : uC/OS error code
*  @param  pFunctionName : name of function in which error occurred 
*  @retval none
**************************************************************************
*/
{
  if ( uCOSerrNo != 0 )
  {
    print_string( "\n\n\rFunction " );
    print_string( pFunctionName );
    print_string( " returned errorcode: " );
    print_hexbyte( uCOSerrNo );
    print_string( "\n\n\r" );
    _getch( );
  }
} // end of function PrintOSerr( )

/**
  * @brief  Initialize Serial Interface(USART2).
  * @retval None.
*/

/************************************************************************\
*        END OF MODULE CANReceiveMotBMS.c
\************************************************************************/
