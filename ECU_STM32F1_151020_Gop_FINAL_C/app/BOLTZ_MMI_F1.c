/**
* ****************************************************************************************************************
*  @file    BOLTZ_MMI_F1.c
*  @author  Aravind Krishnan, Madhan Kumar Gopinath, EEIT-Master Project SS2015/WS2015
*  @version V2.00
*  @date    15.10.2015
*  @brief   Main BOLTZ_MMI_F1 module initializes peripherals and uC/OS, generates StartUpTask() and all user 
*           tasks, ie CANMsgSend(), CANMsgRec(), TerminalOutput(), SDwrite(), LCDOutput().
*
*  The BOLTZ_MMI Unit receives HBC25063 motorcontroller and BMS status information via CAN bus and writes
*  the data to a terminal (via USART = for basic debugging/monitoring) as well as to a QVGA LCD (via SPI =
*  operational graphics screen of cockpit instrument). Furthermore it reads the throttle knob/input setting and 
*  sends it to the motor controller via CAN bus.
*  The OS scheduler is set to 200 ticks per second.  
*  All received & sent values are logged to a file on SDcard (via SPI) every 1s (1 Hz rate).
*  There are two LEDs indications: one indicates the scheduling process of the RTOS by toggling the STATUS_LED 
*  whenever "UserTask_CANMsgSend" (task with the highest priority) is executed. 
*  A second LED indicates if the user pressed the alert button (as yet to simulate an emergency shut-off signal
*  e.g. to a circuit breaker to disconnect the battery).
*
*  Peripherals settings are done via Periph_Init() in module peripheral_init.c.
*  Hardware configuration parameters are provided in module platform_cfg.h.
*
*  Refresh rate for UserTasks
*  - StartUpTask              - 1s
*  - UserTask_CANMsgSend      - 0.1s
*  - UserTask_CANMsgRec       - 0.1s 
*  - UserTask_LCDOutput       - 0.5s
*  - UserTask_SDwrite         - 1s
*  - UserTask_TerminalOutput  - 1s
*
*  Main peripherals/interfaces & operational settings:
*  - Motor Controller:     CAN1,   500 kbps, extended 29-bit IDs
*  - Battery Managements:  CAN1,   500 kbps, extended 29-bit IDs
*  - Terminal:             USART2, 9600 bps, 8N1, no flowcontrol
*  - SD Card:              SPI1,   Prescaler 128 (from master clock), STM32F107 is master 
*  - LCD:                  SPI3,  CS = PB10, CLK = PC10, MISO  = PC11, MOSI = PC12
*  - Throttle              ADC1,  Port GPIOC Bit#4, channel 14, 12bit resolution, single conversion mode
*  - Alert-Button:         USER,  Port GPIOB Bit#7, as interrupt EXTI#7
*  - RTOS_STATUS_LED:      LED#1, Port GPIOE Bit#8
*  - SIM_SHUT_OFF_LED:     LED#3, Port GPIOE Bit#10
*  - SHUT_OFF_PIN:         Port GPIOB Bit#8
*
*
*  Changes: 15.10.2015 Fully tested & operational version V2.00.
*                      by Aravind & Madhan
*
****************************************************************************************************************** 
*/

/************************************************************************\
*                          VERSION & TARGET STRINGS
\************************************************************************/

#define     SW_VERSION "V151015"        //!< version number string !! 


/************************************************************************\
*        CONDITIONAL COMPILATION FLAGS
\************************************************************************/
             
#define      OS_STATISTICS_ON           //!< conditional compile flag

/************************************************************************\
*                               Headerfiles 
\************************************************************************/

#include    <stdio.h>                   //!< sprintf(), scanf() Prototype etc

#ifdef USE_DISCOVERY                    //!< STM32F4 board specific files (condition)
    #include   "stm32f4_discovery.h"    //!< definitions for STM32F4-Discovery Kit's Leds and push-button hardware resources
    #include   "stm32f4xx_conf.h"       //!< Periph. library configuration file
#elif defined USE_MCBSTM32C
    #include   "stm32f10x_lib.h"        //!< STM32 library
    #include   "stm32f10x_conf.h"       //!< Periph. library configuration file
#else
    #error: "Illegal target board selection!"
#endif

#include    "ff.h"                      //!< SD specific - File system 
#include    "platform_cfg.h"            //!< target selection & abstraction defs
#include    "Peripheral_Init.h"         //!< Initialization of all peripherals
#include    "CanDefs.h"                 //!< CAN related functions
#include    "AdcDefs.h"                 //!< ADC related functions 
#include    "LcdDefs.h"                 //!< LCD related functions 
#include    "BasicIO.h"                 //!< Collection of text/number-IO and LED-control function prototypes
#include    "bsp.h"                     //!< low level IO, clock & int related defs for uC/OS

                    
/************************************************************************\
*                            NUMERICAL CONSTANTS
\************************************************************************/

#define     TASK_STK_SIZE     128               //!< a task's stack size (in 16-bit WORDs)
#define     N_TASKS           5                 //!< number of user/application tasks      
#define     USER_BASEPRIO     5                 //!< priority of first application task
#define     STARTUPTASK_PRIO  USER_BASEPRIO-1   //!< priority of startup task
 

/************************************************************************\
*                            GLOBAL VARIABLES
\************************************************************************/

/* OS related */
OS_STK  UserTaskStk[N_TASKS][TASK_STK_SIZE];    //!< user task stacks
OS_STK  StartUpTaskStk[TASK_STK_SIZE];          //!< startup task stack
char    UserTaskData[N_TASKS+1];                //!< parameters for tasks

#ifdef OS_STATISTICS_ON
 UINT32 MaxIdleCount = 0;                       //!< values for CPU load comparison
 UINT32 myCPUUsage   = 1;                       //!< values for CPU load comparison
#endif

/* CAN Bus related */
CanDisplayMotorParams      MotorParam;          //!< Contains Motor Parameters like Current,Voltage, Temperature...
CanDisplayBatteryParams    BatteryParam;        //!< RT-BMS master parameters  like Battery Capacity, Temperature, errors...
extern LcdDisplayParams    LCDParam;            //!< Motor and BMS parameters that are to be displayed on the LCD Screen

UINT16                  Potivalue;              //!< raw potentiometer value received from ADC channel 1
UINT32                  ThrottleCmdNew;         //!< Smoothened throttle command value
UINT8                   ThrottleCmdNew_str[5];  //!< Array to store throttle command values

/* SD Card related */
FATFS FatFs;                                    //!< Fatfs object
FIL fil;                                        //!< File object
FRESULT res;                                    //!< Variable for error handling

/************************************************************************\
*                         LOCAL FUNCTION PROTOTYPES
\************************************************************************/
void     StartUpTask(void *data);               //!< prototype of startup task
void     UserTask_CANMsgSend(void *data);       //!< prototypes of user task for transmitting CAN message
void     UserTask_CANMsgRec(void *data);        //!< prototypes of user tasks for reeciving CAN message
void     UserTask_TerminalOutput(void *data);   //!< prototypes of user tasks for displaying the values on the terminal
void     UserTask_SDwrite (void *data);         //!< prototypes of user task for saving all data on SD drive
void     UserTask_LCDOutput(void *data);        //!< prototypes of user tasks for displaying the values on the LCD
void     PrintOSerr( UINT8 uCOSerrNo, char *pFunctionName );
void     EXTI0_IRQHandler(void);                //!< Interrupt handler for user push button
  

 
/************************************************************************\
*                                   MAIN
\************************************************************************/


int main(void)
/**
*  ***********************************************************************
*  @brief  Initializes MCU peripherals, uCOS and creates StartUp Task.
*  @retval none : uC/OS applications NEVER return !
**************************************************************************
*/
{
  BYTE   err;                                     // µC/OS error return code  
  Periph_Init( );                                 // Initialize CPU/MCU and application relevant peripherals     
  OSInit( );                                      // Initialize the uCOS RTOS
                                                  
    // 'clear' the terminal screen and print hello message
  print_string("\033[2J\033[;H");                              // clear screen and go to home position        
  print_string( "Battery Electric Drive Train  " SW_VERSION  );// Print SW version (defined by user)
  print_string( " on " HW_TARGET " Target using uC/OS V." );   // Print SW version (defined by user)
  print_uint( OSVersion( ) ); 


  OSTaskCreate( StartUpTask, (void *)0,
  (void *) &StartUpTaskStk[TASK_STK_SIZE-1], STARTUPTASK_PRIO );  // Create the startup task and start the multitasking of the OS

  OSTaskNameSet( STARTUPTASK_PRIO, (INT8U*)"StartUpTask", &err ); //Set name for startup task
  OSStart( );                                                     // start multitasking

} // end of main()    

/************************************************************************\
*                               USER TASKS
\************************************************************************/

void StartUpTask ( void *data )
/**
*  ***********************************************************************
*  @brief  Initializes uC/OS system timer and creates all user tasks.
*  @param  data : pointer to optional input parameter block
*  @retval none
**************************************************************************
*/
{
  BYTE  err;                          // µC/OS error return code
  BYTE  iTask;                        // task number

  OS_CPU_SysTickInit();               // set desired uC/OS tick rate according
                                      // to parameter OS_TICKS_PER_SEC in os_cfg.h
  
#ifdef OS_STATISTICS_ON
    OSStatInit();                     // measure OSIdleCtr with NO USER TASK running
    MaxIdleCount = OSIdleCtr;
#endif

  // Create user tasks
  iTask = 1;
  OSTaskCreate( UserTask_CANMsgSend, "UserTask_CANMsgSend", 
   (void *)&UserTaskStk[iTask-1][TASK_STK_SIZE-1], USER_BASEPRIO+iTask);
  OSTaskNameSet(USER_BASEPRIO+iTask, (INT8U*)"CANMsgSend", &err);

  iTask = 2;
  OSTaskCreate( UserTask_CANMsgRec, "UserTask_CANMsgRec", 
   (void *)&UserTaskStk[iTask-1][TASK_STK_SIZE-1], USER_BASEPRIO+iTask);
  OSTaskNameSet(USER_BASEPRIO+iTask, (INT8U*)"CANMsgRec", &err);

  iTask = 3;
  OSTaskCreate( UserTask_LCDOutput, "UserTask_LCDOutput", 
   (void *)&UserTaskStk[iTask-1][TASK_STK_SIZE-1], USER_BASEPRIO+iTask);
  OSTaskNameSet(USER_BASEPRIO+iTask, (INT8U*)"LCDOutput", &err);
  
  iTask = 4;
  OSTaskCreate( UserTask_SDwrite, "UserTask_SDwrite", 
   (void *)&UserTaskStk[iTask-1][TASK_STK_SIZE-1], USER_BASEPRIO+iTask);
  OSTaskNameSet(USER_BASEPRIO+iTask, (INT8U*)"SDwrite", &err);

  iTask = 5;
  OSTaskCreate(UserTask_TerminalOutput, "UserTask_TerminalOutput", 
   (void *)&UserTaskStk[iTask-1][TASK_STK_SIZE-1], USER_BASEPRIO+iTask);
  OSTaskNameSet(USER_BASEPRIO+iTask, (INT8U*)"TerminalOutput", &err);

  while (TRUE)                      // DO FOREVER loop, StartUpTask NEVER returns
  {
    #ifdef OS_STATISTICS_ON
    myCPUUsage = 100L - (UINT32)(OSIdleCtr * 100L) / MaxIdleCount;
    #endif  
    
    OSTimeDly(OS_TICKS_PER_SEC);    // suspend for 1 sec
  }                                             
}  

/************************************************************************\
*                          PERIODIC USER TASKS
\************************************************************************/

void UserTask_CANMsgSend (void *data)
/**
*  ***********************************************************************
*  @brief  LED is switched On and the smoothened throttle command value is 
*          sent to function CAN_FillThrottlemsg to send it via CAN bus at
*          a 10Hz rate.
*  @param  data : pointer to optional input parameter block
*  @retval none
***************************************************************************
*/
{
  BOOL on_off = FALSE;

//UINT16 PotiScale100=0;        // Pot value in the range of 0 to 100%
  UINT16 ThrottleCmd1024=0;     // temp new throttle value buffer   
  UINT16 ThrottleCmdOld = 0;    // temp old throttle value buffer  
  UINT8 k = 5;                  // LPF smoothing factor
	
  while ( TRUE )                // DO FOREVER loop, uC/OS task NEVER returns
  {
    if ( on_off )
    {
       LED_on(STATUS_LED);      // toggle STATUS_LED for each cycle (LED selected in platfrom_cfg.h)
       on_off = FALSE;
    }
    else
    {
       LED_off(STATUS_LED);
       on_off = TRUE;
    }

  Potivalue  = Get_Potvalue();                                       // raw poti position value [0 .. 4095]
  ThrottleCmd1024 = Potivalue / 4;                                   // scale to [0 .. 1023] range
  ThrottleCmdNew = ((k-1) * ThrottleCmdOld + ThrottleCmd1024) / k;   // low pass filter to smooth the throttlecmd value      
  ThrottleCmdOld = ThrottleCmdNew;

  #ifdef PRINT_ADValues 
    print_string( "AD=" );                                           // Printing Scaled ADC values
    print_uint(Potivalue);
    print_string( " S100=" );
    print_uint(PotiScale100);

    print_string( "% S1024=" );
    print_int(ThrottleCmdNew);

    print_string( " Hx32= " );                                       // Printing ThrottleCmdNew in HEX
    print_hex32(ThrottleCmdNew );

    hi= ThrottleCmdNew >> 8 ;                                        // hi byte in hex
    lo= ThrottleCmdNew & 0x00ff ;                                    // lo byte in hex

    print_string( "  hi=" );
    print_hexbyte(hi);                                               // Printing high byte
    print_string( "  lo= " );                 
    print_hexbyte(lo);                                               // Printing low byte  
  #endif
  
  uint2str(ThrottleCmdNew, ThrottleCmdNew_str, DISABLE);
  CAN_FillThrottlemsg(CAN_x, &CanSendMsg, ThrottleCmdNew);           // send throttle command to motor controller

  OSTimeDly( OS_TICKS_PER_SEC / 10 );                                // suspend for 100ms
  }
}                                                                    // end of UserTask_CANMsgSend()

void UserTask_CANMsgRec (void *data)
/**
*  ***********************************************************************
*  @brief  The messages that are sent by moto and Battery via CAN bus is 
*          received through functionsCAN_ReceiveMotorParameters, 
*		   CAN_ReceiveMotorParameters respectively.
*  @param  data : pointer to optional input parameter block
*  @retval none
***************************************************************************
*/
{
  while(TRUE)
 {
   CAN_ReceiveMessage(CAN_x, &CanRecMsg);                         // read available MotorController - Msgs
   CAN_ReceiveMotorParameters(&CanRecMsg, &MotorParam);           // function that receives the motor data that are sent via CAN bus
   CAN_ReceiveBatteryParameters(&CanRecMsg, &BatteryParam);       // function that receives the battery data that are sent via CAN bus
   OSTimeDly( OS_TICKS_PER_SEC / 10 );                            // suspend for 100ms
 }
}                                                                 // end of UserTask_CANMsgRec()

   
void UserTask_LCDOutput (void *data)
/**
*  ***********************************************************************
*  @brief  Print the received motor and battery parameters and the 
*          throttle command value on the LCD
*  @param  data : pointer to optional input parameter block
*  @retval none
***************************************************************************
*/
{ 
  while(TRUE)
  { 
     
    #ifdef USE_MCBSTM32C
      LCD_Display_Parameters ();                    // display units (on Discovery board done at startup)
    #endif 
    
    LCD_Display_Measured_Values();                  // display measured values          
    OSTimeDly( (OS_TICKS_PER_SEC / 2) - (110/5) );  // suspend for 500-110ms
  }
}                                                   // end of UserTask_LCDOutput()
 
void UserTask_SDwrite (void *data)
/**
*  ***********************************************************************
*  @brief  Write the received motor parameters, battery parameters and 
*          the throttle command value on the SD card
*  @param  data : pointer to optional input parameter block
*  @retval none
*  NOTE:  See SD folder for more usage details 
***************************************************************************
*/
{
  while(TRUE){
    
    if (f_mount(&FatFs, "", 1) == FR_OK) {                                       // If mounting the SD card was executed correctly
      res = f_open(&fil, SD_CARD_FILE_NAME, FA_OPEN_ALWAYS|FA_READ|FA_WRITE) ;   // Open file 'name', sets up GPIOS, Pins and Clocks for SPI
                                                                                 // Try to open  FA_OPEN_ALWAYS                                                                                      
      if (f_size(&fil) == 0) {                                                   // if empty create header
        f_puts("Throttle;Battery Voltage;Battery Current;Power;Motor RPM;Odometer;Temperature Controller;Battery Temperature;Motor Temperature;Warning Bits;Error Bits;Battery Total Capacity;Current;Total Voltage;Highest Temperature;Notification;Last Error Address;Internal Error Code;Lowest Voltage;Lowest Voltage Address;HighestVoltage;Highest Voltage Address;Lowest Temperature\n", &fil);;
      }

      if ((res) == FR_OK) {                                                      // test if file is functioning correctly
        f_lseek(&fil, f_size(&fil));                                             // replace write pointer to end of file in order to append new data to already existing data

        // Enter MOTOR data with CSV compatible attributes
        f_printf(&fil, "%d", ThrottleCmdNew);            f_puts(";", &fil);
        f_printf(&fil, "%s", LCDParam.BatVoltage_str);   f_puts(";", &fil);
        f_printf(&fil, "%d", MotorParam.BatCurrent);     f_puts(";", &fil);
        f_printf(&fil, "%s", LCDParam.Power_str);        f_puts(";", &fil);
        f_printf(&fil, "%d", MotorParam.MotorRPM);       f_puts(";", &fil);
        f_printf(&fil, "%d", MotorParam.Odometer);       f_puts(";", &fil);
        f_printf(&fil, "%d", MotorParam.ControllerTemp); f_puts(";", &fil);
        f_printf(&fil, "%d", MotorParam.BatteryTemp);    f_puts(";", &fil);
        f_printf(&fil, "%d", MotorParam.MotorTemp);      f_puts(";", &fil);
        f_printf(&fil, "%16LX", MotorParam.WarningBits); f_puts(";", &fil);
        f_printf(&fil, "%16LX",  MotorParam.ErrorBits);  f_puts(";", &fil);
        
        // Enter BATTERY data with CSV compatible attributes
        f_printf(&fil, "%d", BatteryParam.BatteryTotalCapacity); f_puts(";", &fil);
        f_printf(&fil, "%d", BatteryParam.Current);              f_puts(";", &fil);
        f_printf(&fil, "%s", LCDParam.BattTot_vol);              f_puts(";", &fil);
        f_printf(&fil, "%d", BatteryParam.HighestTemperature);   f_puts(";", &fil);
        f_printf(&fil, "%08LX", BatteryParam.Notification);      f_puts(";", &fil);
        f_printf(&fil, "%d", BatteryParam.lastErrorAddress);     f_puts(";", &fil);
        f_printf(&fil, "%08LX", BatteryParam.InternalErrorCode); f_puts(";", &fil);
        f_printf(&fil, "%s", LCDParam.BattLow_vol);              f_puts(";", &fil);
        f_printf(&fil, "%d", BatteryParam.LowestVoltageAddr);    f_puts(";", &fil);
        f_printf(&fil, "%s", LCDParam.BattHigh_vol);             f_puts(";", &fil);
        f_printf(&fil, "%d", BatteryParam.HighestVoltageAddr);   f_puts(";", &fil);
        f_printf(&fil, "%d", BatteryParam.LowestTemperature);    f_puts("\n", &fil);
        
        f_close(&fil);            // Close file, needed!
      }  
      f_mount(0, "", 1);          // Unmount drive, needed!
    }
    OSTimeDly(OS_TICKS_PER_SEC);  // suspend for 1 sec
  }
}                                 // end of UserTask_SDwrite()
 
void UserTask_TerminalOutput (void *data)
/**
*  ************************************************************************
*  @brief  Print the received motor and battery parameters and the throttle
*          command value on the screen terminal
*  @param  data : pointer to optional input parameter block
*  @retval none
***************************************************************************
*/
{
  while(TRUE)
  { 
    #ifdef OS_STATISTICS_ON
//    print_string( "\n\n\rCalibrating CPU's capacity" );
    print_string( "\033[9;00HMaxIdleCount = " );
    print_uint( MaxIdleCount );
    print_string("\033[11;00HIdleCount    = " );
//    print_string("\033[11;13H"); 
    print_uint( OSIdleCtr );
    print_string( "\033[11;27HOSCPU = " );
    print_uint( OSCPUUsage );
    print_string( "%" );
    print_string( "\033[11;40HmyCPU = " );
    print_uint( myCPUUsage );
    print_string( "% \n" );
    print_string( "\n\r" );
#endif  
    
    print_string("\033[3;0HThrt\033[3;6HUbat\033[3;13HIbat\033[3;19HRPM\033[3;25HOdom\033[3;36HTcont\033[3;42HTbat\033[3;48HTmot\033[3;53HWarnBits\033[3;62HErrBits\033[3;72HPower ");
    print_string("\033[4;0H                                                              ");
    print_string("\033[4;0H");  print_int(ThrottleCmdNew);           
    print_string("\033[4;6H");  print_string( LCDParam.BatVoltage_str);    
    print_string("\033[4;13H"); print_uint( MotorParam.BatCurrent);        
    print_string("\033[4;19H"); print_uint( MotorParam.MotorRPM);       
    print_string("\033[4;25H"); print_uint( MotorParam.Odometer);        
    print_string("\033[4;36H"); print_uint( MotorParam.ControllerTemp); 
    print_string("\033[4;42H"); print_uint( MotorParam.BatteryTemp);   
    print_string("\033[4;48H"); print_uint( MotorParam.MotorTemp);       
    print_string("\033[4;53H"); print_hex16( MotorParam.WarningBits);   
    print_string("\033[4;62H"); print_hex16( MotorParam.ErrorBits);
    print_string("\033[4;72H"); print_string(LCDParam.Power_str);
   
    print_string ("\033[6;0HCapa\033[6;6HCurr\033[6;11HVolt\033[6;16HHighT\033[6;23HStatus\033[6;30HErrAdd\033[6;37HErrCod\033[6;44HLowV\033[6;50HAddr\033[6;56HHighV\033[6;62HAddr\033[6;68HLowT");
    print_string("\033[7;0H                                                                      "); 
    print_string("\033[7;0H");  print_uint( BatteryParam.BatteryTotalCapacity); 
    print_string("\033[7;6H");  print_int( BatteryParam.Current);               
    print_string("\033[7;11H"); print_string( LCDParam.BattTot_vol);         
    print_string("\033[7;16H"); print_uint( BatteryParam.HighestTemperature);     
    print_string("\033[7;23H"); print_hexbyte( BatteryParam.Notification);         
    print_string("\033[7;30H"); print_uint( BatteryParam.lastErrorAddress);     
    print_string("\033[7;37H"); print_hexbyte( BatteryParam.InternalErrorCode);     
    print_string("\033[7;44H"); print_string( LCDParam.BattLow_vol);         
    print_string("\033[7;50H"); print_uint( BatteryParam.LowestVoltageAddr);       
    print_string("\033[7;56H"); print_string( LCDParam.BattHigh_vol);        
    print_string("\033[7;62H"); print_uint( BatteryParam.HighestVoltageAddr);    
    print_string("\033[7;68H"); print_uint( BatteryParam.LowestTemperature);       

    OSTimeDly( OS_TICKS_PER_SEC - (740/5) );  // suspend for 1000-720 msec,
  }
}                                             // end of UserTask_TerminalOutput()

/************************************************************************\
*                 Various Functions
\************************************************************************/


void PrintOSerr( UINT8 uCOSerrNo, char *pFunctionName )
/**
*  ***********************************************************************
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
}  // end of function 


/************************************************************************\
*                 Interrupt Handlers
\************************************************************************/
#ifdef USE_DISCOVERY            //!< STM32F4 board handler for EXTI_Line0
    void EXTI0_IRQHandler(void)
    /**
    *  ***********************************************************************
    *  @brief  Exterior hardware interrupt handler for user input push Alert button
    *  @param  none
    *  @retval none
    **************************************************************************
    */
    {
      if(EXTI_GetITStatus(UI_BTN_EXTI_LINE) != RESET) // test if external interrupt was triggered
      {              
         Set_GPIO_on(UIx_GPIO, UIx_GPIO_pin);         // Set Alert State to HIGH 
         LED_on(UI_LED);                              // LED3                          
         EXTI_ClearITPendingBit(UI_BTN_EXTI_LINE);    // Clear the EXTI line 0 pending bit
      }
    }                                                 // end of EXTI0_IRQHandler()
    
#elif defined USE_MCBSTM32C                           //!< STM32F1 board handler for EXTI_Line7
    void EXTI9_5_IRQHandler(void)
    /**
    *  ***********************************************************************
    *  @brief  Exterior hardware interrupt handler for user input push Alert button
    *  @param  none
    *  @retval none
    **************************************************************************
    */
    {
      if(EXTI_GetITStatus(UI_BTN_EXTI_LINE) != RESET) // test if external interrupt was triggered
      {              
         Set_GPIO_on(UIx_GPIO, UIx_GPIO_pin);         // Set Alert State to HIGH 
         LED_on(UI_LED);                              // LED3                          
         EXTI_ClearITPendingBit(UI_BTN_EXTI_LINE);    // Clear the EXTI line 7 pending bit
      }
    }                                                 // end of EXTI9_5_IRQHandler()
#else
    #error: "Illegal target board selection!"
#endif







