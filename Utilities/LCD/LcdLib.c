/**
*  ***********************************************************************
* @file      LcdLib.c
* @author    Madhan and Aravind
* @version   V2.00
* @brief     Basic LCD functions for MCBSTM32C  
*  
*  Changes:  25.09.2014 Janezic Sommerauer File adaptation for STM32F4 Board
*            08.12.2014 Janezic Sommerauer Battery and Throttle drawings
*            DISCOVERY BOARD CHANGES
*            16.06.2015 Aravind & Madhan:  Included numerical constants as 
*                       boundary values for the Motor and BMS parameters 
*                       displayed          
*            16.06.2015 Aravind & Madhan:  Colour differences introduced for 
*                       boundary values of the Motor and BMS parameters
*            STM32F107 BOARD CHANGES
*            16.06.2015 Aravind & Madhan:  Included numerical constants as 
*                       boundary values for the Motor and BMS parameters 
*                       displayed
*            02.09.2015 Aravind & Madhan:  Colour differences introduced for 
*                       boundary values of the Motor and BMS parameters
* @date      27/09/2015
**************************************************************************
*/

/************************************************************************\
*        Includes
\************************************************************************/
#include "LcdDefs.h"
#include "AdcDefs.h"
#include "CanDefs.h"
#include "BasicIO.h"
#include "platform_cfg.h"                //!< Definitions of pis

/************************************************************************\
*                            NUMERICAL CONSTANTS
\************************************************************************/

#define MAX_VAL_MOTOR_RPM       100
#define MIN_VAL_MOTOR_RPM       10

#define MAX_VAL_MOTOR_TEMP      100
#define MIN_VAL_MOTOR_TEMP      10

#define MAX_VAL_CONTR_TEMP      100
#define MIN_VAL_CONTR_TEMP      10

#define MAX_VAL_BATT_HIGH_TEMP  100
#define MIN_VAL_BATT_HIGH_TEMP  10

#define MAX_VAL_BATT_LOW_TEMP   100
#define MIN_VAL_BATT_LOW_TEMP   10

#define MAX_VAL_BATT_VOLT       100
#define MIN_VAL_BATT_VOLT       10

#define MAX_VAL_BATT_CURR       100
#define MIN_VAL_BATT_CURR       10


/************************************************************************\
*                            GLOBAL VARIABLES
\************************************************************************/

/* CAN Bus related */
extern CanDisplayMotorParams        MotorParam;             //!< Contains Motor Parameters like Current,Voltage, Temperature...
extern CanDisplayBatteryParams      BatteryParam;           //!< RT-BMS master parameters  like Battery Capacity, Temperature, errors...
extern LcdDisplayParams             LCDParam;               //!< Motor and BMS parameters that are to be displayed on the LCD Screen
extern CanDisplayMotorParams        MotorParamsCondition;   //!< Motor structure for color display
extern CanDisplayBatteryParams      BatteryParamsCondition; //!< Battery structure for color display

/************************************************************************\
*        LOCAL VARIABLES
\************************************************************************/
#define     SW_VERSION "V151120"          //!< version number string !!

UINT8 LCD_state;                          //!< So the function knows what page it is by reading this var.
uint8_t LCD_Update_FLAG = 0;              //!< KEY FLAG FOR LCD DISPLAY. 0 means do not need update

extern  LcdDisplayParams    LCDParam;     //!< Structure for Motor and BMS parameters to be displayed on the LCD Screen

/************************************************************************\
*        LOCAL Functions
\************************************************************************/

/**
*  ***********************************************************************
* @brief  Functions for init and output of value on visual element
* @retval None.
**************************************************************************
*/
void     LCD_Init(void)
{

#if defined USE_MCBSTM32C
  GLCD_Init();                              // Initialize the LCD               
  GLCD_Clear(BACKGROUND);                   // Clear the LCD                    
#elif defined USE_DISCOVERY
  STM32f4_Discovery_LCD_Init();             // Initialize the LCD
  LCD_Display_Parameters();                 // Display parameters/units
#else
#error: "Illegal target board selection!"
#endif
}

/**
*  ***********************************************************************
* @brief  Display Battery and Motor parameters text.
* @retval None.
**************************************************************************
*/
void LCD_Display_Parameters (void)
{ 
#ifdef USE_MCBSTM32C  
  wr_LCD_state(State_Display_Screen); 
  GLCD_DisplayStringWithColor(0, 23, 0, (UINT8*)SW_VERSION, BACKGROUND, TEXT);     
  GLCD_DisplayStringWithColor(0, 0,  1, (UINT8*)"RPM"  , BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(0, 17, 1, (UINT8*)"PWR"  , BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(1, 18, 1, (UINT8*)"kW"  , BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(8, 40, 0, (UINT8*)"Throttle", BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(8, 8,  1, (UINT8*)"Tmot", BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(9, 11, 1, (UINT8*)"C", BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(5, 0,  1, (UINT8*)"TBatL", BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(6, 4,  1, (UINT8*)"C", BACKGROUND, TEXT);  
  GLCD_DisplayStringWithColor(5, 7, 1, (UINT8*)"TBatH", BACKGROUND, TEXT);  
  GLCD_DisplayStringWithColor(6, 11, 1, (UINT8*)"C", BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(5, 15, 1, (UINT8*)"Tcont", BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(6, 19, 1, (UINT8*)"C", BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(8, 0,  1, (UINT8*)"Batt"  , BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(9, 4,  1, (UINT8*)"V"  , BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(8, 16, 1, (UINT8*)"Curr"  , BACKGROUND, TEXT);
  GLCD_DisplayStringWithColor(9, 19, 1, (UINT8*)"A"  , BACKGROUND, TEXT);

  GLCD_BargraphWithCorlorLineX (0, 80, 220, 20, ThrottleCmdNew);

  LCD_Update_FLAG = ENABLE;   

#elif defined USE_DISCOVERY 

  /* software version display */
  LCD_SetFont(&LCD_SM_SWV_FONT);
  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Col_Line_Align(0,0, LEFT, SW_VERSION);

  /* Motor units */                                   
  LCD_SetFont(&LCD_BIG_UNIT_FONT);
  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Col_Line_Align(7,1, LEFT, "RPM");

  LCD_SetColors(WHITE,BLACK);                           
  LCD_DisplayString_Col_Line_Align(20,1, LEFT, "PWR");
  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Col_Line_Align(20,2, LEFT, "kW");

  /* Temperature units */
  LCD_SetFont(&LCD_SM_UNIT_FONT);
  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Col_Line_Align(3,6, LEFT, "Tmot");

  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Col_Line_Align(10,6, LEFT, "Tcont");

  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Col_Line_Align(17, 6, LEFT, "TbatH"); 

  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Col_Line_Align(24, 6, LEFT, "TbatL"); 

  /* Battery units */
  LCD_SetFont(&LCD_BIG_UNIT_FONT);
  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Col_Line_Align(8, 8, LEFT,"BATT");
  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Col_Line_Align(8, 9, LEFT, "V");

  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Col_Line_Align(18, 8, LEFT,"CURR");
  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Col_Line_Align(18, 9, LEFT, "A");

  /* Set the font properties for data */
  LCD_SetColors(WHITE,BLACK);

  /* Draw speed meter frame*/
  LCD_SetColors(WHITE,BLACK);
  LCD_DrawRect(DRAW_THROT_BASE1_X, DRAW_THROT_BASE1_Y, DRAW_THROT_LENGTH, DRAW_THROT_HEIGHT);

  /* Draw large battery*/
  LCD_SetColors(WHITE,BLACK);
  LCD_DrawRect(DRAW_BAT_BASE1_X, DRAW_BAT_BASE1_Y, DRAW_BAT_LENGTH, DRAW_BAT_HEIGHT);
  LCD_DrawRect(DRAW_BAT_BASE2_X, DRAW_BAT_BASE2_Y, DRAW_BAT_TIP_LENGTH, DRAW_BAT_TIP_HEIGHT);
  LCD_SetColors(BLACK,BLACK);
  LCD_DrawLine(DRAW_BAT_BASE2_X,DRAW_BAT_BASE2_Y+1,DRAW_BAT_TIP_HEIGHT-2,LCD_DIR_VERTICAL);
#else
#endif  

}

/**
*  ********************************************************************************
* @brief  Check LCD_Update_FLAG whether the information needs to be refreshed and 
Display Battery and Motor parameters .
* @retval None.
************************************************************************************
*/
void LCD_Display_Measured_Values(void)
{
#ifdef USE_MCBSTM32C
  if (get_LCD_state() == State_Display_Screen && LCD_Update_FLAG)
  {
    if(MotorParamsCondition.MotorRPM < MAX_VAL_MOTOR_RPM && MotorParamsCondition.MotorRPM > MIN_VAL_MOTOR_RPM )
    {  
      GLCD_DisplayStringWithColor(1, 0,  1, (UINT8*)"   " , BACKGROUND, TEXT);              // RPM value
      GLCD_DisplayStringWithColor(1, 0,  1, LCDParam.RPM_str , BACKGROUND, TEXT);           // RPM value
    } 
    else
    {  
      GLCD_DisplayStringWithColor(1, 0,  1, (UINT8*)"   " , BACKGROUND, TEXT_HEAD);         // RPM value
      GLCD_DisplayStringWithColor(1, 0,  1, LCDParam.RPM_str , BACKGROUND, TEXT_HEAD);      // RPM value
    }  
    GLCD_DisplayStringWithColor(1, 14, 1, (UINT8*) "   " ,  BACKGROUND, TEXT);              // Refresh
    GLCD_DisplayStringWithColor(1, 14, 1, (UINT8*)LCDParam.Power_str , BACKGROUND, TEXT);   // Power in Watts
    GLCD_DisplayStringWithColor(3, 15, 1, (UINT8*)"     ", BACKGROUND, TEXT);               // Refresh
    GLCD_DisplayStringWithColor(3, 15, 1, ThrottleCmdNew_str, BACKGROUND, TEXT);            // Smoothened Throttle cmd value
    if(MotorParamsCondition.MotorTemp < MAX_VAL_MOTOR_TEMP && MotorParamsCondition.MotorTemp > MIN_VAL_MOTOR_TEMP )
    {
      GLCD_DisplayStringWithColor(9, 8,  1, (UINT8*)"  " ,   BACKGROUND, TEXT);             // Refresh
      GLCD_DisplayStringWithColor(9, 8,  1, LCDParam.TMotor_str, BACKGROUND, TEXT);           // Motor Temp in °C
    }
    else
    {  
      GLCD_DisplayStringWithColor(9, 8,  1, (UINT8*)"  " ,   BACKGROUND, TEXT_HEAD);        // Refresh
      GLCD_DisplayStringWithColor(9, 8,  1, LCDParam.TMotor_str, BACKGROUND, TEXT_HEAD);    // Motor Temp in °C
    } 
    if(BatteryParamsCondition.LowestTemperature < MAX_VAL_BATT_LOW_TEMP && BatteryParamsCondition.LowestTemperature > MIN_VAL_BATT_LOW_TEMP)
    {
      GLCD_DisplayStringWithColor(6, 0,  1, (UINT8*)"  " ,   BACKGROUND, TEXT);             // Refresh
      GLCD_DisplayStringWithColor(6, 0,  1, (UINT8*)LCDParam.TBattLow_str, BACKGROUND, TEXT);       // Lowest Battery Temp in °C
    }
    else
    {
      GLCD_DisplayStringWithColor(6, 0,  1, (UINT8*)"  " ,   BACKGROUND, TEXT_HEAD);        // Refresh
      GLCD_DisplayStringWithColor(6, 0,  1, (UINT8*)LCDParam.TBattLow_str, BACKGROUND, TEXT_HEAD);  // Lowest Battery Temp in °C  
    }
    if(BatteryParamsCondition.HighestTemperature < MAX_VAL_BATT_HIGH_TEMP && BatteryParamsCondition.HighestTemperature > MIN_VAL_BATT_HIGH_TEMP )
    {
      GLCD_DisplayStringWithColor(6, 7, 1, (UINT8*)"  " ,   BACKGROUND, TEXT);              // Refresh
      GLCD_DisplayStringWithColor(6, 7, 1, (UINT8*)LCDParam.TBattHigh_str, BACKGROUND, TEXT);       // Highest Battery Temp in °C
    }
    else
    { 
      GLCD_DisplayStringWithColor(6, 7, 1, (UINT8*)"  " ,   BACKGROUND, TEXT_HEAD);         // Refresh
      GLCD_DisplayStringWithColor(6, 7, 1, (UINT8*)LCDParam.TBattHigh_str, BACKGROUND, TEXT_HEAD);  // Highest Battery Temp in °C          
    }
    if(MotorParamsCondition.ControllerTemp < MAX_VAL_CONTR_TEMP && MotorParamsCondition.ControllerTemp > MIN_VAL_CONTR_TEMP ) 
    {
      GLCD_DisplayStringWithColor(6, 15, 1, (UINT8*)"  " ,   BACKGROUND, TEXT);             // Refresh
      GLCD_DisplayStringWithColor(6, 15, 1, LCDParam.TCont_str, BACKGROUND, TEXT);          // Controller Temp in °C
    }
    else 
    {
      GLCD_DisplayStringWithColor(6, 15, 1, (UINT8*)"  " ,   BACKGROUND, TEXT_HEAD);        // Refresh
      GLCD_DisplayStringWithColor(6, 15, 1, LCDParam.TCont_str, BACKGROUND, TEXT_HEAD);     // Controller Temp in °C
    } 
    if(MotorParamsCondition.BatVoltage < MAX_VAL_BATT_VOLT && MotorParamsCondition.BatVoltage > MIN_VAL_BATT_VOLT )
    {
      GLCD_DisplayStringWithColor(9, 0 , 1, (UINT8*)"  " ,  BACKGROUND, TEXT);                    // Refresh
      GLCD_DisplayStringWithColor(9, 0 , 1, (UINT8*)LCDParam.BatVoltage_str, BACKGROUND, TEXT);   // Battery Voltage in V   
    }
    else
    {
      GLCD_DisplayStringWithColor(9, 0 , 1, (UINT8*)"  " ,  BACKGROUND, TEXT_HEAD);                    // Refresh
      GLCD_DisplayStringWithColor(9, 0 , 1, (UINT8*)LCDParam.BatVoltage_str, BACKGROUND, TEXT_HEAD);   // Battery Voltage in V 
    }
    if(MotorParamsCondition.BatCurrent < MAX_VAL_BATT_CURR && MotorParamsCondition.BatCurrent > MIN_VAL_BATT_CURR)
    {
      GLCD_DisplayStringWithColor(9, 16, 1, (UINT8*)"   ",   BACKGROUND, TEXT);               // Refresh
      GLCD_DisplayStringWithColor(9, 16, 1, LCDParam.BatCurrent_str , BACKGROUND, TEXT);      // Battery Current in A
    } 
    else
    {
      GLCD_DisplayStringWithColor(9, 16, 1, (UINT8*)"   ",   BACKGROUND, TEXT_HEAD);          // Refresh
      GLCD_DisplayStringWithColor(9, 16, 1, LCDParam.BatCurrent_str , BACKGROUND, TEXT_HEAD); // Battery Current in A
    }  
  }

#elif defined USE_DISCOVERY

  static float charge_state_hist;
  static float throttle_state_hist;

  /* Check if received data is valid */  
  if(LCDParam.BatChargeCap>1)LCDParam.BatChargeCap=1;

  /* Draw throttle status if value has changed */
  if(throttle_state_hist!=LCDParam.Throttle_Status)
  {
    /* check if value is lower, -> clear pixel that that are highlighted too much */
    if(throttle_state_hist>LCDParam.Throttle_Status)
    {
      LCD_SetColors(BLACK,BLACK);
      LCD_DrawFullRect(DRAW_THROT_BASE1_X+2+(DRAW_THROT_LENGTH-3)*LCDParam.Throttle_Status,
      DRAW_THROT_BASE1_Y+2,
      (DRAW_THROT_LENGTH-3)*(1-LCDParam.Throttle_Status),
      DRAW_THROT_HEIGHT-5);
    }

    /* Set new value as history */
    throttle_state_hist=LCDParam.Throttle_Status;

    /* Highlight pixel that represent new value */
    if(LCDParam.Throttle_Status<(float)DRAW_THROT_MDLEVEL)
    { 
      LCD_SetColors(GREEN,GREEN);
    }
    else if(LCDParam.Throttle_Status>(float)DRAW_THROT_MDLEVEL & LCDParam.Throttle_Status<(float)DRAW_THROT_HLEVEL)  
    {
      LCD_SetColors(YELLOW,YELLOW);
    }
    else LCD_SetColors(RED,RED);
    LCD_DrawFullRect(DRAW_THROT_BASE1_X+2,DRAW_THROT_BASE1_Y+2,(DRAW_THROT_LENGTH-4)*LCDParam.Throttle_Status,DRAW_THROT_HEIGHT-5);
  }

  /* Draw battery status if value has changed */
  if(charge_state_hist!=LCDParam.BatChargeCap)
  {
    /* check if charge is lower, -> clear pixel that that are highlighted to much */
    if(charge_state_hist>LCDParam.BatChargeCap)
    {
      LCD_SetColors(BLACK,BLACK);
      LCD_DrawFullRect(DRAW_BAT_BASE1_X+2+(DRAW_BAT_LENGTH-3)*LCDParam.BatChargeCap,
      DRAW_BAT_BASE1_Y+2,
      (DRAW_BAT_LENGTH-3)*(1-LCDParam.BatChargeCap),
      DRAW_BAT_HEIGHT-5);
    }
    /* Set new value as history */
    charge_state_hist=LCDParam.BatChargeCap;
    /* Highlight pixel that represent new value */
    if(LCDParam.BatChargeCap<(float)DRAW_BAT_LOWLEVEL)  
    {
      LCD_SetColors(RED,RED); 
    }
    else if (LCDParam.BatChargeCap>(float)DRAW_BAT_LOWLEVEL & LCDParam.BatChargeCap<(float)DRAW_BAT_MIDLEVEL )
    {
      LCD_SetColors(YELLOW,YELLOW);
    }
    else LCD_SetColors(GREEN,GREEN);
    LCD_DrawFullRect(DRAW_BAT_BASE1_X+2,DRAW_BAT_BASE1_Y+2,(DRAW_BAT_LENGTH-4)*LCDParam.BatChargeCap,DRAW_BAT_HEIGHT-5);
  }

  // Display BIG!
  LCD_SetFont(&LCD_BIG_DATA_FONT);
  LCD_SetColors(WHITE,BLACK);

  /* Display motor parameters*/
  if(MotorParamsCondition.MotorRPM < MAX_VAL_MOTOR_RPM && MotorParamsCondition.MotorRPM > MIN_VAL_MOTOR_RPM ) 
  {
    LCD_SetColors(WHITE,BLACK);
    LCD_DisplayString_Pos_Align(70,25, RIGHT,LCD_Form_Str((char*)&LCDParam.RPM_str,4));     // display new RPM value      
  }
  else
  {
    LCD_SetColors(RED,BLACK);  
    LCD_DisplayString_Pos_Align(70,25, RIGHT,LCD_Form_Str((char*)&LCDParam.RPM_str,4));     // display new RPM value 
  }
  LCD_SetColors(WHITE,BLACK); 
  LCD_DisplayString_Pos_Align(235,25,RIGHT,LCD_Form_Str((char*)&LCDParam.Power_str,5));     // Power in Watts  


  //Display MIDDLE!
  LCD_SetFont(&LCD_SM_DATA_FONT);
  LCD_SetColors(WHITE,BLACK);

  if(MotorParamsCondition.MotorTemp < MAX_VAL_MOTOR_TEMP && MotorParamsCondition.MotorTemp > MIN_VAL_MOTOR_TEMP )// Newly Addded for color display 
  {  
    LCD_SetColors(WHITE,BLACK);
    LCD_DisplayString_Pos_Align(55, 105, RIGHT,LCD_Form_Str((char*)&LCDParam.TMotor_str,3));      // Motor Temp in °C 
  }
  else
  {
    LCD_SetColors(RED,BLACK);
    LCD_DisplayString_Pos_Align(55, 105, RIGHT,LCD_Form_Str((char*)&LCDParam.TMotor_str,3));      // Motor Temp in °C 
  }
  if(MotorParamsCondition.ControllerTemp < MAX_VAL_CONTR_TEMP && MotorParamsCondition.ControllerTemp > MIN_VAL_CONTR_TEMP )
  {
    LCD_SetColors(WHITE,BLACK);
    LCD_DisplayString_Pos_Align(130,105, RIGHT,LCD_Form_Str((char*)&LCDParam.TCont_str,3));       // Controller Temp in °C
  }
  else
  {
    LCD_SetColors(RED,BLACK);
    LCD_DisplayString_Pos_Align(130, 105, RIGHT,LCD_Form_Str((char*)&LCDParam.TCont_str,3));      // Controller Temp in °C
  }
  /* Display Temperatures*/
  if(BatteryParamsCondition.HighestTemperature < MAX_VAL_BATT_HIGH_TEMP && BatteryParamsCondition.HighestTemperature > MIN_VAL_BATT_HIGH_TEMP )
  {  
    LCD_SetColors(WHITE,BLACK);
    LCD_DisplayString_Pos_Align(195, 105, RIGHT,LCD_Form_Str((char*)&LCDParam.TBattHigh_str,3));  // Highest Battery Temp in °C
  }
  else
  {
    LCD_SetColors(RED,BLACK);
    LCD_DisplayString_Pos_Align(195, 105, RIGHT,LCD_Form_Str((char*)&LCDParam.TBattHigh_str,3));  // Highest Battery Temp in °C
  }
  if(BatteryParamsCondition.LowestTemperature < MAX_VAL_BATT_LOW_TEMP && BatteryParamsCondition.LowestTemperature > MIN_VAL_BATT_LOW_TEMP)
  { 
    LCD_SetColors(WHITE,BLACK);
    LCD_DisplayString_Pos_Align(265, 105, RIGHT,LCD_Form_Str((char*)&LCDParam.TBattLow_str,3)); 
  }
  else
  {
    LCD_SetColors(RED,BLACK);
    LCD_DisplayString_Pos_Align(265, 105, RIGHT,LCD_Form_Str((char*)&LCDParam.TBattLow_str,3)); 
  }

  // Display BIG!
  LCD_SetFont(&LCD_BIG_DATA_FONT);

  /* Display battery parameters*/
  if(MotorParamsCondition.BatVoltage < MAX_VAL_BATT_VOLT && MotorParamsCondition.BatVoltage > MIN_VAL_BATT_VOLT )
  {
    LCD_SetColors(WHITE,BLACK);
    LCD_DisplayString_Pos_Align(80, 160, RIGHT,LCD_Form_Str((char*)&LCDParam.BatVoltage_str,4));  // Battery Voltage in V
  }
  else
  { 
    LCD_SetColors(RED,BLACK);
    LCD_DisplayString_Pos_Align(80, 160, RIGHT,LCD_Form_Str((char*)&LCDParam.BatVoltage_str,4));  // Battery Voltage in V
  }
  if(MotorParamsCondition.BatCurrent < MAX_VAL_BATT_CURR && MotorParamsCondition.BatCurrent > MIN_VAL_BATT_CURR)
  { 
    LCD_SetColors(WHITE,BLACK);
    LCD_DisplayString_Pos_Align(210, 160, RIGHT,LCD_Form_Str((char*)&LCDParam.BatCurrent_str,4)); // Battery Current in A
  }   
  else
  { 
    LCD_SetColors(RED,BLACK);
    LCD_DisplayString_Pos_Align(210, 160, RIGHT,LCD_Form_Str((char*)&LCDParam.BatCurrent_str,4));  // Battery Current in A
  }
  LCD_SetFont(&LCD_SM_DATA_FONT);
  if (BatteryParamsCondition.BatteryTotalCapacity<10)
  {
    LCD_SetColors(RED,BLACK);
    LCD_DisplayString_Pos_Align(285, 207, RIGHT,LCD_Form_Str((char*)&LCDParam.batchargecap_str,4)); 
  }
  else if (BatteryParamsCondition.BatteryTotalCapacity>10 && BatteryParamsCondition.BatteryTotalCapacity<40 )
  {
    LCD_SetColors(YELLOW,BLACK);
    LCD_DisplayString_Pos_Align(285, 207, RIGHT,LCD_Form_Str((char*)&LCDParam.batchargecap_str,4)); 
  }
  else
  {      
    LCD_SetColors(WHITE,BLACK);
    LCD_DisplayString_Pos_Align(285, 207, RIGHT,LCD_Form_Str((char*)&LCDParam.batchargecap_str,4)); 
  }
  LCD_SetColors(WHITE,BLACK);
  LCD_DisplayString_Pos_Align(295, 207, RIGHT,"%"); 

#else
#error: "Illegal target board selection!"
#endif
}




/************* Functions only used by STM32F1 board ************************/

/**
*  *************************************************************************
* @brief  Write LCD Display state.
* @param  lcd_state: stores page info
* @retval None.
*****************************************************************************
*/
void wr_LCD_state(UINT8 lcd_state)
{
  LCD_state = lcd_state;
}

/**
*  ***********************************************************************
* @brief  Get LCD Display state.
* @retval None.
**************************************************************************
*/
UINT8 get_LCD_state(void)
{
  return LCD_state;
}

/**
*  ***********************************************************************
* @brief  Display Boot_Screen.
* @retval None.
**************************************************************************
*/
void Boot_Screen(void)
{
#ifdef USE_MCBSTM32C
  wr_LCD_state(State_Boot_Screen);
  GLCD_Clear(BACKGROUND);                                                  // Clear the GLCD
#elif defined USE_DISCOVERY

#else
#error: "Illegal target board selection!"
#endif
}
