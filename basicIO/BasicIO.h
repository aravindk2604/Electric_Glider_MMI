/**
*  ***********************************************************************
*  @file    BasicIO.h
*  @author  B. Mysliwetz
*  @version V3.00
*  @brief   Collection of text/number-IO and LED-control function prototypes 
*           for Real-Time Systems Lab & Praktikum Mikrocomputertechnik.
*
*  Provides TARGET-INDEPENDENT functions for serial character- & string-IO,
*  simple hex-formatted, decimal/integer-formatted IO and for LED-control.
*
*  Changes:
*  07.09.1997 mys          Initial version for EVA80386EX evalboard
*  17.04.2002 mys          LED-IO functions added
*  15.03.2009 mys          Adaptation of LED-functions for MCBSTM32
*  03.03.2011 mys          modularized include file structure
*  09.02.2015 jan/som      added basic GPIO on off function 
*  16.06.2015 Aravind & Madhan  : added float to string function for conversion of power and voltage values of the battery
*  15.10.2015 Aravind & Madhan  : added fixedpoint function for inserting a dot "." at desired place in an integer
*  @date    15/10/2015
**************************************************************************
*/

#ifndef _BASICIO_H       // avoid multiple includes
#define _BASICIO_H


/* Includes */
#ifdef USE_DISCOVERY                  //!< STM32F4 board specific files (condition)
  #include "stm32f4_discovery.h"      //!< definitions for STM32F4-Discovery Kit's Leds and push-button hardware resources
  #include "stm32f4xx_gpio.h"         //!< Library containing GPIO relate functions
  #include "stm32f4xx_rcc.h"          //!< Library containing RCC firmware related functions
  #include "stm32f4xx_conf.h"         //!< Periph. library configuration file
#elif defined USE_MCBSTM32C
  #include "stm32f10x_gpio.h"         //!< Library containing GPIO relate functions
  #include "stm32f10x_rcc.h"          //!< Library containing RCC firmware related functions
  #include "stm32f10x_conf.h"         //!< Periph. library configuration file
#else
  #error: "Illegal target board selection!"
#endif


#include "mctDefs.h"     // useful constants, macros, type shorthands 

/************************************************************************\
*    Hardware-IO functions
\************************************************************************/
#ifdef USE_DISCOVERY
  extern void    LED_on( Led_TypeDef LED );
  extern void    LED_off( Led_TypeDef LED );
#elif defined USE_MCBSTM32C
  extern void    LED_on( int LED_number );
  extern void    LED_off( int LED_number );
#else
  #error: "Illegal target board selection!"
#endif

  extern void    Set_GPIO_on( GPIO_TypeDef* GPIOx, UINT16 GPIO_Pin );
  extern void    Set_GPIO_off( GPIO_TypeDef* GPIOx, UINT16 GPIO_Pin );

/************************************************************************\
*    Mid-level formatted console IO
\************************************************************************/

  extern int     asciitoint( char *pintstring );
  extern void    print_string( char *pstring );
  extern void    print_hexbyteh( char c );
  extern void    print_hexbyte( char c );
  extern void    print_hex16( UINT16 word16 );
  extern void    print_hex32( UINT32 word32 );
  extern void    print_uint( UINT32 uintval );
  extern void    print_int( INT32 intval );
  extern void    getint( char *pintstring );
  extern char   _putch (int c);  // send char to console
  extern char   _getch (void);   // read char from console
  extern char   _keyhit(void);   // 0 if no input, else char from console
  extern void     reverse(char *str, int len); 
  extern void     uint2str( UINT32 uintval,UINT8 *str, UINT8 clock_mode); ///<Function for converting uint32 to string.
  extern void     ftostr( float z, char *res, int afterpoint); ///<Function for converting float to string.
  extern void     fixedpoint( UINT16 input, char *output, UINT8 point); ///<Function to insert decimal point in an integer

#endif  // _BASICIO_H

/************************************************************************\
*    END OF MODULE BasicIO.h
\************************************************************************/
