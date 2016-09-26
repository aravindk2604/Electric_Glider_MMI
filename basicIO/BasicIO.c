/**
*  ***********************************************************************
*  @file    BasicIO.c
*  @author  B. Mysliwetz
*  @version V3.00
*
*  @brief   Collection of text/number-IO and LED-control functions  
*           for Real-Time Systems Lab & Praktikum Mikrocomputertechnik.
*
*  Provides TARGET-INDEPENDENT functions for serial character- & string-IO,
*  simple hex-formatted, decimal/integer-formatted IO and for LED-control.
*
*  Changes:
*  07.09.1997 mys          Initial version for EVA80386EX evalboard
*  17.04.2002 mys          LED-IO functions added
*  15.03.2009 mys          Adaptation of LED-functions for MCBSTM32
*  07.01.2013 mys          Doxygen compatible & new project folder structure 
*  24.02.2013 mys          target independent version
*  09.02.2015 jan/som      Added GPIO and LED compatibility for STM32F4 / Discovery board
*  02.06.2015 jan/som      Ported Serial IO functions from bsp.c/h
*  16.06.2015 Aravind & Madhan  : added float to string function for conversion of power and voltage values of the battery
*  15.10.2015 Aravind & Madhan  : added fixedpoint function for inserting a dot "." at desired place in an integer
*  @date    15/10/2015
**************************************************************************
*/



//* Includes */
#include  "bsp.h"           //!< target-specific, low level IO functions
#include  "BasicIO.h"       //!< Collection of text/number-IO and LED-control function prototypes
#include  "platform_cfg.h"  //!< hardware selection & abstraction defs
#include  <stdio.h>         //!< needed here for defs of FILE, EOF

/************************* Hardware IO functions ***********************************/

/************************* GPIO on/of ***********************************/

void    Set_GPIO_on( GPIO_TypeDef* GPIOx, UINT16 GPIO_Pin )
/**
* @brief  Sets the selected data port bits.
* @note   This functions uses GPIOx_BSRR register to allow atomic read/modify 
*         accesses. In this way, there is no risk of an IRQ occurring between
*         the read and the modify access.
* @param  GPIOx: where x can be (A..I) to select the GPIO peripheral.
* @param  GPIO_Pin: specifies the port bits to be written.
*          This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
* @retval None
*/
{
    GPIO_SetBits( GPIOx, GPIO_Pin );                            // Function provided by stm32f4_discovery.h and stm32f10x_gpio.h
}


void    Set_GPIO_off( GPIO_TypeDef* GPIOx, UINT16 GPIO_Pin )
/**
  * @brief  Clears the selected data port bits.
  * @note   This functions uses GPIOx_BSRR register to allow atomic read/modify 
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  * @param  GPIOx: where x can be (A..I) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bits to be written.
  *          This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
  * @retval None
  */
{

    GPIO_ResetBits( GPIOx, GPIO_Pin );                          // Function provided by stm32f4_discovery.c and stm32f10x_gpio.h
}


/************************* LED on/of ***********************************/
#ifdef USE_DISCOVERY
  void  LED_on( Led_TypeDef LED )
  /**
    * @brief  Turns selected LED On.
    * @param  Led: Specifies the Led to be set on.  
    * @retval None
    */
  {
    STM_EVAL_LEDOn(LED);                                    // Function provided by stm32f4_discovery. 
  }

  void  LED_off( Led_TypeDef LED )
  /**
    * @brief  Turns selected LED Off.
    * @param  Led: Specifies the Led to be set on.  
    * @retval None
    */
  {
    STM_EVAL_LEDOff(LED);                              // Function provided by stm32f4_discovery.c
  }


#elif defined USE_MCBSTM32C
  void  LED_on( int LED_number )
  /**
  *  ***********************************************************************
  *  @brief  Turn ON an individual LED n = {0-7}.
  *  @param  LED_number : BYTE defines LED number, 
  *  @retval none
  *  Global: GPIO_LED defined in platform_cfg.h
  **************************************************************************
  */
  {
    GPIO_SetBits( GPIO_LED, LED_number);
  }

  void  LED_off( int LED_number )
  /**
  *  ***********************************************************************
  *  @brief  Turn OFF an individual LED n = {0-7}.
  *  @param  LED_number : BYTE defines LED number
  *  @retval none
  *  Global: GPIO_LED defined in platform_cfg.h
  **************************************************************************
  */
  {
    GPIO_ResetBits( GPIO_LED, LED_number);
  }
#else
  #error: "Illegal target board selection!"
#endif

/************************* Type Conversion functions ********************************/
char  bin2ascii( char nibble )
/**
*  ***********************************************************************
*  @brief  Convert 4 bit value into ASCII hex character
*          Wandelt einen 4-Bit Eingangswert Zwischen 0 .. 15 in eine
*          Hexadezimalziffer, dh in ASCII-Zeichen '0'-'9', 'A'-'F' um.
*  @param  nibble : char  8 bit input value, only 4 lower bits significant
*  @retval hexadecimal digit ASCII value
**************************************************************************
*/
{
  nibble = nibble & 0x0f;              // only lower 4 bits used
  if ( nibble < 10 )
    return (nibble + 0x30);            // ASCII decimal digit '0'-'9'
  else
    return (nibble + 0x37);            // ASCII character 'A'-'F'
}

int  asciitoint( char *pintstring )
/**
*  ***********************************************************************
*  @brief  Convert integer value string in "%d" format to 32 bit integer
*
*          Wandelt Integerzahl-String analog "%d" in 32 Bit int-Wert um.
*  @param  *pintstring : string input buffer
*  @retval none
**************************************************************************
*/
{
  INT8   negative = 0;         // default assumed positive, no sign
  INT32  intval   = 0;

  while ( *pintstring != 0 )   // do until end of string
  {
    if ( *pintstring == '-' )  negative = 1;
    if ( *pintstring == '+' )  negative = 0;
    if ( (*pintstring >= '0' ) L_AND (*pintstring <= '9') )
    {
      intval = intval*10 + (*pintstring - '0');
    }
    pintstring++;              // next char in input buffer
  }

  if ( negative )              // take 2's complement
  {
    intval = B_NOT intval + 1;
  }
  return  intval;
}


// reverses a string 'str' of length 'len'
void  reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

void  fixedpoint(UINT16 input, char *outputString, UINT8 point)
/**
  * @brief  Function for inserting a decimal point in an integer
  * @param  input: input integer 
  * @param  outputString: array where output string is stored
  * @param  point: number of digits to be considered after decimal point
  * @retval None.
*/
{
  UINT16 locData = input;
  char str[4] = {0};
  signed char counter = 0;
   while(counter < point)
   {
    	str[counter++] = (locData % 10) + 48;
    	locData = locData/10;
   }
   str[counter++] = '.';

   while(locData>0)
   {
		str[counter++] = (locData % 10) + 48;
       	locData = locData/10;
   }
   while(counter < 4)
   {
    str[counter++] = 48;
   }
   str[counter] = '\0';

   // String Reverse operation.
   for (counter = 3; counter >=0 ; counter--)
   {
    	outputString[3 - counter] = str[counter];
   }
   outputString[4] = '\0';
} 

void  ftostr(float powerValue, char *outputString, int afterpoint)
/**
  * @brief  Function for converting float to string.
  * @param  powerValue: input number in float 
  * @param  outputString: array where output string is stored
  * @param  afterpoint: number of digits to be considered after decimal point
  * @retval None.
*/
{

   int locData = (int)(powerValue * 10.0);
   char str[4] = {0};
   signed char counter = 0;
   while(counter<1)
   {
    	str[counter++] = (locData % 10) + 48;
    	locData = locData/10;
   }
   str[counter++] = '.';

   while(locData>0)
   {
		str[counter++] = (locData % 10) + 48;
       	locData = locData/10;
   }
   while(counter < 4)
   {
    str[counter++] = 48;
   }
   str[counter] = '\0';

   // String Revere operation.
   for (counter = 3; counter >=0 ; counter--)
   {
    	outputString[3 - counter] = str[counter];
   }
   outputString[4] = '\0';
}



void  uint2str(UINT32 uintval, UINT8 *str, UINT8 clock_mode)
/**
  * @brief  Function for converting uint32 to string.
  * @param  uintval: uint
  * @param  str: string buffer
  * @param  clock_mode: if there is a '0' before 1-9
  * @retval None.
*/
{
  UINT32  decDigit;
  UINT32  pow10[] = { 1000000000, 100000000, 10000000, 1000000,
                                        100000, 10000, 1000, 100, 10, 1 };
  char   bIsLeading0 = 1;   // flag, do not print leading zeroes
  char   i;
  if(clock_mode)
     if(uintval < 10)       //only 1-9
        *str++ = '0';
  for ( i = 0; i < 10; i++ ) // do for max. 10 digits = powers of 10
  {
     decDigit = uintval / pow10[i];    // div. by powers of 10
     uintval  = uintval - decDigit * pow10[i];  // compute remainder
     if ( decDigit > 0 )
       {
       bIsLeading0 = 0;
       }
     if ( !bIsLeading0 || (i == 9) )    // if not a leading zero  or 
       {                                // if lowest digit
       *str++ = bin2ascii( decDigit ) ;  // print current digit
       }
  }
  *str = 0x00; //end                                  
}


/************************* USART print functions ********************************/

void  print_string( char *pstring )
/**
*  ***********************************************************************
*  @brief  Output string to stdout / UART#1
*  @param  pstring : *char  adress of string
*  @retval none
**************************************************************************
*/
{
  while ( *pstring != 0 )              // while not end of string
  {
    _putch( *pstring );                // output to stdout
    pstring++;                         // next char
  }
}


void  print_hexbyte( char c )
/**
*  ***********************************************************************
*  @brief  Output 8 bit value as two hex characters to stdout / UART#1
*          Gibt einen 8-Bit Eingangswert in hexadezimal formatiert als
*          zwei ASCII-Zeichen aus.
*  @param  c : char  8 bit input value
*  @retval none
**************************************************************************
*/
{
  _putch( bin2ascii((c & 0xf0) >> 4) ); // print hi nibble in hex
  _putch( bin2ascii( c & 0x0f) );      // print lo nibble in hex
}


void  print_hexbyteh( char c )
/**
*  ***********************************************************************
*  @brief  Output 8 bit value as two hex characters + 'h' to stdout
*          Gibt einen 8-Bit Eingangswert in hexadezimal formatiert als
*          zwei ASCII-Zeichen aus;
*          Wie print_hexbyte, zusaetzlich wird nach den beiden Hexziffern
*          noch ein 'h' ausgegeben
*  @param  c : char  8 bit input value
*  @retval none
**************************************************************************
*/
{
  print_hexbyte( c );
  _putch( 'h' );
}


void  print_hex16( UINT16 word16 )
/**
*  ***********************************************************************
*  @brief  Output 16 bit value in "0x%04X" format to stdout / UART#1
*
*          Gibt einen 16-Bit Eingangswert in hexadezimal formatiert als
*          vier ASCII-Zeichen mit '0x' Präfix aus.
*  @param  word16 : UINT16 input value
*  @retval none
**************************************************************************
*/
{
  print_string( "0x" );
  print_hexbyte( word16 >> 8 );      // print hi byte in hex
  print_hexbyte( word16 & 0x00ff );  // print lo byte in hex
}

void  print_hex0x16(UINT16 word16)
/**
*  ***********************************************************************
*  @brief  Output 16 bit value in "%04X" format to stdout / UART#1
*       ie WITHOUT leading '0x'
*          Gibt einen 16-Bit Eingangswert in hexadezimal formatiert als
*          vier ASCII-Zeichen aus OHNE '0x' Präfix.
*  @param  word16 : UINT16 input value
*  @retval none
**************************************************************************
*/
{
  print_hexbyte( word16 >> 8 );      // print hi byte in hex
  print_hexbyte( word16 & 0x00ff );  // print lo byte in hex
}


void  print_hex32( UINT32 word32 )
/**
*  ***********************************************************************
*  @brief  Output 32 bit value in "0x%08X" format to stdout / UART#1
*
*          Gibt einen 32-Bit Eingangswert in hexadezimal formatiert als
*          acht ASCII-Zeichen mit '0x' Präfix aus.
*  @param  word32 : UINT32 input value
*  @retval none
**************************************************************************
*/
{
  print_hex16( (WORD) (word32 >> 16) );           // print hi word
  print_hex0x16( (WORD) (word32 & 0x0000ffff) );  // print lo word
}


void    print_uint( UINT32 uintval )
/**
*  ***********************************************************************
*  @brief  Output 32 bit value in "%u" format to stdout / UART#1
*
*          Gibt einen 32 Bit Wert als vorzeichenlose Integerzahl aus.
*  @param  uintval : UINT32 input value
*  @retval none
**************************************************************************
*/                                              
{
  UINT32  decDigit;
  UINT32  pow10[] = { 1000000000, 100000000, 10000000, 1000000,
    100000, 10000, 1000, 100, 10, 1 };
  UINT8   bIsLeading0 = 1;   // flag, do not print leading zeroes
  UINT8   i;

  for ( i = 0; i < 10; i++ ) // do for max. 10 digits = powers of 10
  {
    decDigit = uintval / pow10[i];    // div. by powers of 10
    uintval  = uintval - decDigit * pow10[i];  // compute remainder

    if ( decDigit > 0 )
    {
      bIsLeading0 = 0;
    }

    if ( !bIsLeading0 L_OR (i == 9) )  // if not a leading zero  or 
    {                                  // if lowest digit
      _putch( bin2ascii( decDigit ) ); // print current digit
    }
  }                                   
}



void    print_int( INT32 intval )
/**
*  ***********************************************************************
*  @brief  Output 32 bit value in "%d" format to stdout / UART#1
*
*          Gibt einen 32 Bit Wert als vorzeichenbehaftete Integerzahl aus.
*  @param  intval : INT32 input value
*  @retval none
**************************************************************************
*/
{
  UINT32  uintval;

  if ( intval < 0 )                // if negative ->
  {
    uintval = B_NOT intval + 1;    // compute 2's complement
    _putch('-');                   // print - sign
  }
  else
  {
    uintval = intval;              // if positive -> print as is
  }

  print_uint( uintval );
}


void  getint( char *pintstring )
/**
*  ***********************************************************************
*  @brief  Read integer value string in "%d" format from stdin / UART#1
*
*          Liest eine Integerzahl als entsprechende ASCII-Ziffernfolge
*          ggf. mit Vorzeichen von der Tastatur ein.
*  @param  *pintstring : string input buffer
*  @retval none
**************************************************************************
*/
{
  char c = 0;

  while ( (c = _getch()) != ASC_CR )  // read until CR is entered
  {
    if ( ((c >= '0') && (c <= '9')) || (c == '+') || (c == '-') || (c == ' ') )
  // parse input for illegal characters
    {
      if (c != ' ')            // ignore/skip blanks
      {
        *pintstring = c;       // store char to input buffer
        pintstring++;          // next char
        _putch(c);             // echo input
      }
    }
    else                       // warning - illegal character entered
      _putch(ASC_BELL);

  } // end while

  *pintstring = 0;             // add end of string
}


char  _putch (int ch)
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial port#1.
*  @param  ch : character to output
*  @retval output character
**************************************************************************
*/
{
  while ( !(USART_x->SR & USART_FLAG_TXE) );
  USART_x->DR = (ch & 0xFF);
  return (ch);
}


char  _getch (void)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial port#1 (BLOCKiNG MODE).
*  @retval input character
**************************************************************************
*/
{
  while ( !(USART_x->SR & USART_FLAG_RXNE) );
  return (USART_x->DR & 0xFF);
}


char  _keyhit (void)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial port#1 (NON-BLOCKiNG MODE).
*  @retval input character
**************************************************************************
*/
{
  if ( USART_x->SR & USART_FLAG_RXNE )
    return (USART_x->DR & 0xFF);
  else
    return (0);
}


int  fputc (int ch, FILE *f)
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial port#1.
*  @param  ch : character to output
*  @param  f  : filepointer (as yet ignored here)
*  @retval output character
**************************************************************************
*/
{
  return ( _putch(ch) );
}


int  fgetc (FILE *f)
/**
*  ***********************************************************************
*  @brief  Read character from stdin / serial port#1 (BLOCKiNG MODE).
*  @param  f  : filepointer (as yet ignored here)
*  @retval input character
**************************************************************************
*/
{
  return ( _putch( _getch() ) );
}


void  _ttywrch (int ch)
/**
*  ***********************************************************************
*  @brief  Write character to stdout / serial port#1.
*  @param  ch : character to output
*  @retval none
**************************************************************************
*/
{
  _putch( ch );
}


int  ferror (FILE *f)
/**
*  ***********************************************************************
*  @brief  Return error status of file f, not yet implemented.
*  @param  f  : filepointer (as yet ignored here)
*  @retval error code (as yet const EOF)
**************************************************************************
*/
{
  // your implementation of ferror
  return EOF;
}


/************************************************************************\
*    END OF MODULE BasicIO.c
\************************************************************************/
