/**
*  ***********************************************************************
*  @file    mctDefs.h
*  @author  B. Mysliwetz
*  @version V3.01
*  @brief   Shorthands, Useful constants and operator 
*
*           General purpose constants, macros & type definitions
*           Should be included after the system include files and before
*           any application include files.
*
*  Changes:
*  13.08.2010  mys         Initial Version for STM32F10x    
*  21.07.2011  mys         STM32F107: CR -> ASC_CR, to avoid 
*                          ID conflict with stm32f10x.h struct elements
*  25.01.2013  mys         Doxygen conformal headers and commenting style
*                      
*  @date    25/01/2013
**************************************************************************
*/

#ifndef _mCT_BASIC_DEFS_H  //!< avoid multiple/recursive includes
#define _mCT_BASIC_DEFS_H


/************************************************************************\
*  some useful constants
\************************************************************************/

#define  ASC_CR   0x0D     //!< ASCII carriage return
#define  ASC_LF   0x0A     //!< ASCII line feed
#define  ASC_BELL 0x07     //!< ASCII bell

#define  TRUE     1        //!< logical TRUE value
#define  FALSE    0        //!< logical FALSE value


/************************************************************************\
*  some useful operators and macros
\************************************************************************/

#define  L_AND    &&       //!< logical and operator
#define  L_OR     ||       //!< logical or operator
#define  L_NOT    !        //!< logical not operator

#define  B_AND    &        //!< bitwise and operator
#define  B_OR     |        //!< bitwise or operator
#define  B_XOR    ^        //!< bitwise xor operator
#define  B_NOT    ~        //!< bitwise not operator









#define  lowbyte(word16)      ((word16) & 0xff)                         //!< lowbyte(x)           Extract the low order byte of USHORT 'x'
#define  highbyte(word16)     lowbyte((word16) >> 8)                   //!< highbyte(x)         Extract the high order byte of USHORT 'x'
#define  dim(x)               (sizeof(x) / sizeof(x[0]))                  //!< dim(x)              Computes the dimension of a array 'x'
#define  setvect(inum, addr) *((ISRP far *) ((inum) * 4)) = ((ISRP) addr)  //!< setvect(inum, addr) Install interrupt handler 'addr' in vector 'inum'
#define  getvect(inum)        (ISRP) (*((ISRP far *) ((inum) * 4)))        //!< getvect(inum)       Get the contents of interrupt vector 'inum'

/************************************************************************\
*  type shorthands
\************************************************************************/

typedef  unsigned char    BOOL ;     //!< 8-bit  unsigned
typedef  signed char      SCHAR ;    //!< 8-bit  signed
typedef  unsigned char    UCHAR ;    //!< 8-bit  unsigned
typedef  signed char      INT8 ;     //!< 8-bit  signed
typedef  unsigned char    UINT8 ;    //!< 8-bit  unsigned
typedef  short            INT16 ;    //!< 16-bit signed 
typedef  unsigned short   UINT16 ;   //!< 16-bit unsigned
typedef  int              INT32 ;    //!< 32-bit signed  
typedef  unsigned int     UINT32 ;   //!< 32-bit unsigned
typedef  long long        INT64 ;    //!< 64-bit signed  
typedef  unsigned long long UINT64 ; //!< 64-bit unsigned

typedef  float            FLOAT ;    //!<32-bit floating point
typedef  double           DOUBLE ;   //!<64-bit floating point
typedef  long double      LDOUBLE ;  //!<80-bit floating point


//!<embedded programming / ARM related type variants

typedef  unsigned char    BYTE ;     //!< 8-bit  unsigned
typedef  unsigned short   HWORD ;    //!< 16-bit unsigned
typedef  unsigned long    LWORD ;     //!< 32-bit unsigned


//!<SD related variants
typedef unsigned char  BYTE;   //!< 8-bit  

//!< 16-bit
typedef short      SHORT;
typedef unsigned short  WORD;
typedef unsigned short  WCHAR;

//!< 16-bit / 32-bit
typedef int        INT;
typedef unsigned int  UINT;

//!< 32-bit
typedef long      LONG;
typedef unsigned long  DWORD;




#endif  //!<ifndef _mCT_BASIC_DEFS_H

/************************************************************************\
*  end of module mctDefs.h
\************************************************************************/

