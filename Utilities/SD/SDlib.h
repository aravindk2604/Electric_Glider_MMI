/**
 * Fatfs implementation for STM32F4xx devices
 *
 * This library uses Chan's Fatfs implementation.
 * 
 * This library is only for communication. To work with files, you have to look at Chan's FatFs manual, link below:
 * http://elm-chan.org/fsw/ff/00index_e.html
 *
 * You can work with SPI or SDIO protocol to interface SDcard.
 *
 * For more details look at link below
 *
 *  @author   Tilen Majerle
 *  @email    tilen@majerle.eu
 *  @website  http://stm32f4-discovery.com
 *  @link    http://stm32f4-discovery.com/2014/07/library-21-read-sd-card-fatfs-stm32f4xx-devices/
 *  @link    http://stm32f4-discovery.com/2014/08/library-29-usb-msc-host-usb-flash-drive-stm32f4xx-devices
 *  @version   v1.2
 *  @ide    Keil uVision
 *  @license  GNU GPL v3
 *  
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 *  
 * Version 1.2
 *  - 28.08.2014
 *  - Added support for USB
 *
 *
 * -----------------------------------------------------------------------------------------
 * Library works with SPI or SDIO mode. Also, when in SDIO mode, you can set to 1- or 4-bit mode.
 * By default, SDIO with 4-bit mode is used, so you will look into right column on table below.
 * 
 * SD CARD PINS
 *      _________________
 *     / 1 2 3 4 5 6 7 8 |
 *    /          |
 *  / 9          |
 *  |           |
 *  |          |
 *  |          |
 *  |          |
 *  |   SD CARD Pinout  |
 *  |          |
 *  |          |
 *  |          |
 *  |          |
 *  |___________________|
 *  
 *  
 * Default pinout for SD card
 * 
 *  NR  |SDIO INTERFACE                                  |SPI INTERFACE
 *      |NAME     STM32F4XX        DESCRIPTION           |NAME  STM32F4XX  DESCRIPTION
 *      |         4-BIT  1-BIT                           |
 *      |                                                |
 *  1  |CD/DAT3   PC11    -       Connector data line 3  |CS    PB5     Chip select for SPI
 *  2  |CMD       PD2     PD2     Command/Response line  |MOSI  PA7     Data input for SPI
 *  3  |VSS1      GND     GND     GND                    |VSS1  GND     GND
 *  4  |VDD       3.3V    3.3V    3.3V Power supply      |VDD   3.3V    3.3V Power supply
 *  5  |CLK       PC12    PC12    Clock                  |SCK   PA5     Clock for SPI
 *  6  |VSS2      GND     GND     GND                    |VSS2  GND     GND
 *  7  |DAT0      PC8     PC8     Connector data line 0  |MISO  PA6     Data output for SPI
 *  8  |DAT1      PC9     -       Connector data line 1  |-     -        -
 *  9  |DAT2      PC10    -       Connector data line 2  |-     -        -
 *  
 *  
 * SDIO Communication
 * 
 * By default, SDIO with 4-bit communication is used.
 * If you want to use SDIO 1-bit communication, set defines below in your defines.h file:
 * 
 * Files needed for SDIO
 *    - fatfs/diskio.h
 *    - fatfs/diskio.c
 *    - fatfs/ff.h
 *    - fatfs/ff.c
 *    - fatfs/ffconf.h
 *    - fatfs/drivers/fatfs_sd_sdio.h
 *    - fatfs/drivers/fatfs_sd_sdio.c
 * 
 * SPI Communication
 * 
 * Files, needed for SPI
 *    - fatfs/diskio.h
 *    - fatfs/diskio.c
 *    - fatfs/ff.h
 *    - fatfs/ff.c
 *    - fatfs/ffconf.h
 *    - fatfs/drivers/fatfs_sd.h
 *    - fatfs/drivers/fatfs_sd.c
 * 
 * Also, library has support for Write protect and Card detect pins.
 * This two pins are by default on pins below.
 * They are the same for any communication used, and are disabled by default
 * WP and CD Pinout
 * 
 *   NAME  STM32F4XX  DESCRIPTION
 *   
 *  WP    PB7      Write protect pin. Pin low when write is enabled
 *  CD    PB6      Card detect pin. Pin low when card detected
 *  
 *  Like I said before, these 2 pins are disabled by default. If you want to use it, you have to add 2 lines in your defines.h file
 *  
 *  //Enable Card detect pin
 *  #define FATFS_USE_DETECT_PIN        1
 *
 *  //Enable Write protect pin
 *  #define FATFS_USE_WRITEPROTECT_PIN      1
 *  
 *  Add lines below to your defines.h file only if you want to overwrite default pin settings
 *  //Default CD pin
 *  #define FATFS_USE_DETECT_PIN_RCC      RCC_AHB1Periph_GPIOB      
 *  #define FATFS_USE_DETECT_PIN_PORT     GPIOB
 *  #define FATFS_USE_DETECT_PIN_PIN      GPIO_Pin_6
 *  //Default WP pin
 *  #define FATFS_USE_WRITEPROTECT_PIN_RCC    RCC_AHB1Periph_GPIOB      
 *  #define FATFS_USE_WRITEPROTECT_PIN_PORT    GPIOB
 *  #define FATFS_USE_WRITEPROTECT_PIN_PIN    GPIO_Pin_7
 *
 * 
 * get_fattime() time function for FatFs
 * 
 * FatFs uses function for time, to set timestamp for files, when they were edited or created.
 * 
 * By default, function returns 0, but if you want to create your own function, you have to set defines in defines.h file:
 * 
 *   //Use custom get_fattime() function
 *   #define FATFS_CUSTOM_FATTIME  1
 *
 * And then somewhere in your project, add function like below:
 * 
 *   //Use custom get_fattime function
 *   //Implement RTC get time here if you need it
 *   DWORD get_fattime (void) {
 *     //Get your time from RTC or something like that
 *     
 *     return    ((DWORD)(2014 - 1980) << 25)  // Year 2014
 *         | ((DWORD)7 << 21)        // Month 7
 *         | ((DWORD)10 << 16)        // Mday 10
 *         | ((DWORD)16 << 11)        // Hour 16
 *         | ((DWORD)0 << 5)        // Min 0
 *         | ((DWORD)0 >> 1);        // Sec 0
 *   }
 * 
 * 
 */
#ifndef SDLIB_H
#define SDLIB_H  120
/**
 * Dependencies
 *  - STM32F4xx
 *  - STM32F4xx RCC
 *  - STM32F4xx GPIO
 *  - STM32Fx44 SPI    (only when SPI)
 *  - STM32F4xx DMA    (only when SDIO)
 *  - STM32Fx44 SDIO  (only when SDIO)
 *  - MISC        (only when SDIO)
 *  - FatFS by chan
 */




/**
 * Includes
 */
#ifdef USE_DISCOVERY
  #include "stm32f4xx.h"
  #include "stm32f4xx_rcc.h"
  #include "stm32f4xx_gpio.h"
#elif defined USE_MCBSTM32C
  #include "stm32f10x_spi.h"
  #include "stm32f10x_rcc.h"
  #include "stm32f10x_gpio.h"
#else
  #error: "Illegal target board selection!"
#endif

#include "platform_cfg.h"
#include "ff.h"

/**
 * Get SD card drive size
 *
 * Parameters:
 *   - uint32_t* total: pointer to variable to store total size of card
 *   - uint32_t* free: pointer to variable to store free space on card
 *
 * Returns FRESULT struct members. If data are valid, FR_OK is returned.
 */
FRESULT TM_FATFS_DriveSize(uint32_t* total, uint32_t* free);

/**
 * Get SD card drive size
 *
 * Parameters:
 *   - uint32_t* total: pointer to variable to store total size of card
 *   - uint32_t* free: pointer to variable to store free space on card
 *
 * Returns FRESULT struct members. If data are valid, FR_OK is returned.
 */
FRESULT TM_FATFS_USBDriveSize(uint32_t* total, uint32_t* free);





uint8_t TM_SPI_Send(SPI_TypeDef* SPIx, uint8_t data);

void TM_SPI_SendMulti(SPI_TypeDef* SPIx, uint8_t* dataOut, uint8_t* dataIn, uint16_t count);

void TM_SPI_WriteMulti(SPI_TypeDef* SPIx, uint8_t* dataOut, uint16_t count);

void TM_SPI_ReadMulti(SPI_TypeDef* SPIx, uint8_t* dataIn, uint8_t dummy, uint16_t count);




#endif

