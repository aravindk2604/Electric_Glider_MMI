/*------------------------------------------------------------------------ /
/  Low level disk interface modlue include file   (C)ChaN, 2013             /
/  mod : 28.11.2014 Jan-Mos          defines.h integrated to platform_cfg.h /  
/-------------------------------------------------------------------------*/

#ifndef _DISKIO_DEFINED_SD
#define _DISKIO_DEFINED_SD

#define _USE_WRITE  1  /* 1: Enable disk_write function */
#define _USE_IOCTL  1  /* 1: Enable disk_ioctl fucntion */

#include "diskio.h"
#include "mctDefs.h"

#ifdef USE_DISCOVERY
  #include "stm32f4xx.h"
  #include "stm32f4xx_rcc.h"
  #include "stm32f4xx_gpio.h"
#elif defined USE_MCBSTM32C
  #include "stm32f10x.h"
  #include "stm32f10x_rcc.h"
  #include "stm32f10x_gpio.h"
#else
  #error: "Illegal target board selection!"
#endif

#include "misc.h"
#include "platform_cfg.h"
#include "BasicIO.h"

#define FATFS_CS_LOW              Set_GPIO_off(SPI_CS_PORT, SPI_CS_PIN);
#define FATFS_CS_HIGH             Set_GPIO_on(SPI_CS_PORT, SPI_CS_PIN);

//#ifndef FATFS_USE_DETECT_PIN
//#define FATFS_USE_DETECT_PIN        0
//#endif

//#ifndef FATFS_USE_WRITEPROTECT_PIN
//#define FATFS_USE_WRITEPROTECT_PIN      0
//#endif

//#if FATFS_USE_DETECT_PIN > 0
//#ifndef FATFS_USE_DETECT_PIN_PIN
//#define FATFS_USE_DETECT_PIN_RCC      RCC_AHB1Periph_GPIOB      
//#define FATFS_USE_DETECT_PIN_PORT      GPIOB
//#define FATFS_USE_DETECT_PIN_PIN      GPIO_Pin_6
//#endif
//#endif

//#if FATFS_USE_WRITEPROTECT_PIN > 0
//#ifndef FATFS_USE_WRITEPROTECT_PIN_PIN
//#define FATFS_USE_WRITEPROTECT_PIN_RCC    RCC_AHB1Periph_GPIOB      
//#define FATFS_USE_WRITEPROTECT_PIN_PORT    GPIOB
//#define FATFS_USE_WRITEPROTECT_PIN_PIN    GPIO_Pin_7
//#endif
//#endif
/*---------------------------------------*/
/* Prototypes for disk control functions */
DSTATUS TM_FATFS_SD_disk_initialize(void);
DSTATUS TM_FATFS_SD_disk_status(void);
DRESULT TM_FATFS_SD_disk_read(BYTE* buff, DWORD sector, UINT count);
DRESULT TM_FATFS_SD_disk_write(const BYTE* buff, DWORD sector, UINT count);
DRESULT TM_FATFS_SD_disk_ioctl(BYTE cmd, void* buff);

#endif

