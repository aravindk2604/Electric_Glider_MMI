/**  
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
 */          


/************************************************************************\
*    Headerfiles 
\************************************************************************/

#include "SDlib.h"



/************************************************************************\
*    local functions
\************************************************************************/


/************** FATFS related functions ***********************/
FRESULT TM_FATFS_DriveSize(uint32_t* total, uint32_t* free) 
/************************************************************************
*  @brief  Evoke the FATFS Drive size
*  @param  data: Total size, free space
*  @retval  FRESULT structre
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
*           and functions from:
*           fatfs_sd.c and ff.c
************************************************************************/
{
  FATFS *fs;
    DWORD fre_clust;
  FRESULT res;

	/* Get volume information and free clusters of drive */
	res = f_getfree("0:", &fre_clust, &fs);
	if (res != FR_OK) {
		return res;
	}

	/* Get total sectors and free sectors */
	*total = (fs->n_fatent - 2) * fs->csize / 2;
	*free = fre_clust * fs->csize / 2;

	/* Return OK */
	return FR_OK;
}

FRESULT TM_FATFS_USBDriveSize(uint32_t* total, uint32_t* free) 
/************************************************************************
*  @brief  Evoke the USB Drive size
*  @param  data: Total size, free space
*  @retval  FRESULT structre
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
*           and functions from:
*           fatfs_sd.c and ff.c
************************************************************************/
{
  FATFS *fs;
    DWORD fre_clust;
  FRESULT res;

	/* Get volume information and free clusters of drive */
	res = f_getfree("1:", &fre_clust, &fs);
	if (res != FR_OK) {
		return res;
	}

	/* Get total sectors and free sectors */
	*total = (fs->n_fatent - 2) * fs->csize / 2;
	*free = fre_clust * fs->csize / 2;

	/* Return OK */
	return FR_OK;
}


/************** SPI related functions ***********************/
uint8_t TM_SPI_Send(SPI_TypeDef* SPIx, uint8_t data) 
/************************************************************************
*  @brief  Send data over specified SPI port.
*  @param  data: SPIx port, data to be sent
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
*           and functions from:
*           SDlib.c
************************************************************************/
{
  //Fill output buffer with data
  SPIx->DR = data;
  //Wait for transmission to complete
  while (!SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE));
  //Wait for received data to complete
  while (!SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE));
  //Wait for SPI to be ready
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY));
  //Return data from buffer
  return SPIx->DR;
}




void TM_SPI_SendMulti(SPI_TypeDef* SPIx, uint8_t* dataOut, uint8_t* dataIn, uint16_t count)
/************************************************************************
*  @brief  Send (mutiple) data over specified SPI port.
*  @param  data: SPIx port, dataOut output data, dataIn input data, count number of Bytes
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
*           and functions from:
*           SDlib.c
************************************************************************/
{
  uint16_t i;
  for (i = 0; i < count; i++) {
    dataIn[i] = TM_SPI_Send(SPIx, dataOut[i]);
  }
}

void TM_SPI_WriteMulti(SPI_TypeDef* SPIx, uint8_t* dataOut, uint16_t count)
/************************************************************************
*  @brief  Write (mutiple) data over specified SPI port.
*  @param  data: SPIx port, dataOut output data, count number of Bytes
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
*           and functions from:
*           SDlib.c
************************************************************************/
{
  uint16_t i;
  for (i = 0; i < count; i++) {
    TM_SPI_Send(SPIx, dataOut[i]);
  }
}

void TM_SPI_ReadMulti(SPI_TypeDef* SPIx, uint8_t* dataIn, uint8_t dummy, uint16_t count) 
/************************************************************************
*  @brief  Read (mutiple) data over specified SPI port.
*  @param  data: SPIx port, dataIn input data, dummy, count number of Bytes
*  @retval  none
* Additionnal    : Conditionnal compile flags, Global variables and 
*           Constants defined in platform_cfg.h
*           and functions from:
*           SDlib.c
************************************************************************/
{
  uint16_t i;
  for (i = 0; i < count; i++) {
    dataIn[i] = TM_SPI_Send(SPIx, dummy);
  }
}
