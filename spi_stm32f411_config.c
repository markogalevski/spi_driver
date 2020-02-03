/*******************************************************************************
* Title                 :   SPI Config Table for STM32F411
* Filename              :   spi_stm32f411_config.c
* Author                :   Marko Galevski
* Origin Date           :   20/01/2020
* Version               :   1.0.0
* Compiler              :   gcc
* Target                :   STM32F411 (ARM Cortex M4)
* Notes                 :   None
*
*
*******************************************************************************/
/****************************************************************************
* Doxygen C Template
* Copyright (c) 2013 - Jacob Beningo - All Rights Reserved
*
* Feel free to use this Doxygen Code Template at your own risk for your own
* purposes.  The latest license and updates for this Doxygen C template can be
* found at www.beningo.com or by contacting Jacob at jacob@beningo.com.
*
* For updates, free software, training and to stay up to date on the latest
* embedded software techniques sign-up for Jacobs newsletter at
* http://www.beningo.com/814-2/
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Template.
*
*****************************************************************************/

/** @file spi_stm32f411_config.c
 *  @brief Contains the configuration information for each spi channel
 */
#include "spi_stm32f411_config.h"

static const spi_config_t config_table[NUM_SPI] =
{				//ENABLED			//MASTER		//SS_MODE			//BIDIR			//BAUD
									//SLAVE												//RATE
	/*SPI1*/	{},
	/*SPI2*/	{},
	/*SPI3*/	{},
	/*SPI4*/	{},
	/*SPI5*/	{}
};

/******************************************************************************
* Function: spi_config_get()
*//**
* \b Description:
*
* 	Returns a pointer to the base of the configuration table for spi peripherals
*
*
* PRE-CONDITION: The config table has been filled out and is non-null
*
* @return 		*spi_config_t
*
* \b Example:
* @code
*	const spi_config_t *config_table = spi_config_get();
*	spi_init(config_table);
* @endcode
*
* @see spi_init
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
const spi_config_t *spi_config_get(void)
{
	return(config_table);
}
