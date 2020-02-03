/*******************************************************************************
* Title                 :   SPI Interface
* Filename              :   spi_interface.h
* Author                :   Marko Galevski
* Origin Date           :   20/01/2020
* Version               :   1.0.0
* Compiler              :   None
* Target                :   None
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

/** @file spi_interface.h
 *  @brief General interface covering user accesses to the spi communication
 *  	bus.
 */
#ifndef _SPI_H
#define _SPI_H

#include "spi_stm32f411_config.h"
#include "gpio_interface.h"
#include <stdint.h>

/**
 * Contains the options for slave select
 */
typedef enum
{
	SS_ACTIVE_LOW, /**<A slave is selected by pulling its select pin low */
	SS_ACTIVE_HIGH /**<A slave is selected by pulling its select pin high */
}spi_ss_polarity_t;

/**
 * Contains the options for the clock's polarity
 */
typedef enum
{
	ACTIVE_HIGH, 	/**<The clock's idle state is low and ticks upwards*/
	ACTIVE_LOW		/**<The clock's idle state is high and ticks downwards */
}spi_clock_polarity_t;

/**
 * Contains the options for the clock's phase
 */
typedef enum
{
	FIRST_EDGE,	/**<Data is clocked out/sampled on the first edge */
	SECOND_EDGE	/**<Data is clocked out/sampled on the second edge */
}spi_clock_phase_t;

/**
 * Contanins the options for the bit order of transfers
 */
typedef enum
{
	MSB_FIRST,	/**<The most significant bit is sent first */
	LSB_FIRST	/**<The least significant bit is sent first */
}spi_bit_format_t;

/**
 * Contains the options for payload size
 */
typedef enum
{
	SPI_DATA_8BIT,	/**<The data is in 8 bit frames */
	SPI_DATA_16BIT	/**<The data is in 16 bit frames */
}spi_data_format_t;

/**
 * Contains the direction selections when using single data wire (Bidrectional) spi
 */
typedef enum
{
	BIDIR_RECEIVE,	/**<The data line is being used for data reception */
	BIDIR_TRANSMIT	/**<The data line is being used for data transmission */
}spi_bidir_dir_t;

/**
 * Struct containing implementation agnostic transfer information.
 */
typedef struct
{
	spi_channel_t channel;					/**<The on-chip spi device to manage the transfer*/
	gpio_pin_t slave_pin;					/**<The slave's ss pin */
	spi_ss_polarity_t ss_polarity;			/**<The polarity of slave_pin */
	uint16_t *tx_buffer;					/**<Pointer to the data buffer for transfers*/
	uint32_t tx_length;						/**<Length of the transfer buffer*/
	uint16_t *rx_buffer;					/**<Pointer to the data buffer for reception */
	uint32_t rx_length;						/**<Length of the reception buffer */
	spi_data_format_t data_format;			/**<Data size of the transfer elements*/
	spi_bit_format_t bit_format;			/**<MSB or LSB first*/
	spi_clock_polarity_t clock_polarity;	/**<Selection of the clock's active and idle states */
	spi_clock_phase_t clock_phase;			/**<Edge sensitivity on sampling and shifts */
	spi_bidir_dir_t bidir_direction;		/**<Direction of the single data line transfer*/
}spi_transfer_t;

void spi_init(spi_config_t *config_table);
void spi_transfer(spi_transfer_t *transfer);
void spi_transfer_it(spi_transfer_t *transfer);
void spi_irq_handler(spi_channel_t channel);
void spi_register_write(uint32_t spi_register, uint16_t value);
uint16_t spi_register_read(uint32_t spi_register);

#endif
