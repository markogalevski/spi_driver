/*******************************************************************************
* Title                 :   SPI Config Header for STM32F411
* Filename              :   spi_interface.h
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

/** @file spi_stm32f411_config.h
 *  @brief Contains the definitions and structures required to configure the
 *  		spi peripherals on an stm32f411
 */
#ifndef _SPI_STM32F411_CONFIG
#define _SPI_STM32F411_CONFIG

/**
 * Contains all of the spi devices found on chip
 */
typedef enum
{
	SPI_1 = 0x00UL,
	SPI_2 = 0x01UL,
	SPI_3 = 0x02UL,
	SPI_4 = 0x03UL,
	SPI_5 = 0x04UL,
	NUM_SPI = 0x05UL
}spi_channel_t;

/**
 * Contains the options for the spi device's functioning mode
 */
typedef enum
{
	SPI_SLAVE,	/**<The spi is functioning as a slave and will await selection and a clock*/
	SPI_MASTER	/**<The spi is functioning as a master and will select slaves and generate a clock */
}spi_master_slave_t;

/**
 *	Contains all of the prescaler options for the master clock generation
 */
typedef enum
{
	PCLK_DIV_2,
	PCLK_DIV_4,
	PCLK_DIV_8,
	PCLK_DIV_16,
	PCLK_DIV_32,
	PCLK_DIV_64,
	PCLK_DIV_128,
	PCLK_DIV_256
}spi_baud_rate_t;

/**
 *	Spi devices which are disabled are ignored during init
 */
typedef enum
{
	SPI_DISABLE,
	SPI_ENABLE
}spi_enable_t;

/**
 * Contains options to enable the self selection of the spi (as a slave) through software
 */
typedef enum
{
	HARDWARE_SMM,
	SOFTWARE_SMM
}spi_slave_mgmt_t;

/**
 * Contains options determining the topology of the bus system
 */
typedef enum
{
	UNIDIR_FULL_DUPLEX,	/**<Two data wire spi, with simultaneous data reception and transmission */
	UNIDIR_RXONLY,		/**<Two data wire spi, but with reception only */
	BIDIR_MODE			/**<Single data wire spi, must select transfer direction upon call*/
}spi_bidir_t;

/**
 * Contains options for enabling the hardware CRC calculation of received data
 */
typedef enum
{
	CRC_DISABLE,
	CRC_ENABLE
}spi_crc_en_t;

/**
 * Contains options to enable DMA requests upon data reception
 */
typedef enum
{
	RX_DMA_REQ_DISABLE,
	RX_DMA_REQ_ENABLE
}spi_rx_dma_t;

/**
 * Contains options to enable DMA requests upon data transmission
 */
typedef enum
{
	TX_DMA_REQ_DISABLE,
	TX_DMA_REQ_ENABLE
}spi_tx_dma_t;

/**
 * Contains options for slave select output control. Allows work in multi-master mode when disabled
 */
typedef enum
{
	SS_OUTPUT_DISABLE,
	SS_OUTPUT_ENABLE
}spi_ssoe_t;

/**
 * Options for motorola's vs TI's spi format. Motorola is the defualt and should suffice
 */
typedef enum
{
	SPI_MOTOROLA,
	SPI_TI
}spi_frame_format_t;

/**
 * Contains all the possible interrupts a spi device can handle
 */
typedef enum
{
	ERROR_INTERRUPT,	/**<Interrupt generated upon a communication error*/
	RXNE_INTERRUPT,		/**<Interrupt generated upon having non-read received data*/
	TXE_INTERRUPT		/**<Interrupt generated when no data is in the transfer buffer */
}spi_interrupt_t;

/**
 * An initialisation config struct which configures the global parameters for a spi device
 */
typedef struct
{
	spi_enable_t spi_enable;			/**<Decides whether the spi will be used at all */
	spi_master_slave_t master_slave;	/**<Decides whether the spi is in master or slave mode*/
	spi_slave_mgmt_t slave_management;	/**<Determines method of (self) slave management*/
	spi_bidir_t	bidirectional_mode;		/**<Configured based upon physical topology of the spi */
	spi_baud_rate_t baud_rate;			/**<Communication rate of the spi*/
}spi_config_t;

const spi_config_t *spi_config_get(void);


#endif
