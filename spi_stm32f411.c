/*******************************************************************************
* Title                 :   SPI Implementation for STM32F411
* Filename              :   spi_stm32f411.c
* Author                :   Marko Galevski
* Origin Date           :   20/01/2020
* Version               :   1.0.0
* Compiler              :   GCC
* Target                :   STM32F411 (Arm Cortex M4)
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

/** @file spi_stm32f411.c
 *  @brief Chip specific implementation for spi communication.
 */
#include "spi_interface.h"
#include "stm32f411xe.h"
#include <assert.h>

/**
 * Redefinition of NULL macro in case stdlib isn't used by the rest of the project
 */
#ifndef NULL
#define NULL (void*) 0
#endif

/**
 * Array of pointers to Control Register 1 registers
 */
static volatile uint16_t *const SPI_CR1[NUM_SPI] =
{
	(uint16_t *)SPI1_BASE,	(uint16_t *)SPI2_BASE,
	(uint16_t *)SPI3_BASE,	(uint16_t *)SPI4_BASE,
	(uint16_t *)SPI5_BASE
};

/**
 * Array of pointers to Control Register 2 registers
 */
static volatile uint16_t *const SPI_CR2[NUM_SPI] =
{
	(uint16_t *)SPI1_BASE + 0x04UL,	(uint16_t *)SPI2_BASE + 0x04UL,
	(uint16_t *)SPI3_BASE + 0x04UL,	(uint16_t *)SPI4_BASE + 0x04UL,
	(uint16_t *)SPI5_BASE + 0x04UL
};

/**
 * Array of pointers to Status registers
 */
static volatile uint16_t *const SPI_SR[NUM_SPI] =
{
	(uint16_t *)SPI1_BASE + 0x08UL,	(uint16_t *)SPI2_BASE + 0x08UL,
	(uint16_t *)SPI3_BASE + 0x08UL,	(uint16_t *)SPI4_BASE + 0x08UL,
	(uint16_t *)SPI5_BASE + 0x08UL
};

/**
 * Array of pointers to Data registers
 */
static volatile uint16_t *const SPI_DR[NUM_SPI] =
{
	(uint16_t *)SPI1_BASE + 0x0CUL,	(uint16_t *)SPI2_BASE + 0x0CUL,
	(uint16_t *)SPI3_BASE + 0x0CUL,	(uint16_t *)SPI4_BASE + 0x0CUL,
	(uint16_t *)SPI5_BASE + 0x0CUL
};

/**
 * Array of pointers to CRC Polynomial registers
 */
static volatile uint16_t *const SPI_CRCPR[NUM_SPI] =
{
	(uint16_t *)SPI1_BASE + 0x10UL,	(uint16_t *)SPI2_BASE + 0x10UL,
	(uint16_t *)SPI3_BASE + 0x10UL,	(uint16_t *)SPI4_BASE + 0x10UL,
	(uint16_t *)SPI5_BASE + 0x10UL
};

/**
 * Array of pointers to reception CRC registers
 */
static volatile uint16_t *const SPI_RXCRCR[NUM_SPI] =
{
	(uint16_t *)SPI1_BASE + 0x14UL,	(uint16_t *)SPI2_BASE + 0x14UL,
	(uint16_t *)SPI3_BASE + 0x14UL,	(uint16_t *)SPI4_BASE + 0x14UL,
	(uint16_t *)SPI5_BASE + 0x14UL
};

/**
 * Array of pointers to transmission CRC registers
 */
static volatile uint16_t *const SPI_TXCRCR[NUM_SPI] =
{
	(uint16_t *)SPI1_BASE + 0x18UL,	(uint16_t *)SPI2_BASE + 0x18UL,
	(uint16_t *)SPI3_BASE + 0x18UL,	(uint16_t *)SPI4_BASE + 0x18UL,
	(uint16_t *)SPI5_BASE + 0x18UL
};

/**
 * Static array which holds safe copies of transfers for interrupt routines,
 * mapped to spi devices
 */
static spi_transfer_t spi_interrupt_transfers[NUM_SPI];

/**
 * Callback typedef for interrupt callbacks
 */
typedef void (*spi_interrupt_callback_t)(spi_transfer_t *);

/**
 * Static array of callback functions mapped to each spi device
 */
static spi_interrupt_callback_t spi_interrupt_callbacks[NUM_SPI];

static void spi_select_slave(spi_transfer_t *transfer);
static void spi_release_slave(spi_transfer_t *transfer);
static void spi_configure_clock(spi_transfer_t *transfer);
static void spi_configure_data_frame(spi_transfer_t *transfer);

static void spi_transfer_bidir(spi_transfer_t *transfer);
static void spi_transfer_bidir_transmit(spi_transfer_t *transfer);
static void spi_transfer_bidir_receive(spi_transfer_t *transfer);

static void spi_transfer_full_duplex_rxonly(spi_transfer_t *transfer);

static void spi_transfer_full_duplex(spi_transfer_t *transfer);
static void spi_transfer_full_duplex_master(spi_transfer_t *transfer);
static void spi_transfer_full_duplex_slave(spi_transfer_t *transfer);

static void spi_transfer_it_bidir(spi_transfer_t *transfer);
static void spi_transfer_it_full_duplex_rxonly(spi_transfer_t *transfer);
static void spi_transfer_it_full_duplex(spi_transfer_t *transfer);


/******************************************************************************
* Function: spi_init()
*//**
* \b Description:
*
* 	Carries out the initialisation of the spi channels as per the information
* 	in the config table
*
*
* PRE-CONDITION: The config table has been obtained and is non-null
* PRE-CONDITION: The required GPIO pins for the spi combination have been configured
* 					correctly with gpio_init
* PRE-CONDITION: The appropriate peripheral clocks have been activated
*
* POST-CONDITION: The selected spi channels have been activated and ready to be used
*
*
* @return 		void
*
* \b Example:
* @code
*	const spi_config_t *config_table = spi_config_get();
*	spi_init(config_table);
* @endcode
*
* @see spi_config_get
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
void spi_init(spi_config_t *config_table)
{
	for (int spi_channel = 0; spi_channel < NUM_SPI; spi_channel++)
	{
		*SPI_CR1[spi_channel] &= ~(SPI_CR1_SPE_Msk);
		if (sizeof(config_table[spi_channel])!= 0)
		{
			if (config_table[spi_channel].spi_enable == SPI_ENABLE)
			{
				*SPI_CR1[spi_channel] &= ~(SPI_CR1_MSTR_Msk);
				*SPI_CR1[spi_channel] |= config_table[spi_channel].master_slave << SPI_CR1_MSTR_Pos;

				*SPI_CR1[spi_channel] &= ~(SPI_CR1_SSM_Msk);
				*SPI_CR1[spi_channel] |= config_table[spi_channel].slave_management << SPI_CR1_SSM_Pos;

				*SPI_CR1[spi_channel] &= ~(SPI_CR1_BIDIMODE_Msk | SPI_CR1_RXONLY_Msk);
				if (config_table[spi_channel].bidirectional_mode == BIDIR_MODE)
				{
					*SPI_CR1[spi_channel] |= SPI_CR1_BIDIMODE_Msk;
				}
				else if (config_table[spi_channel].bidirectional_mode == UNIDIR_RXONLY)
				{
					*SPI_CR1[spi_channel] |= SPI_CR1_RXONLY_Msk;
				}

				*SPI_CR1[spi_channel] &= ~(SPI_CR1_BR_Msk);
				*SPI_CR1[spi_channel] |= config_table[spi_channel].baud_rate << SPI_CR1_BR_Pos;

			}
		}
	}
}

/******************************************************************************
* Function: spi_transfer()
*//**
* \b Description:
*
* 	Carries out a blocking spi transfer according to the specifications of the
* 	 transfer parameter.
*
*
*
* PRE-CONDITION: spi_init() has been successfully carried out for the required spi channel
* PRE-CONDITION: gpio_init() has been called for the slave select pin to configure it as an output/input,
* 					depending on desired direction
* PRE-CONDITION: The proper data buffers and lengths are non-null/non-zero
* PRE-CONDITION: The transfer pointer is non-null
*
* POST-CONDITION: The desired transfer has been successfully carried out
*
*
* @return 		void
*
* \b Example:
* @code
*	spi_transfer_t flash_transfer;
*	flash_transfer.channel = SPI_3;
*	flash_transfer.slave_pin = GPIO_C_3;
*	flash_transfer.ss_polarity = ACTIVE_LOW;
*	flash_transfer.tx_buffer = &data_out;
*	flash_transfer.tx_length = sizeof(data_out);
*	flash_transfer.rx_buffer = &dummy_data;
*	flash_transfer.rx_length = sizeof(data_out);
*	flash_transfer.data_format = SPI_DATA_8BIT;
*	flash_transfer.bit_format = MSB_FIRST;
*	flash_transfer.clock_polarity = ACTIVE_HIGH;
*	flash_transfer.clock_phase = SECOND_EDGE;
*	spi_transfer(&flash_transfer);
* @endcode
*
* @see spi_init
* @see spi_transfer_it
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
void spi_transfer(spi_transfer_t *transfer)
{
	assert(transfer != NULL);
	spi_configure_clock(transfer);
	spi_configure_data_frame(transfer);
	uint16_t CR1_state = *SPI_CR1[transfer->channel];

	if (CR1_state & SPI_CR1_MSTR_Msk)
	{
		spi_select_slave(transfer);
	}

	*SPI_CR1[transfer->channel] |= SPI_CR1_SPE_Msk;

	if (CR1_state & SPI_CR1_BIDIMODE_Msk)
	{
		spi_transfer_bidir(transfer);
	}
	else if (CR1_state & SPI_CR1_RXONLY_Msk)
	{
		spi_transfer_full_duplex_rxonly(transfer);
	}
	else
	{
		spi_transfer_full_duplex(transfer);
	}

	if (CR1_state & SPI_CR1_MSTR_Msk)
	{
	spi_release_slave(transfer);
	}

	*SPI_CR1[transfer->channel] &= ~(SPI_CR1_SPE_Msk);
}

/******************************************************************************
* Function: spi_transfer_it()
*//**
* \b Description:
*
* 	Sets up an interrupt based spi transfer according to the specifications of
* 	 the transfer parameter. Makes a safe copy of the transfer structure and
* 	 calls a function to map the correct callback
*
*
*
* PRE-CONDITION: spi_init() has been successfully carried out for the required spi channel
* PRE-CONDITION: gpio_init() has been called for the slave select pin to configure it as an output/input,
* 					depending on desired direction
* PRE-CONDITION: The proper data buffers and lengths are non-null/non-zero
* PRE-CONDITION: The transfer pointer is non-NULL
*
* POST-CONDITION: The irq handler will now handle the rest of the transfer
* POST-CONDITION: A safe copy of the transfer structure has been placed in the static file-wide buffer
*
*
* @return 		void
*
* \b Example:
* @code
*	spi_transfer_t flash_transfer;
*	flash_transfer.channel = SPI_3;
*	flash_transfer.slave_pin = GPIO_C_3;
*	flash_transfer.ss_polarity = ACTIVE_LOW;
*	flash_transfer.tx_buffer = &data_out;
*	flash_transfer.tx_length = sizeof(data_out);
*	flash_transfer.rx_buffer = &dummy_data;
*	flash_transfer.rx_length = sizeof(data_out);
*	flash_transfer.data_format = SPI_DATA_8BIT;
*	flash_transfer.bit_format = MSB_FIRST;
*	flash_transfer.clock_polarity = ACTIVE_HIGH;
*	flash_transfer.clock_phase = SECOND_EDGE;
*	spi_transfer_it(&flash_transfer);
* @endcode
*
* @see spi_init
* @see spi_transfer
* @see spi_irq_handler
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
void spi_transfer_it(spi_transfer_t *transfer)
{
	assert(transfer != NULL);
	spi_interrupt_transfers[transfer->channel] = *transfer;
	spi_configure_clock(transfer);
	spi_configure_data_frame(transfer);

	uint16_t CR1_state = *SPI_CR1[transfer->channel];

	if (CR1_state & SPI_CR1_MSTR_Msk)
	{
		spi_select_slave(transfer);
	}

	if (CR1_state & SPI_CR1_BIDIMODE_Msk)
	{
		spi_transfer_it_bidir(transfer);
	}
	else if (CR1_state & SPI_CR1_RXONLY_Msk)
	{
		spi_transfer_it_full_duplex_rxonly(transfer);
	}
	else
	{
		spi_transfer_it_full_duplex(transfer);
	}

}

/******************************************************************************
* Function: spi_iqr_handler()
*//**
* \b Description:
*
*	Calls the appropriate callback function (registered during spi_transfer_it) and feeds it a safe copy
*	of the desired transfer (made during spi_transfer_it).
*
* PRE-CONDITION: spi_transfer_it has been called and set up on the desired channel
* POST-CONDITION: The callback has been called and has handled a single reception/transfer/end of transfer
*
* @return 		void
*
* \b Example:
*  Called from withn the hardware defined irqs defined in the vector table
* @code
* SPI3_IRQHandler()
* {
* 	spi_irq_handler(SPI_3);
* }
* @endcode
*
* @see spi_transfer_it
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
void spi_irq_handler(spi_channel_t channel)
{
	spi_transfer_t *transfer = &spi_interrupt_transfers[channel];
	if (spi_interrupt_callbacks[channel] != NULL)
	{
		spi_interrupt_callbacks[channel](transfer);
	}
}


/******************************************************************************
* Function: spi_register_write()
*//**
* \b Description:
*
*	Write the desired value into the register in spi address space
*
* PRE-CONDITION: the spi_register is within spi address space
* POST-CONDITION: the spi_register contains the desired value
* @return 		void
*
* \b Example:
* @code
* uint16_t new_value = 0xFF2A;
* spi_register_write(SPI1_BASE + 0x20UL, new_value);
* @endcode
*
* @see spi_register_read
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
void spi_register_write(uint32_t spi_register, uint16_t value)
{
	assert(		(spi_register >= SPI2_BASE && spi_register < I2S3ext_BASE)
			||  (spi_register >= SPI1_BASE && spi_register < SYSCFG_BASE)
			|| 	(spi_register >= SPI5_BASE && spi_register < GPIOA_BASE));

	*((uint16_t *)spi_register) = value;
}

/******************************************************************************
* Function: spi_register_read()
*//**
* \b Description:
*
*	Reads the current value from the register in spi address space
*
* PRE-CONDITION: the spi_register is within spi address space
* POST-CONDITION: Returns the value within the register
*
* @return 		uint16_t
*
* \b Example:
* @code
* uin16_t current_value = spi_register_write(SPI1_BASE + 0x20UL);
* @endcode
*
* @see spi_register_write
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
uint16_t spi_register_read(uint32_t spi_register)
{
	assert(		(spi_register >= SPI2_BASE && spi_register < I2S3ext_BASE)
			||  (spi_register >= SPI1_BASE && spi_register < SYSCFG_BASE)
			|| 	(spi_register >= SPI5_BASE && spi_register < GPIOA_BASE));

	return ( *((uint16_t *)spi_register));
}

/******************************************************************************
* Function: spi_transfer_bidir()
*//**
* \b Description:
*
*	A static function called auomatically by spi_transfer when appropriate
*	which subsequently simply calls the appropriate transfer subroutine
*	based on transfer direction
*
* PRE-CONDITION: (Soft assertion) The pointer to the data buffer to be transferred
* 					is non-NULL. Failure of this check results in an uneventful return
*
* POST-CONDITION: The data has been transferred as specified in the transfer parameter
* 					by the subroutine.
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called automatically by spi_transfer when BIDIMODE == 1
*
* @see spi_transfer_bidir_transmit
* @see spi_transfer_bidir_receive
* @see spi_transfer_full_duplex_rxonly
* @see spi_transfer_full_duplex
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_bidir(spi_transfer_t *transfer)
{
	uint16_t CR1_state = *SPI_CR1[transfer->channel];

	if ((CR1_state & SPI_CR1_BIDIOE_Msk) != 0)
	{
		if (transfer->tx_buffer == NULL)
		{
			return;
		}
		spi_transfer_bidir_transmit(transfer);
	}
	else
	{
		if (transfer->rx_buffer == NULL)
		{
			return;
		}
		spi_transfer_bidir_receive(transfer);
	}
}

/******************************************************************************
* Function: spi_transfer_bidir_transmit()
*//**
* \b Description:
*
*	A static function called automatically by spi_transfer_bidir when the single
*	data line is being used to send information
*
* PRE-CONDITION: (Soft assertion) The length of the data buffer is non-NULL.
* 					Failure of this check simply ends the function
*
* POST-CONDITION: Transmits the data in tx_buffer to the selected slave
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called automatically by spi_transfer when BIDIMODE == 1 && BIDIOE != 0
*
* @see spi_transfer_bidir
* @see spi_transfer_bidir_receive
* @see spi_transfer_bidir_it
* @see spi_transfer_bidir_transmit_it
* @see spi_transfer_bidir_receive_it
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_bidir_transmit(spi_transfer_t *transfer)
{
	uint16_t SR_state;
	*SPI_DR[transfer->channel] = *transfer->tx_buffer;
	transfer->tx_buffer++;
	transfer->tx_length--;
	while(transfer->tx_length > 0)
	{
		do
		{
			SR_state = *SPI_SR[transfer->channel];
		} while((SR_state & SPI_SR_TXE_Msk) == 0);

		*SPI_DR[transfer->channel] = *transfer->tx_buffer;
		transfer->tx_buffer++;
		transfer->tx_length--;
	}
	do
	{
		SR_state = *SPI_SR[transfer->channel];
	} while((SR_state & SPI_SR_TXE_Msk) == 0);

	do
	{
		SR_state = *SPI_SR[transfer->channel];
	} while((SR_state & SPI_SR_BSY_Msk) != 0);
}

/******************************************************************************
* Function: spi_transfer_bidir_receive()
*//**
* \b Description:
*
*	A static function called automatically by spi_transfer_bidir when the single
*	data line is being used to receive information
*
* PRE-CONDITION: (Soft assertion) The length of the data buffer is non-NULL.
* 					Failure of this check simply skips the function
*
* POST-CONDITION: Places received spi data in the rx_buffer
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called automatically by spi_transfer when BIDIMODE == 1 && BIDIOE == 0
*
* @see spi_transfer_bidir
* @see spi_transfer_bidir_transmit
* @see spi_transfer_bidir_it
* @see spi_transfer_bidir_transmit_it
* @see spi_transfer_bidir_receive_it
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_bidir_receive(spi_transfer_t *transfer)
{
	uint16_t SR_state;
	while(transfer->rx_length > 0)
	{
		do
		{
			SR_state = *SPI_SR[transfer->channel];
		} while((SR_state & SPI_SR_RXNE_Msk) == 0);

		*transfer->rx_buffer = *SPI_DR[transfer->channel];
		transfer->rx_buffer++;
		transfer->rx_length--;
	}
}

/******************************************************************************
* Function: spi_transfer_full_duplex_rxonly()
*//**
* \b Description:
*
*	A static function called automatically by spi_transfer when only the MISO line
*	is being used to receive data
*
* PRE-CONDITION: (Soft assertion) The address and length of the data buffer are non-NULL.
* 					Failure of this check simply results in a meaningless return
*
* POST-CONDITION: The received data is placed in rx_buffer
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called automatically by spi_transfer when RX_ONLY == 1
*
* @see spi_transfer
* @see spi_transfer_bidir
* @see spi_transfer_full_duplex
*
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_full_duplex_rxonly(spi_transfer_t *transfer)
{
	if (transfer->rx_buffer == NULL)
	{
		return;
	}
	uint16_t SR_state;
	while(transfer->rx_length > 0)
	{
		do
		{
			SR_state = *SPI_SR[transfer->channel];
		}while((SR_state & SPI_SR_RXNE_Msk) == 0);
		*transfer->rx_buffer = *SPI_DR[transfer->channel];
		transfer->rx_buffer++;
		transfer->rx_length--;
	}
	do
	{
		SR_state = *SPI_SR[transfer->channel];
	} while((SR_state & SPI_SR_BSY) != 0);
}

/******************************************************************************
* Function: spi_transfer_full_duplex()
*//**
* \b Description:
*
*	A static function called automatically by spi_transfer when the spi is configured
*	with two data lines. It boots a subroutine dependent on whether or not the spi is
*	configured in master or slave mode
*
* PRE-CONDITION: (Soft assertion) The address and length of the data buffer are non-NULL.
* 					Failure of this check simply results in a meaningless return
*
* POST-CONDITION: All data in the tx_buffer has been sent to the selected slave
* POST-CONDITION: All received data has been placed in the rx_buffer
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called automatically by spi_transfer when no other special transfer modes are valid
*
* @see spi_transfer
* @see spi_transfer_bidir
* @see spi_transfer_full_duplex_rxonly
*
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_full_duplex(spi_transfer_t *transfer)
{
	if (transfer->tx_buffer == NULL || transfer->rx_buffer == NULL)
		return;
	uint16_t CR1_state = *SPI_CR1[transfer->channel];


	if ((CR1_state & SPI_CR1_MSTR_Msk) != 0)
	{
		spi_transfer_full_duplex_master(transfer);
	}
	else
	{
		spi_transfer_full_duplex_slave(transfer);
	}
}

/******************************************************************************
* Function: spi_transfer_full_duplex_master()
*//**
* \b Description:
*
*	A static function called automatically by spi_transfer_full_duplex when the spi
*	is configured as a master with two data lines.
*
* PRE-CONDITION: The tx_buffer is non-NULL and of non-zero length
* PRE-CONDITION: The rx_buffer is non-NULL and of non-zero length
*
* POST-CONDITION: All data in the tx_buffer has been sent to the selected slave
* POST-CONDITION: All received data has been placed in the rx_buffer
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called automatically by spi_transfer when no other special transfer modes are valid
*
* @see spi_transfer
* @see spi_transfer_full_duplex
* @see spi_transfer_full_duplex_slave
*
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_full_duplex_master(spi_transfer_t *transfer)
{
	assert(transfer->tx_buffer != NULL && transfer->tx_length != 0);
	assert(transfer->rx_buffer != NULL && transfer->rx_length != 0);
	uint16_t SR_state;
	*SPI_DR[transfer->channel] = *transfer->tx_buffer;
	transfer->tx_buffer++;
	transfer->tx_length--;
	while(transfer->rx_length > 1)
	{
		do
		{
			SR_state = *SPI_SR[transfer->channel];
		} while((SR_state & SPI_SR_TXE_Msk) == 0);

		*SPI_DR[transfer->channel] = *transfer->tx_buffer;
		transfer->tx_buffer++;
		transfer->tx_length--;

		do
		{
			SR_state = *SPI_SR[transfer->channel];
		} while((SR_state & SPI_SR_RXNE_Msk) == 0);

		*transfer->rx_buffer = *SPI_DR[transfer->channel];
		transfer->rx_buffer++;
		transfer->rx_length--;
	}

	do
	{
		SR_state = *SPI_SR[transfer->channel];
	} while((SR_state & SPI_SR_RXNE_Msk) == 0);

	*transfer->rx_buffer = *SPI_DR[transfer->channel];
	transfer->rx_buffer++;
	transfer->rx_length--;

	do
	{
		SR_state = *SPI_SR[transfer->channel];
	} while((SR_state & SPI_SR_TXE_Msk) == 0);

	do
	{
		SR_state = *SPI_SR[transfer->channel];
	} while((SR_state & SPI_SR_BSY_Msk) != 0);
}

/******************************************************************************
* Function: spi_transfer_full_duplex_slave()
*//**
* \b Description:
*
*	A static function called automatically by spi_transfer_full_duplex when the spi
*	is configured as a slave with two data lines.
*
* PRE-CONDITION: The tx_buffer is non-NULL and of non-zero length
* PRE-CONDITION: The rx_buffer is non-NULL and of non-zero length
*
* POST-CONDITION: All data in the tx_buffer has been sent to the master
* POST-CONDITION: All received data has been placed in the rx_buffer
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called automatically by spi_transfer when no other special transfer modes are valid
*
* @see spi_transfer
* @see spi_transfer_full_duplex
* @see spi_transfer_full_duplex_master
*
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_full_duplex_slave(spi_transfer_t *transfer)
{
	uint16_t SR_state;
	do
			{
				SR_state = *SPI_SR[transfer->channel];
			} while((SR_state & SPI_SR_RXNE_Msk) == 0);

			*transfer->rx_buffer = *SPI_DR[transfer->channel];
			transfer->rx_buffer++;
			transfer->rx_length--;

			while(transfer->tx_length > 1)
			{
				do
				{
					SR_state = *SPI_SR[transfer->channel];
				} while((SR_state & SPI_SR_RXNE_Msk) == 0);

				*transfer->rx_buffer = *SPI_DR[transfer->channel];
				transfer->rx_buffer++;
				transfer->rx_length--;
				do
				{
					SR_state = *SPI_SR[transfer->channel];
				} while((SR_state & SPI_SR_TXE_Msk) == 0);

				*SPI_DR[transfer->channel] = *transfer->tx_buffer;
				transfer->tx_buffer++;
				transfer->tx_length--;
			}

			do
			{
				SR_state = *SPI_SR[transfer->channel];
			} while((SR_state & SPI_SR_TXE_Msk) == 0);

			*SPI_DR[transfer->channel] = *transfer->tx_buffer;
			transfer->tx_buffer++;
			transfer->tx_length--;

			do
			{
				SR_state = *SPI_SR[transfer->channel];
			}while((SR_state & SPI_SR_RXNE_Msk) == 0);

			do
			{
				SR_state = *SPI_SR[transfer->channel];
			}while((SR_state & SPI_SR_BSY_Msk) != 0);
}

/******************************************************************************
* Function: spi_transfer_it_bidir_transmit_callback()
*//**
* \b Description:
*
*	A static callback mapped to the irq handler by spi_transfer_bidir_it and
*	called by the irq handler. Handles the transmission of a single
*	data unit or ends the communication and releases the slave.
*
* PRE-CONDITION: The tx_buffer is of non-zero length
*
* POST-CONDITION: A single data unit has been sent to the target
* OR
* POST-CONDITION: The communication has been ended and the slave released
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Registered when appropriate by spi_transfer_it_bidir and called
*	by the spi_irq_handler
*
* @see spi_transfer_it
* @see spi_transfer_it_bidir
* @see spi_irq_handler
* @see spi_transfer_it_full_duplex
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_it_bidir_transmit_callback(spi_transfer_t *transfer)
{
	uint16_t SR_state = *SPI_SR[transfer->channel];
	if (transfer->tx_length > 0 && (SR_state & SPI_SR_TXE_Msk))
	{
		*SPI_DR[transfer->channel] = *transfer->tx_buffer;
		transfer->tx_buffer++;
		transfer->tx_length--;
	}
	else
	{
		*SPI_CR2[transfer->channel] &= ~(SPI_CR2_TXEIE_Msk);
		*SPI_CR1[transfer->channel] &= ~(SPI_CR1_SPE_Msk);
		spi_release_slave(transfer);
	}
}

/******************************************************************************
* Function: spi_transfer_it_bidir_receive_callback()
*//**
* \b Description:
*
*	A static callback mapped to the irq handler by spi_transfer_bidir_it and
*	called by the irq handler. Handles the reception of a single
*	data unit or ends the communication and releases the slave.
*
* PRE-CONDITION: The rx_buffer is of non-zero length
*
* POST-CONDITION: A single data unit has been received from the target
* OR
* POST-CONDITION: The communication has been ended and the slave released
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Registered when appropriate by spi_transfer_it_bidir and called
*	by the spi_irq_handler
*
* @see spi_transfer_it
* @see spi_transfer_it_bidir
* @see spi_transfer_it_full_duplex
* @see spi_irq_handler
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_it_bidir_receive_callback(spi_transfer_t *transfer)
{
	uint16_t SR_state = *SPI_SR[transfer->channel];
	if (transfer->rx_length > 0 && (SR_state & SPI_SR_RXNE_Msk))
	{
		*transfer->rx_buffer = *SPI_DR[transfer->channel];
		transfer->rx_buffer++;
		transfer->rx_length--;
	}
	else
	{
		*SPI_CR2[transfer->channel] &= ~(SPI_CR2_RXNEIE_Msk);
		*SPI_CR1[transfer->channel] &= ~(SPI_CR1_SPE_Msk);
		spi_release_slave(transfer);
	}
}

/******************************************************************************
* Function: spi_transfer_it_bidir()
*//**
* \b Description:
*
*	Maps the appropriate bidir callback and makes a safe copy of the transfer
*	structure.
*
* PRE-CONDITION: (Soft Assert) The tx_buffer is non-NULL and of non-zero length
* OR
* PRE-CONDITION: (Soft Assert) The rx_buffer is non-NULL and of non-zero length
*
* POST-CONDITION: The correct callback function has been mapped to the file-scope
* 					callback array
* POST-CONDITION: The correct buffer interrupt (RXNEIE or TXEIE) has been enabled
* POST-CONDITION: The SPI Enable (SPE) has been switched on
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called by spi_transfer_it when BIDIMODE == 1
*
*
* @see spi_transfer_it
* @see spi_transfer_it_bidir_transmit_callback
* @see spi_transfer_it_bidir_receive_callback
* @see spi_transfer_it_full_duplex
*
* @see spi_irq_handler
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_it_bidir(spi_transfer_t *transfer)
{
	uint16_t CR1_state = *SPI_CR1[transfer->channel];
	if ((CR1_state & SPI_CR1_BIDIOE_Msk) != 0)
	{
		if (transfer->tx_buffer == NULL || transfer->tx_length == 0)
		{
			return;
		}

		spi_interrupt_callbacks[transfer->channel] = spi_transfer_it_bidir_transmit_callback;
		*SPI_CR2[transfer->channel] |= SPI_CR2_TXEIE_Msk;
	}
	else
	{
		if (transfer->rx_buffer == NULL || transfer->rx_length == 0)
		{
			return;
		}
		spi_interrupt_callbacks[transfer->channel] = spi_transfer_it_bidir_receive_callback;
		*SPI_CR2[transfer->channel] |= SPI_CR2_RXNEIE_Msk;
	}

	*SPI_CR1[transfer->channel] |= SPI_CR1_SPE_Msk;

}

/******************************************************************************
* Function: spi_transfer_it_full_duplex_rxonly_callback()
*//**
* \b Description:
*
*	A callback function called by the irq handler which manages the reception
*	of a single data unit when the spi has two data lines.
*
* PRE-CONDITION: (Soft Assert) The rx_buffer is of non-zero length
*
* POST-CONDITION: A single data unit has been received
* OR
* POST-CONDITION: The communication has been shut down and the slave released
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called by the irq_handler if it's mapped
*
*
* @see spi_transfer_it
* @see spi_transfer_it_bidir
* @see spi_transfer_it_full_duplex
* @see spi_irq_handler
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_it_full_duplex_rxonly_callback(spi_transfer_t *transfer)
{
	uint16_t SR_state = *SPI_SR[transfer->channel];
	if (transfer->rx_length > 0 && (SR_state & SPI_SR_RXNE_Msk))
	{
		*transfer->rx_buffer = *SPI_DR[transfer->channel];
		transfer->rx_buffer++;
		transfer->rx_length--;
	}
	else
	{
		*SPI_CR2[transfer->channel] &= ~(SPI_CR2_RXNEIE_Msk);
		*SPI_CR1[transfer->channel] &= ~(SPI_CR1_SPE_Msk);
		spi_release_slave(transfer);
	}
}

/******************************************************************************
* Function: spi_transfer_it_full_duplex_rxonly()
*//**
* \b Description:
*
*	Registers the rxonly callback and makes a safe copy of the transfer structure.
*
* PRE-CONDITION: The rx buffer is non-NULL and of non-zero length

* POST-CONDITION: The rxonly callback has been mapped to the right spi device
* POST-CONDITION: The RXNEIE interrupt has been enabled
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called by spi_transfer_it when RXONLY == 1
*
*
* @see spi_transfer_it
* @see spi_transfer_it_bidir
* @see spi_transfer_it_full_duplex
* @see spi_irq_handler
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_it_full_duplex_rxonly(spi_transfer_t *transfer)
{
	assert(transfer->rx_buffer != NULL && transfer->rx_length != 0);
	spi_interrupt_callbacks[transfer->channel] = spi_transfer_it_full_duplex_rxonly_callback;
	*SPI_CR2[transfer->channel] |= SPI_CR2_RXNEIE_Msk;
	*SPI_CR1[transfer->channel] |= SPI_CR1_SPE_Msk;
}

/******************************************************************************
* Function: spi_transfer_it_full_duplex_callback()
*//**
* \b Description:
*
*	A callback function called by the irq handler which manages the reception and
*	transmission of a single data unit when the spi has two data lines.
*
* PRE-CONDITION: (Soft Assert) The rx_buffer is of non-zero length
* PRE-CONDITION: (Soft Assert) The tx_buffer is of non-zero length
*
* POST-CONDITION: A single data unit has been received and another sent
* OR
* POST-CONDITION: The communication has been shut down and the slave released
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called by the irq_handler if it's mapped
*
*
* @see spi_transfer_it
* @see spi_transfer_it_bidir
* @see spi_transfer_it_full_duplex_rxonly
* @see spi_irq_handler
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_it_full_duplex_callback(spi_transfer_t *transfer)
{
	uint16_t SR_state = *SPI_SR[transfer->channel];
	if (transfer->tx_length > 0 && (SR_state & SPI_SR_TXE_Msk))
	{
		*SPI_DR[transfer->channel] = *transfer->tx_buffer;
		transfer->tx_buffer++;
		transfer->tx_length--;
	}

	else if (transfer->rx_length > 0 && (SR_state & SPI_SR_RXNE_Msk))
	{
		*transfer->rx_buffer = *SPI_DR[transfer->channel];
		transfer->rx_buffer++;
		transfer->rx_length--;
	}
	else
	{
		*SPI_CR2[transfer->channel] &= ~(SPI_CR2_TXEIE_Msk | SPI_CR2_RXNEIE_Msk);
		*SPI_CR1[transfer->channel] &= ~(SPI_CR1_SPE_Msk);
		spi_release_slave(transfer);
	}
}

/******************************************************************************
* Function: spi_transfer_it_full_duplex()
*//**
* \b Description:
*
*	Maps the correct callback to the callback array and enables the correct
*	interrupt flags
*
* PRE-CONDITION: None
*
* POST-CONDITION: The reception and transmission (RXNEIE and TXEIE) interrupts have
* 					been enabled
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called by spi_transfer_it when full duplex configuration is selected
*
*
* @see spi_transfer_it
* @see spi_transfer_it_bidir
* @see spi_transfer_it_full_duplex_rxonly
* @see spi_irq_handler
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_transfer_it_full_duplex(spi_transfer_t *transfer)
{
	spi_interrupt_callbacks[transfer->channel] = spi_transfer_it_full_duplex_callback;
	*SPI_CR2[transfer->channel] |= SPI_CR2_TXEIE_Msk | SPI_CR2_RXNEIE_Msk;
	*SPI_CR1[transfer->channel] |= SPI_CR1_SPE_Msk;
}

/******************************************************************************
* Function: spi_select_slave()
*//**
* \b Description:
*
*	Static function used to select a slave (whether it is active high or low) from
*	within transfer functions
*
* PRE-CONDITION: The GPIO pin for controlling the slave has been correctly configured
*
* POST-CONDITION: The GPIO output is at the correct level to select the slave
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called by spi_transfer and spi_transfer_it in master mode before transmission
*	begins
*
*
* @see spi_release_slave
* @see spi_transfer
* @see spi_transfer_it
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_select_slave(spi_transfer_t *transfer)
{
	if (transfer->ss_polarity == SS_ACTIVE_LOW)
	{
		gpio_pin_write(transfer->slave_pin, GPIO_PIN_LOW);
	}
	else if (transfer->ss_polarity == SS_ACTIVE_HIGH)
	{
		gpio_pin_write(transfer->slave_pin, GPIO_PIN_HIGH);
	}
}

/******************************************************************************
* Function: spi_release_slave()
*//**
* \b Description:
*
*	Static function used to release a slave (whether it is active high or low) from
*	within transfer functions
*
* PRE-CONDITION: The GPIO pin for controlling the slave has been correctly configured
*
* POST-CONDITION: The GPIO output is at the correct level to release the slave
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called by spi_transfer and spi_transfer_it and callbacks in master mode
*
*
* @see spi_select_slave
* @see spi_transfer
* @see spi_transfer_it
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_release_slave(spi_transfer_t *transfer)
{
	if (transfer->ss_polarity == SS_ACTIVE_LOW)
	{
		gpio_pin_write(transfer->slave_pin, GPIO_PIN_HIGH);
	}
	else if (transfer->ss_polarity == SS_ACTIVE_HIGH)
	{
		gpio_pin_write(transfer->slave_pin, GPIO_PIN_LOW);
	}
}

/******************************************************************************
* Function: spi_configure_clock()
*//**
* \b Description:
*
*	Static function used to configure the clock phase and polarity to be used
*	during the spi communication
*
* PRE-CONDITION: the clock_polarity member of the transfer structure is valid
* PRE-CONDITION: the clock_phase member of the transfer structure is valid
*
* POST-CONDITION: The CR1 registers now contain the desired clock configuration
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called by spi_transfer and spi_transfer_it and callbacks
*
*
* @see spi_configure_data_frame
* @see spi_transfer
* @see spi_transfer_it
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_configure_clock(spi_transfer_t *transfer)
{
	if (transfer->clock_polarity == ACTIVE_HIGH)
	{
		*SPI_CR1[transfer->channel] &= ~(SPI_CR1_CPOL_Msk);
	}
	else if (transfer->clock_polarity == ACTIVE_LOW)

	{
		*SPI_CR1[transfer->channel] |= (SPI_CR1_CPOL_Msk);
	}

	if (transfer->clock_phase == FIRST_EDGE)
	{
		*SPI_CR1[transfer->channel] &= ~(SPI_CR1_CPHA_Msk);
	}
	else if (transfer->clock_phase == SECOND_EDGE)
	{
		*SPI_CR1[transfer->channel] |= (SPI_CR1_CPHA_Msk);
	}
}

/******************************************************************************
* Function: spi_configure_data_frame()
*//**
* \b Description:
*
*	Static function used to configure the the data size and bit format to be used
*	during the current transfer
*
* PRE-CONDITION: the data_format member of the transfer structure is valid
* PRE-CONDITION: the bit_format member of the transfer structure is valid
*
* POST-CONDITION: The CR1 registers now contain the desired data format configuration
*
* @param		transfer a pointer to the transfer structure containing all relevant
* 					information for the transmission
* @return 		void
*
* \b Example:
*	Called by spi_transfer and spi_transfer_it and callbacks
*
*
* @see spi_configure_clock
* @see spi_transfer
* @see spi_transfer_it
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static void spi_configure_data_frame(spi_transfer_t *transfer)
{
	if (transfer->data_format == SPI_DATA_8BIT)
	{
		*SPI_CR1[transfer->channel] &= ~(SPI_CR1_DFF_Msk);
	}
	else if (transfer->data_format == SPI_DATA_16BIT)
	{
		*SPI_CR1[transfer->channel] |= SPI_CR1_DFF_Msk;
	}

	if (transfer->bit_format == MSB_FIRST)
	{
		*SPI_CR1[transfer->channel] &= ~(SPI_CR1_LSBFIRST_Msk);
	}
	else if (transfer->bit_format == LSB_FIRST)
	{
		*SPI_CR1[transfer->channel] |= SPI_CR1_LSBFIRST_Msk;
	}
}


