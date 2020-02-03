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
* Function: spi_transfer()
*//**
* \b Description:
*
* 	Sets up an interrupt based spi transfer according to the specifications of
* 	 the transfer parameter.
*
*
*
* PRE-CONDITION: spi_init() has been successfully carried out for the required spi channel
* PRE-CONDITION: gpio_init() has been called for the slave select pin to configure it as an output/input,
* 					depending on desired direction
* PRE-CONDITION: The proper data buffers and lengths are non-null/non-zero
*
* POST-CONDITION: The irq handler will now handle the rest of the transfer
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

static void spi_transfer_full_duplex_master(spi_transfer_t *transfer)
{
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

static void spi_transfer_it_bidir(spi_transfer_t *transfer)
{
	uint16_t CR1_state = *SPI_CR1[transfer->channel];
	if ((CR1_state & SPI_CR1_BIDIOE_Msk) != 0)
	{
		if (transfer->tx_buffer == NULL || transfer->tx_length == 0)
		{
			return;
		}
		spi_interrupt_transfers[transfer->channel] = *transfer;
		spi_interrupt_callbacks[transfer->channel] = spi_transfer_it_bidir_transmit_callback;
		*SPI_CR2[transfer->channel] |= SPI_CR2_TXEIE_Msk;
	}
	else
	{
		if (transfer->rx_buffer == NULL || transfer->rx_length == 0)
		{
			return;
		}
		spi_interrupt_transfers[transfer->channel] = *transfer;
		spi_interrupt_callbacks[transfer->channel] = spi_transfer_it_bidir_receive_callback;
		*SPI_CR2[transfer->channel] |= SPI_CR2_RXNEIE_Msk;
	}

	*SPI_CR1[transfer->channel] |= SPI_CR1_SPE_Msk;

}

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

static void spi_transfer_it_full_duplex_rxonly(spi_transfer_t *transfer)
{
	spi_interrupt_transfers[transfer->channel] = *transfer;
	spi_interrupt_callbacks[transfer->channel] = spi_transfer_it_full_duplex_rxonly_callback;
	*SPI_CR2[transfer->channel] |= SPI_CR2_RXNEIE_Msk;
	*SPI_CR1[transfer->channel] |= SPI_CR1_SPE_Msk;
}

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

static void spi_transfer_it_full_duplex(spi_transfer_t *transfer)
{
	spi_interrupt_transfers[transfer->channel] = *transfer;
	spi_interrupt_callbacks[transfer->channel] = spi_transfer_it_full_duplex_callback;
	*SPI_CR2[transfer->channel] |= SPI_CR2_TXEIE_Msk | SPI_CR2_RXNEIE_Msk;
	*SPI_CR1[transfer->channel] |= SPI_CR1_SPE_Msk;
}


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


