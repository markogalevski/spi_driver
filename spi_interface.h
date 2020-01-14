#ifndef _SPI_H
#define _SPI_H

#include "spi_stm32f411_config.h"
#include "gpio_interface.h"
#include <stdint.h>

typedef enum
{
	SS_ACTIVE_LOW,
	SS_ACTIVE_HIGH
}spi_ss_polarity_t;

typedef enum
{
	ACTIVE_HIGH,
	ACTIVE_LOW
}spi_clock_polarity_t;

typedef enum
{
	FIRST_EDGE,
	SECOND_EDGE
}spi_clock_phase_t;

typedef enum
{
	MSB_FIRST,
	LSB_FIRST
}spi_bit_format_t;

typedef enum
{
	SPI_DATA_8BIT,
	SPI_DATA_16BIT
}spi_data_format_t;


typedef enum
{
	BIDIR_RECEIVE,
	BIDIR_TRANSMIT
}spi_bidir_dir_t;

typedef struct
{
	spi_channel_t channel;
	gpio_pin_t slave_pin;
	spi_ss_polarity_t ss_polarity;
	uint16_t *tx_buffer;
	uint32_t tx_length;
	uint16_t *rx_buffer;
	uint32_t rx_length;
	spi_data_format_t data_format;
	spi_bit_format_t bit_format;
	spi_clock_polarity_t clock_polarity;
	spi_clock_phase_t clock_phase;
	spi_bidir_dir_t bidir_direction;
}spi_transfer_t;

void spi_init(spi_config_t *config_table);
void spi_deinit(spi_channel_t instance);
void spi_transfer(spi_transfer_t *transfer);
void spi_transfer_it(spi_transfer_t *transfer);
void spi_irq_handler(spi_channel_t channel);
void spi_register_write(uint32_t spi_register, uint16_t value);
uint16_t spi_register_read(uint32_t spi_register);

#endif
