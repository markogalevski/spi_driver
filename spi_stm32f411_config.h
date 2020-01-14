#ifndef _SPI_STM32F411_CONFIG
#define _SPI_STM32F411_CONFIG

typedef enum
{
	SPI_1 = 0x00UL,
	SPI_2 = 0x01UL,
	SPI_3 = 0x02UL,
	SPI_4 = 0x03UL,
	SPI_5 = 0x04UL,
	NUM_SPI = 0x05UL
}spi_channel_t;


typedef enum
{
	SPI_SLAVE,
	SPI_MASTER
}spi_master_slave_t;

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

typedef enum
{
	SPI_DISABLE,
	SPI_ENABLE
}spi_enable_t;


typedef enum
{
	HARDWARE_SMM,
	SOFTWARE_SMM
}spi_slave_mgmt_t;

typedef enum
{
	UNIDIR_FULL_DUPLEX,
	UNIDIR_RXONLY,
	BIDIR_MODE
}spi_bidir_t;

typedef enum
{
	CRC_DISABLE,
	CRC_ENABLE
}spi_crc_en_t;

typedef enum
{
	RX_DMA_REQ_DISABLE,
	RX_DMA_REQ_ENABLE
}spi_rx_dma_t;

typedef enum
{
	TX_DMA_REQ_DISABLE,
	TX_DMA_REQ_ENABLE
}spi_tx_dma_t;

typedef enum
{
	SS_OUTPUT_DISABLE,
	SS_OUTPUT_ENABLE
}spi_ssoe_t;

typedef enum
{
	SPI_MOTOROLA,
	SPI_TI
}spi_frame_format_t;

typedef enum
{
	ERROR_INTERRUPT,
	RXNE_INTERRUPT,
	TXE_INTERRUPT
}spi_interrupt_t;

typedef struct
{
	spi_enable_t spi_enable;
	spi_master_slave_t master_slave;
	spi_slave_mgmt_t slave_management;
	spi_bidir_t	bidirectional_mode;
	spi_baud_rate_t baud_rate;
}spi_config_t;


const spi_config_t *spi_config_table_get(void);


#endif
