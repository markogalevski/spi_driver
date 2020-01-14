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


const spi_config_t *spi_config_table_get(void)
{
	return(config_table);
}
