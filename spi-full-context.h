#ifndef SPI_CONTEXT_H
#define SPI_CONTEXT_H

#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>
#include <stdint.h>

#define SPI_DEVICE_TEMPLATE		"/dev/spidev%d.%d"
#define DEFAULT_SPI_BUS			1
#define DEFAULT_SPI_CS_LINE		0
#define DEFAULT_SPI_MODE		SPI_MODE_1
#define DEFAULT_SPI_BITS_PER_WORD	8
#define DEFAULT_SPI_SPEED		1000000
#define DEFAULT_SPI_DELAY_USECS		0

struct spi_config {
	int bus;
	int cs_line;
	uint8_t mode;
	uint32_t speed;
	uint8_t bits;
	uint16_t delay;
};

static const struct spi_config default_spi_config = {
	.bus		= DEFAULT_SPI_BUS,
	.cs_line	= DEFAULT_SPI_CS_LINE,
	.mode		= DEFAULT_SPI_MODE,
	.speed		= DEFAULT_SPI_SPEED,
	.bits		= DEFAULT_SPI_BITS_PER_WORD,
	.delay		= DEFAULT_SPI_DELAY_USECS,
};

struct spi_ctx {
	int fd;
	int power_en;
	int start_en;
	int reset;
	int led;
	int plug;
	struct spi_config config;
};

/* create SPI context with given configuration, returns NULL on failure */
extern struct spi_ctx *spi_init(struct spi_config *config);
/* close descriptor and free resources */
extern void spi_exit(struct spi_ctx *ctx);
/* process RX/TX transfer, ensure buffers are long enough */
extern bool spi_transfer(struct spi_ctx *ctx, uint8_t *txbuf,
			 uint8_t *rxbuf, int len);
uint32_t set_spi_wrspeed(struct spi_ctx *spi_ctx, uint32_t speed);

#endif /* SPI_CONTEXT_H */

