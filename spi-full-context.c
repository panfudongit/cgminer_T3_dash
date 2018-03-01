/*
 * generic SPI functions
 *
 * Copyright 2017  pang fu dong
 *
 */

#include "spi-full-context.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "miner.h"

static pthread_mutex_t spi_lock;

#define SPI_PATH_MAX 50

uint32_t set_spi_wrspeed(struct spi_ctx *spi_ctx, uint32_t speed)
{
	struct spi_config *config = &spi_ctx->config;

	if (config == NULL)
		return false;

	config->speed = speed;

	if ((ioctl(spi_ctx->fd, SPI_IOC_WR_MODE32, &config->mode) < 0) ||
	    (ioctl(spi_ctx->fd, SPI_IOC_RD_MODE32, &config->mode) < 0) ||
	    (ioctl(spi_ctx->fd, SPI_IOC_WR_BITS_PER_WORD, &config->bits) < 0) ||
	    (ioctl(spi_ctx->fd, SPI_IOC_RD_BITS_PER_WORD, &config->bits) < 0) ||
	    (ioctl(spi_ctx->fd, SPI_IOC_WR_MAX_SPEED_HZ, &config->speed) < 0) ||
	    (ioctl(spi_ctx->fd, SPI_IOC_RD_MAX_SPEED_HZ, &config->speed) < 0)) {
		close(spi_ctx->fd);
		return false;
	}
	return true;
}

struct spi_ctx *spi_init(struct spi_config *config)
{
	char dev_fname[SPI_PATH_MAX];
	struct spi_ctx *ctx;

	if (config == NULL)
		return NULL;

	sprintf(dev_fname, SPI_DEVICE_TEMPLATE, config->bus, config->cs_line);

	int fd = open(dev_fname, O_RDWR);
	if (fd < 0) {
		return NULL;
	}

	if ((ioctl(fd, SPI_IOC_WR_MODE32, &config->mode) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_MODE32, &config->mode) < 0) ||
	    (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &config->bits) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &config->bits) < 0) ||
	    (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &config->speed) < 0) ||
	    (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &config->speed) < 0)) {
		close(fd);
		return NULL;
	}

	ctx = malloc(sizeof(*ctx));
	assert(ctx != NULL);

	ctx->fd = fd;
	ctx->config = *config;
	applog(LOG_WARNING, "SPI '%s': mode=%hhu, bits=%hhu, speed=%u",
       dev_fname, ctx->config.mode, ctx->config.bits,
       ctx->config.speed);
	return ctx;
}

extern void spi_exit(struct spi_ctx *ctx)
{
	if (NULL == ctx)
		return;

	close(ctx->fd);
	free(ctx);
}

extern bool spi_transfer(struct spi_ctx *ctx, uint8_t *txbuf,
			 uint8_t *rxbuf, int len)
{
	mutex_lock(&spi_lock);
	if (rxbuf != NULL)
		memset(rxbuf, 0xff, len);

	struct spi_ioc_transfer xfr = {
                .tx_buf = (unsigned long)txbuf,
                .rx_buf = (unsigned long)rxbuf,
                .len = len,
                .speed_hz = ctx->config.speed,
                .delay_usecs = ctx->config.delay,
                .bits_per_word = ctx->config.bits,
                .cs_change = 0,
                .pad = 0,
         };
	int ret;

	ret = len;

	ret = ioctl(ctx->fd, SPI_IOC_MESSAGE(1), &xfr);

	mutex_unlock(&spi_lock);
	return ret > 0;
}

uint8_t default_tx[] = {
	0xc0, 0xf6, 0x7F, 0x45, 0x3a, 0x43,
	0xc0, 0xf6, 0x7F, 0x45, 0x3a, 0x43,
};

uint8_t default_rx[ARRAY_SIZE(default_tx)] = {0, };

void test_spi(void)
{
	struct spi_config cfg = default_spi_config;
	struct spi_ctx *spi;

	spi = spi_init(&cfg);
	if (spi == NULL)
	{
		printf("SPI initialization failed\n");
		return;
	}

	if(!spi_transfer(spi, default_tx, default_rx, sizeof(default_tx)))
	{
		printf("SPI spi_transfer failed\n");
		return;
	}

	spi_exit(spi);
}
