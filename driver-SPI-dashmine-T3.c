/*
 * cgminer SPI driver for Bitmine.ch T2 devices
 *
 * Copyright 2013, 2014 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "spi-full-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"
#include "crc.h"
#include "T2-common.h"

struct spi_config cfg[ASIC_CHAIN_NUM];
struct spi_ctx *spi[ASIC_CHAIN_NUM];
struct T2_chain *chain[ASIC_CHAIN_NUM];

#define DISABLE_CHIP_FAIL_THRESHOLD	3
#define LEAST_CORE_ONE_CHAIN	432
#define RESET_CHAIN_CNT	2

static uint32_t A1Pll1=0;  //120MHz
static uint32_t A1Pll2=0;  //120MHz
static uint32_t A1Pll3=0;  //120MHz
static uint32_t A1Pll4=0;  //120MHz
static int T2spi[10];

int opt_diff=15;

static const uint8_t difficult_Tbl[24][8] = {
	{0x0f, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff},	// 0.0625
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff},	// 1
	{0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x7f},	// 2
	{0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x3f},	// 4
	{0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x1f},	// 8
	{0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x0f},	// 16
	{0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x07},	// 32
	{0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x03},	// 64
	{0x1e, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},	// 256
	{0x1e, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff},	// 512
	{0x1e, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff},	// 1024
	{0x1e, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff},	// 2048
	{0x1e, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff},	// 4096
	{0x1e, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff},	// 8192
	{0x1e, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff},	// 16384
	{0x1e, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff},	// 32768
	{0x1e, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff}	// 65536
};

int CHIP_PIN_POWER_EN[4] = {
115,
27,
};

int CHIP_PIN_START_EN[4] = {
53,
54,
};

int CHIP_PIN_RESET[4] = {
114,
45,
};

/********** work queue */
static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
	if (work == NULL)
		return false;
	struct work_ent *we = malloc(sizeof(*we));
	assert(we != NULL);

	we->work = work;
	INIT_LIST_HEAD(&we->head);
	list_add_tail(&we->head, &wq->head);
	wq->num_elems++;
	return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
	if (wq == NULL)
		return NULL;
	if (wq->num_elems == 0)
		return NULL;
	struct work_ent *we;
	we = list_entry(wq->head.next, struct work_ent, head);
	struct work *work = we->work;

	list_del(&we->head);
	free(we);
	wq->num_elems--;
	return work;
}

/********** temporary helper for hexdumping SPI traffic */
static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[256];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0) {
			applog(level, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(level, "%s", line);
}

static void hexdump(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_DEBUG);
}

static void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
}

static void flush_spi(struct T2_chain *t2)
{
	memset(t2->spi_tx, 0, 64);
	spi_transfer(t2->spi_ctx, t2->spi_tx, t2->spi_rx, 64);
}


/********** upper layer SPI functions */
static uint8_t *exec_cmd(struct T2_chain *t2,
			  uint8_t cmd, uint8_t chip_id,
			  uint8_t *data, uint8_t len,
			  uint8_t resp_len)
{
	int tx_len = 4 + len;
	memset(t2->spi_tx, 0, tx_len);
	t2->spi_tx[0] = cmd;
	t2->spi_tx[1] = chip_id;

	if (data != NULL)
		memcpy(t2->spi_tx + 2, data, len);

	assert(spi_transfer(t2->spi_ctx, t2->spi_tx, t2->spi_rx, tx_len));
	hexdump("send: TX", t2->spi_tx, tx_len);
	hexdump("send: RX", t2->spi_rx, tx_len);

	int poll_len = resp_len;
	if (chip_id == 0) {
		if (t2->num_chips == 0) {
			applog(LOG_INFO, "%d: unknown chips in chain, "
			       "assuming 8", t2->chain_id);
			poll_len += 32;
		}
		poll_len += 4 * t2->num_chips;
	}
	else {
		poll_len += 4 * chip_id - 2;
	}
	assert(spi_transfer(t2->spi_ctx, NULL, t2->spi_rx + tx_len, poll_len));
	hexdump("poll: RX", t2->spi_rx + tx_len, poll_len);
	int ack_len = tx_len + resp_len;
	int ack_pos = tx_len + poll_len - ack_len;
	hexdump("poll: ACK", t2->spi_rx + ack_pos, ack_len - 2);

	return (t2->spi_rx + ack_pos);

}


/********** T2 SPI commands */
static uint8_t *cmd_BIST_FIX_BCAST(struct T2_chain *t2)
{
	uint8_t *ret = exec_cmd(t2, T2_BIST_FIX, 0x00, NULL, 0, 0);
	if (ret == NULL || ret[0] != T2_BIST_FIX) {
		applog(LOG_ERR, "%d: cmd_BIST_FIX_BCAST failed", t2->chain_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_BIST_COLLECT_BCAST(struct T2_chain *t2)
{
	uint8_t *ret = exec_cmd(t2, T2_BIST_COLLECT, 0x00, NULL, 0, 0);
	if (ret == NULL || ret[0] != T2_BIST_COLLECT) {
		applog(LOG_ERR, "%d: cmd_BIST_COLLECT_BCAST failed", t2->chain_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_RESET_BCAST(struct T2_chain *t2, uint8_t strategy)
{
	static uint8_t s[2];
	s[0] = strategy;
	s[1] = strategy;
	uint8_t *ret = exec_cmd(t2, T2_RESET, 0x00, s, 2, 0);
	if (ret == NULL || (ret[0] != T2_RESET_RES && t2->num_chips != 0)) {
		applog(LOG_ERR, "%d: cmd_RESET_BCAST failed", t2->chain_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_RESET_IDUAL(struct T2_chain *t2, uint8_t chip_id)
{
	static uint8_t s[2];
	s[0] = 0xed;
	s[1] = 0xed;
	uint8_t *ret = exec_cmd(t2, T2_RESET, chip_id, s, 2, 0);
	if (ret == NULL || (ret[0] != T2_RESET && ret[1] != chip_id)) {
		applog(LOG_ERR, "%d: cmd_RESET_BCAST failed", t2->chain_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_READ_RESULT_BCAST(struct T2_chain *t2)
{
	int tx_len = 10;
	uint16_t clc_crc;
	uint16_t res_crc;
	memset(t2->spi_tx, 0, tx_len);
	t2->spi_tx[0] = T2_READ_RESULT;

	assert(spi_transfer(t2->spi_ctx, t2->spi_tx, t2->spi_rx, tx_len));
	hexdump("send: TX", t2->spi_tx, tx_len);
	hexdump("send: RX", t2->spi_rx, tx_len);

	int poll_len = tx_len + 4 * t2->num_chips;
	assert(spi_transfer(t2->spi_ctx, NULL, t2->spi_rx + tx_len, poll_len));
	hexdump("poll: RX", t2->spi_rx + tx_len, poll_len);

	uint8_t *scan = t2->spi_rx;

	int i;
	for (i = 0; i < poll_len; i += 2) {
		if ((scan[i] & 0x0f) == T2_READ_RESULT && (scan[i] & 0xf0) != 0) {
			res_crc = (scan[i + ASIC_RESULT_LEN] << 8) + (scan[i + ASIC_RESULT_LEN+1] << 0);
			clc_crc = CRC16_T2(scan + i, ASIC_RESULT_LEN);
				if(clc_crc == res_crc)
					return scan + i;
		}
	}
	
	return NULL;
}

static uint8_t *cmd_WRITE_REG(struct T2_chain *t2, uint8_t chip, uint8_t *reg)
{	
	uint8_t buffer[14];
	uint8_t reg_crc[14];

	buffer[0] = 0x09;
	buffer[1] = chip;
	memcpy(buffer + 2, reg, 12);

	unsigned short crc = CRC16_T2(buffer, 14);

	memcpy(reg_crc, reg, 12);
	reg_crc[12] = (crc >> 8) & 0xff;
	reg_crc[13] = crc & 0xff;
	
	uint8_t *ret = exec_cmd(t2, T2_WRITE_REG, chip, reg_crc, 14, 0);
	if (ret == NULL || ret[0] != T2_WRITE_REG) {
		applog(LOG_ERR, "%d: cmd_WRITE_REG failed", t2->chain_id);
		return NULL;
	}

	return ret;
}

static uint8_t *cmd_READ_REG(struct T2_chain *t2, uint8_t chip)
{
	uint8_t *ret = exec_cmd(t2, T2_READ_REG, chip, NULL, 14, 18);
	unsigned short crc = ((((unsigned short)ret[14]) << 8) & 0xff00) | ((unsigned short)ret[15] & 0x00ff);
	if (ret == NULL || ret[0] != T2_READ_REG_RESP || ret[1] != chip || crc != CRC16_T2(ret, 14)) {
		applog(LOG_ERR, "%d: cmd_READ_REG chip %d failed",
		       t2->chain_id, chip);
		return NULL;
	}
	memcpy(t2->spi_rx, ret, 16);
	return ret;
}

uint8_t cmd_CHECK_BUSY(struct T2_chain *t2, uint8_t chip_id)
{
	//printf("[check busy] \r\n");
	
	if(cmd_READ_REG(t2, chip_id) && (t2->spi_rx[11] & 0x01) == 1)
	{
		applog(LOG_DEBUG, "chip %d is busy now", chip_id);
		return WORK_BUSY;
	}
	//applog(LOG_WARNING, "chip %d is free now", chip_id);
	return WORK_FREE;
}

static bool cmd_WRITE_JOB(struct T2_chain *t2, uint8_t chip_id,
			      uint8_t *job)
{
	/* ensure we push the SPI command to the last chip in chain */
	int tx_len = JOB_LENGTH + 2;
	memcpy(t2->spi_tx, job, JOB_LENGTH);
	memset(t2->spi_tx + JOB_LENGTH, 0, tx_len - JOB_LENGTH);

	assert(spi_transfer(t2->spi_ctx, t2->spi_tx, t2->spi_rx, tx_len));

	int poll_len = 4 * chip_id - 2;

	assert(spi_transfer(t2->spi_ctx, NULL, t2->spi_rx + tx_len, poll_len));

//	int ack_len = tx_len;
//	int ack_pos = tx_len + poll_len - ack_len;
//	hexdump("poll: ACK", t2->spi_rx + ack_pos, tx_len);

	//printf("[write job] \r\n");
	//hexdump("job:", t2->spi_tx, JOB_LENGTH);

	cgsleep_us(1000);

	if(cmd_CHECK_BUSY(t2, chip_id) != WORK_BUSY)
	{
		return false;
	}

	return true;

}

/********** T2 low level functions */
#define MAX_PLL_WAIT_CYCLES 25
#define PLL_CYCLE_WAIT_TIME 40
static bool check_chip_pll_lock(struct T2_chain *t2, int chip_id, uint8_t *wr)
{
	int n;
	for (n = 0; n < MAX_PLL_WAIT_CYCLES; n++) {
		/* check for PLL lock status */

		if (cmd_READ_REG(t2, chip_id) && (t2->spi_rx[4] & 0x01) == 1) {
			/* double check that we read back what we set before */

			return wr[0] == t2->spi_rx[2] && wr[1] == t2->spi_rx[3] && wr[3] == t2->spi_rx[5];
		}
	}
	return false;
}

static int chain_chips(struct T2_chain *t2)
{
	int tx_len = 6;
	int rx_len = 4;
	int cid = t2->chain_id;

	memset(t2->spi_tx, 0, tx_len);
	memset(t2->spi_rx, 0, rx_len); 
	t2->spi_tx[0] = T2_BIST_START;
	t2->spi_tx[1] = 0;

	struct spi_ctx *spi = t2->spi_ctx;

	if (!spi_transfer(t2->spi_ctx, t2->spi_tx, t2->spi_rx, 6))
		return 0;
	hexdump("TX", t2->spi_tx, 6);
	hexdump("RX", t2->spi_rx, 6);

	int i;
	int max_poll_words = MAX_CHAIN_LENGTH * 2;

	for(i = 0; i < max_poll_words; i++) {
		if (t2->spi_rx[0] == T2_BIST_START && t2->spi_rx[1] == 0) {
			spi_transfer(t2->spi_ctx, NULL, t2->spi_rx, 2);
			hexdump("TX", t2->spi_tx, 2);
			uint8_t n = t2->spi_rx[1];
			t2->num_chips = (i / 2) + 1;
			if(t2->num_chips != n) {
				applog(LOG_ERR, "%d: enumeration: %d <-> %d",cid, t2->num_chips, n);
				if(n != 0)
					t2->num_active_chips = n;
			}
			applog(LOG_WARNING, "%d: detected %d chips", cid, t2->num_chips);
			return t2->num_chips;
		}
		bool s = spi_transfer(t2->spi_ctx, NULL, t2->spi_rx, 2);
		hexdump("RX", t2->spi_rx, 2);
		if(!s)
			return 0;
	}
	applog(LOG_WARNING, "%d: no T2 chip-chain detected", cid);
	return 0;
}

extern const uint8_t default_reg[][12];
static bool set_pll_config(struct T2_chain *t2, int idxpll)
{

	uint8_t temp_reg[REG_LENGTH];
	int i;
	uint32_t pll_fbdiv, pll_prediv, pll_postdiv, postdiv;
	uint32_t f_vcc, f_pll;

	for(i = 0; i <= idxpll; i++)
	{
		pll_fbdiv = ((((uint32_t)default_reg[i][0]) << 8) & 0x00000100) | (((uint32_t)default_reg[i][1]) & 0x000000ff);
		pll_prediv = (((uint32_t)default_reg[i][0]) >> 1) & 0x0000001f;
		pll_postdiv = (((uint32_t)default_reg[i][3]) >> 6) & 0x00000003;
		postdiv = (1 << pll_postdiv);

		f_vcc = 12 * pll_fbdiv / pll_prediv;
		f_pll = f_vcc / postdiv;

		memcpy(temp_reg, default_reg[i], REG_LENGTH-2);
		applog(LOG_DEBUG,
	       "{ 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, },//PLL=%dMHz, VCO=%dMHz",
	       temp_reg[0], temp_reg[1], temp_reg[2],
	       temp_reg[3], temp_reg[4], temp_reg[5],
	       temp_reg[6], temp_reg[7], temp_reg[8],
	       temp_reg[9], temp_reg[10], temp_reg[11], f_pll, f_vcc);
	    if (!cmd_WRITE_REG(t2, ADDR_BROADCAST, temp_reg))
		{
			applog(LOG_ERR, "set PLL %d MHz fail vco %d MHz", f_pll, f_vcc);
			return false;
		}
		applog(LOG_WARNING, "set PLL %d MHz success vco %d MHz", f_pll, f_vcc);

		cgsleep_us(100000);
	}

	t2->pll = f_pll;
	int from = 0;
	int to = t2->num_chips;

	for (i = from; i < to; i++) {
		int cid = i + 1;
		if (!check_chip_pll_lock(t2, cid, temp_reg)) {
			applog(LOG_ERR, "%d: chip %d failed PLL lock",
			       t2->chain_id, cid);
			//return false;
		}
	}
	return true;
}
static bool check_chip(struct T2_chain *t2, int i)
{
	int chip_id = i + 1;
	int cid = t2->chain_id;
	if (!cmd_READ_REG(t2, chip_id)) {
		applog(LOG_WARNING, "%d: Failed to read register for "
		       "chip %d -> disabling", cid, chip_id);
		t2->chips[i].num_cores = 0;
		t2->chips[i].disabled = 1;
		return false;;
	}
	t2->chips[i].num_cores = t2->spi_rx[13];
	t2->num_cores += t2->chips[i].num_cores;
	applog(LOG_WARNING, "%d: Found chip %d with %d active cores",
	       cid, chip_id, t2->chips[i].num_cores);
	return true;
}

/********** disable / re-enable related section (temporary for testing) */
static int get_current_ms(void)
{
	cgtimer_t ct;
	cgtimer_time(&ct);
	return cgtimer_to_ms(&ct);
}

static bool is_chip_disabled(struct T2_chain *t2, uint8_t chip_id)
{
	struct T2_chip *chip = &t2->chips[chip_id - 1];
	return chip->disabled || chip->cooldown_begin != 0;
}


int chain_t2_detect(struct T2_chain *t2, int idxpll)
{
	int i;
	int cid = t2->chain_id;
	int spi_clk_khz = 100000;
//	set_spi_wrspeed(t2->spi_ctx, spi_clk_khz);

	if(!cmd_RESET_BCAST(t2, 0x00))
		applog(LOG_WARNING, "cmd_RESET_BCAST fail");

	cgsleep_us(1000);
	t2->num_chips = chain_chips(t2);
	if (t2->num_chips == 0)
		goto failure;

	applog(LOG_WARNING, "spidev%d.%d %d: Found %d T2 chips",
   		t2->spi_ctx->config.bus, t2->spi_ctx->config.cs_line,
   		t2->chain_id, t2->num_chips);

	cgsleep_us(10000);
	if (!set_pll_config(t2, idxpll))
		goto failure;

	spi_clk_khz = 2000000;
//	set_spi_wrspeed(t2->spi_ctx, spi_clk_khz);
	cgsleep_us(1000);

	t2->num_active_chips = t2->num_chips;
	t2->chips = calloc(t2->num_active_chips, sizeof(struct T2_chip));
	assert(t2->chips != NULL);

	cgsleep_us(10000);
	if(!cmd_BIST_COLLECT_BCAST(t2))
		goto failure;
	applog(LOG_WARNING, "collect core success");

	cgsleep_us(1000);
	if (!cmd_BIST_FIX_BCAST(t2))
		goto failure;
		
	applog(LOG_WARNING, "bist fix success");
		
	for (i = 0; i < t2->num_active_chips; i++)
	{
		check_chip(t2, i);
	}
	return t2->num_chips;
failure:
	return 0;
}

/* check and disable chip, remember time */
static void disable_chip(struct T2_chain *t2, uint8_t chip_id)
{
	flush_spi(t2);
	struct T2_chip *chip = &t2->chips[chip_id - 1];
	int cid = t2->chain_id;
	if (is_chip_disabled(t2, chip_id)) {
		applog(LOG_WARNING, "%d: chip %d already disabled",
		       cid, chip_id);
		return;
	}
	applog(LOG_WARNING, "%d: temporary disabling chip %d", cid, chip_id);
	chip->cooldown_begin = get_current_ms();
}

/* check if disabled chips can be re-enabled */
void check_disabled_chips(struct T2_chain *t2, int pllnum)
{
	int i;
	int cid = t2->chain_id;
	uint8_t reg[REG_LENGTH];
	struct spi_ctx *ctx = t2->spi_ctx;

	for (i = 0; i < t2->num_active_chips; i++) 
	{
		int chip_id = i + 1;
		struct T2_chip *chip = &t2->chips[i];
		if (!is_chip_disabled(t2, chip_id))
			continue;
		/* do not re-enable fully disabled chips */
		if (chip->disabled)
			continue;
		if (chip->cooldown_begin + COOLDOWN_MS > get_current_ms())
			continue;

		//if the core in chain least than 432, reinit this chain
		if(t2->num_cores <= LEAST_CORE_ONE_CHAIN && chip->fail_reset < RESET_CHAIN_CNT)
		{
			chip->fail_reset++;
			asic_gpio_write(spi[i]->reset, 1);
			usleep(500000);
			asic_gpio_write(spi[i]->reset, 0);

			t2->num_chips = chain_t2_detect(t2, pllnum);
		}
		
		if (!cmd_READ_REG(t2, chip_id)) 
		{
			chip->fail_count++;
			applog(LOG_WARNING, "%d: chip %d not yet working - %d",
				   cid, chip_id, chip->fail_count);
			if (chip->fail_count > DISABLE_CHIP_FAIL_THRESHOLD) 
			{
				applog(LOG_WARNING, "%d: completely disabling chip %d at %d",
					   cid, chip_id, chip->fail_count);
				chip->disabled = true;
				t2->num_cores -= chip->num_cores;	
				
				continue;
			}
			/* restart cooldown period */
			chip->cooldown_begin = get_current_ms();
			continue;
		}
		applog(LOG_WARNING, "%d: chip %d is working again", cid, chip_id);
		chip->cooldown_begin = 0;
		chip->fail_count = 0;
		chip->fail_reset = 0;
	}
}

static void rev(unsigned char *s, size_t l)
{
	size_t i, j;
	unsigned char t;

	for (i = 0, j = l - 1; i < j; i++, j--) {
		t = s[i];
		s[i] = s[j];
		s[j] = t;
	}
}


static uint8_t *create_job(uint8_t chip_id, uint8_t job_id, struct work *work)
{
	unsigned char *wdata = work->data;
	uint8_t data[128];
	double sdiff = work->sdiff;
	uint8_t tmp_buf[JOB_LENGTH];
	uint16_t crc;
	uint8_t i;

	memset(data, 0, 128);

	for(int j=0; j<20; j++)
	{
		data[j*4 + 3] = work->data[j*4 + 0];
		data[j*4 + 2] = work->data[j*4 + 1];
		data[j*4 + 1] = work->data[j*4 + 2];
		data[j*4 + 0] = work->data[j*4 + 3];
	}
	wdata = data;

	static uint8_t job[JOB_LENGTH] = {
		/* command */
		0x00, 0x00,
		/* wdata 75 to 0 */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		/* start nonce */
		0x00, 0x00, 0x00, 0x00,
		/* difficulty */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		/* end nonce */
		0x00, 0x00, 0x00, 0x00,
		/* crc data */
		0x00, 0x00
	};

	uint8_t diffIdx;
	uint8_t data75to0[76];
	uint8_t startnonce[4] = {0x00, 0x00, 0x00, 0x00};
	uint8_t diff[8] = {0x1e, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff};
	uint8_t endnonce[4] = {0x00, 0x40, 0x00, 0x00}; // 10s

	memcpy(data75to0, wdata, 76);

	if(sdiff > 63.0)
		memcpy(diff, difficult_Tbl[7], 8);
	else if(sdiff > 31.0)
			memcpy(diff, difficult_Tbl[6], 8);
	else if(sdiff > 15.0)
			memcpy(diff, difficult_Tbl[5], 8);
	else if(sdiff > 7.0)
			memcpy(diff, difficult_Tbl[4], 8);
	else if(sdiff > 3.0)
				memcpy(diff, difficult_Tbl[3], 8);
	else if(sdiff == 2.0)
		memcpy(diff, difficult_Tbl[2], 8);
	else if(sdiff == 1.0)
		memcpy(diff, difficult_Tbl[1], 8);
	else if(sdiff < 1.0)
		memcpy(diff, difficult_Tbl[0], 8);

	startnonce[0]=0x00;
	startnonce[1]=0x00;
	startnonce[2]=0x00;
	startnonce[3]=0x00;

	endnonce[0]=0xff;
	endnonce[1]=0xff;
	endnonce[2]=0xff;
	endnonce[3]=0xff;

	//rev(data75to0, 76);
	rev(startnonce, 4);
	rev(diff, 8);
	rev(endnonce, 4);

	job[0] = (job_id << 4) | T2_WRITE_JOB;
	job[1] = chip_id;
	memcpy(job+2,			data75to0,	76);
	memcpy(job+2+76,		startnonce, 4);
	memcpy(job+2+76+4,		diff, 8);
	memcpy(job+2+76+4+8,	endnonce, 4);

	/* crc */
	memset(tmp_buf, 0, sizeof(tmp_buf));
	for(i = 0; i < 47; i++)
	{
		tmp_buf[(2 * i) + 1] = job[(2 * i) + 0];
		tmp_buf[(2 * i) + 0] = job[(2 * i) + 1];
	}
	crc = CRC16_2(tmp_buf, 94);
	job[94] = (uint8_t)((crc >> 8) & 0xff);
	job[95] = (uint8_t)((crc >> 0) & 0xff);

	return job;
}



/* set work for given chip, returns true if a nonce range was finished */
static bool set_work(struct T2_chain *t2, uint8_t chip_id, struct work *work,
		     uint8_t queue_states)
{
	int cid = t2->chain_id;
	struct T2_chip *chip = &t2->chips[chip_id - 1];
	bool retval = false;

	int job_id = chip->last_queued_id + 1;

	//applog(LOG_INFO, "%d: queuing chip %d with job_id %d, state=0x%02x",
	       //cid, chip_id, job_id, queue_states);
	if (job_id == (queue_states & 0x0f) || job_id == (queue_states >> 4))
		applog(LOG_WARNING, "%d: job overlap: %d, 0x%02x",
		       cid, job_id, queue_states);

	if (chip->work[chip->last_queued_id] != NULL) {
		work_completed(t2->cgpu, chip->work[chip->last_queued_id]);
		chip->work[chip->last_queued_id] = NULL;
		retval = true;
	}
	uint8_t *jobdata = create_job(chip_id, job_id, work);
	if (!cmd_WRITE_JOB(t2, chip_id, jobdata)) {
		/* give back work */
		work_completed(t2->cgpu, work);
		applog(LOG_ERR, "%d: failed to set work for chip %d.%d",
		       cid, chip_id, job_id);
		disable_chip(t2, chip_id);
	} else {
		chip->work[chip->last_queued_id] = work;
		chip->last_queued_id++;
		chip->last_queued_id &= 3;
	}
	return retval;
}

static bool get_nonce(struct T2_chain *t2, uint8_t *nonce,
		      uint8_t *chip, uint8_t *job_id)
{
	uint8_t *ret = cmd_READ_RESULT_BCAST(t2);
	if (ret == NULL)
		return false;

	*job_id = ret[0] >> 4;
	*chip = ret[1];
	memcpy(nonce, ret + 2, 4);
	return true;
}

/* reset input work queues in chip chain */
static bool abort_work(struct T2_chain *t2)
{
	/* drop jobs already queued and result queue: reset strategy (0xeded & 0xf7f7) */
	//applog(LOG_INFO,"Start to reset ");
//return cmd_RESET_BCAST(t2, 0xe5);
	return true;
}

void exit_t2_chain(struct T2_chain *t2)
{
	if (t2 == NULL)
		return;
	free(t2->chips);
	t2->chips = NULL;
	t2->spi_ctx = NULL;
	free(t2);
}

void exit_T2_chain(struct T2_chain *t2)
{
	if (t2 == NULL)
		return;
	free(t2->chips);
	t2->chips = NULL;
	t2->spi_ctx = NULL;
	free(t2);
}

bool inno_cmd_resetjob(struct T2_chain *t2, uint8_t chip_id)
{
	//printf("send command job [reset] \n");

	if(!cmd_RESET_IDUAL(t2, chip_id))
	{
		applog(LOG_WARNING, "cmd_RESET_BCAST fail !");
	}

	if(cmd_CHECK_BUSY(t2, chip_id) != WORK_FREE)
	{
		return false;
	}

	return true;
	
}

struct T2_chain *init_T2_chain(struct spi_ctx *ctx, int chain_id)
{
	int i;
	struct T2_chain *t2 = malloc(sizeof(*t2));
	assert(t2 != NULL);

	
	memset(t2, 0, sizeof(*t2));
	t2->spi_ctx = ctx;
	t2->chain_id = chain_id;

	applog(LOG_WARNING,"chain_id:%d", chain_id);
	switch(chain_id){
		case 0:t2->num_chips = chain_t2_detect(t2, A1Pll1);break;
		case 1:t2->num_chips = chain_t2_detect(t2, A1Pll2);break;
		case 2:t2->num_chips = chain_t2_detect(t2, A1Pll3);break;
		case 3:t2->num_chips = chain_t2_detect(t2, A1Pll4);break;
		default:;
	}
	cgsleep_us(10000);
	
	if (t2->num_chips == 0)
		goto failure;

	applog(LOG_WARNING, "spidev%d.%d: %d: Found %d T2 chips",
	       t2->spi_ctx->config.bus, t2->spi_ctx->config.cs_line,
	       t2->chain_id, t2->num_chips);
	
	t2->num_active_chips = t2->num_chips;

	t2->chips = calloc(t2->num_active_chips, sizeof(struct T2_chip));
	assert (t2->chips != NULL);


	applog(LOG_WARNING, "%d: found %d chips with total %d active cores",
	       t2->chain_id, t2->num_active_chips, t2->num_cores);

	mutex_init(&t2->lock);
	INIT_LIST_HEAD(&t2->active_wq.head);

	return t2;

failure:
	exit_T2_chain(t2);
	return NULL;
}

static bool detect_T2_chain()
{
	int i;
	
	applog(LOG_WARNING, "T2: checking T2 chain");

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		cfg[i] = default_spi_config;
		cfg[i].bus     = i + 1;
		cfg[i].cs_line = 0;
		cfg[i].mode = SPI_MODE_1;
		cfg[i].speed = DEFAULT_SPI_SPEED;

		spi[i] = spi_init(&cfg[i]);
		if (spi == NULL)
		{
			applog(LOG_ERR, "spi %d init fail", i + 1);
			continue;
		}

		spi[i]->power_en = CHIP_PIN_POWER_EN[i];
		spi[i]->start_en = CHIP_PIN_START_EN[i];
		spi[i]->reset = CHIP_PIN_RESET[i];
		//spi[i]->plug  = SPI_PIN_PLUG[i];
		//spi[i]->led   = SPI_PIN_LED[i];

		asic_gpio_init(spi[i]->power_en, 0);
		asic_gpio_init(spi[i]->start_en, 0);
		asic_gpio_init(spi[i]->reset, 0);
		//asic_gpio_init(spi[i]->plug, 0);
		//asic_gpio_init(spi[i]->led, 0);
		usleep(500000);
	}

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		if(spi[i] != NULL)
			break;
		if(i + 1 == ASIC_CHAIN_NUM)
			return false;
	}
	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		asic_gpio_write(spi[i]->power_en, 1);
		asic_gpio_write(spi[i]->reset, 0);
		usleep(500000);
		asic_gpio_write(spi[i]->start_en, 0);
		usleep(500000);

	}
	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		chain[i] = init_T2_chain(spi[i], i);
		if (chain[i] == NULL)
		{
			applog(LOG_ERR, "init t2 chain %d fail", i + 1);
			continue;
		}

		struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
		assert(cgpu != NULL);
	
		memset(cgpu, 0, sizeof(*cgpu));
		cgpu->drv = &bitmineT2_drv;
		cgpu->name = "BitmineA1.SingleChainn";
		cgpu->threads = 1;

		cgpu->device_data = chain[i];

		chain[i]->cgpu = cgpu;
		add_cgpu(cgpu);

		applog(LOG_WARNING, "Detected the %d T2 chain with %d chips / %d cores",
		       i, chain[i]->num_active_chips, chain[i]->num_cores);
	}

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		if(chain[i] != NULL)
			return true;
	}
	return false;
}

static void chain_hw_dis(void)
{
}

/* Probe SPI channel and register chip chain */
void T2_detect(bool hotplug)
{
	/* no hotplug support for SPI */
	if (hotplug)
		return;

	applog(LOG_DEBUG, "T2 detect");

	A1Pll1 = T2_ConfigT2PLLClock(opt_A1Pll1);
	A1Pll2 = T2_ConfigT2PLLClock(opt_A1Pll2);
	A1Pll3 = T2_ConfigT2PLLClock(opt_A1Pll3);
	A1Pll4 = T2_ConfigT2PLLClock(opt_A1Pll4);
	/* detect and register supported products */
	if (detect_T2_chain())
		return;

    int i = 0;
	/* release SPI context if no T2 products found */
	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		spi_exit(spi[i]);
	}

}

#define TEMP_UPDATE_INT_MS	2000
static int64_t T2_scanwork(	struct thr_info *thr)
{
	  int i;
	  int32_t A1Pll = 1000;
	  struct cgpu_info *cgpu = thr->cgpu;
	  struct T2_chain *t2 = cgpu->device_data;
	  int32_t nonce_ranges_processed = 0;
	
	  if (t2->num_cores == 0) {
		  cgpu->deven = DEV_DISABLED;
		  return 0;
	  }
	
	  //board_selector->select(t3->chain_id);
	  //applog(LOG_DEBUG, "T3 running scanwork");
	
	  uint32_t nonce;
	  uint8_t chip_id;
	  uint8_t job_id;
	  bool work_updated = false;
	  uint32_t k, *work_nonce=NULL;
	  unsigned char pworkdata[128]= {0},  hash1[32]= {0};
	  unsigned int endiandata[32]= {0};
	
	  mutex_lock(&t2->lock);
	
	  if (t2->last_temp_time + TEMP_UPDATE_INT_MS < get_current_ms())
	  {
		  //t2->temp = board_selector->get_temp(0);
		  t2->last_temp_time = get_current_ms();
	  }
	  int cid = t2->chain_id; 
	  uint32_t *target32;
	
	  /* poll queued results */
	  while (true)
	  {
		  if (!get_nonce(t2, (uint8_t*)&nonce, &chip_id, &job_id))
			  break;
	
		  //nonce = bswap_32(nonce);   //modify for A4
		  work_updated = true;
		  if (chip_id < 1 || chip_id > t2->num_active_chips) 
		  {
			  applog(LOG_WARNING, "%d: wrong chip_id %d", cid, chip_id);
			  continue;
		  }
		  if (job_id < 1 && job_id > 15) 
		  {
			  applog(LOG_WARNING, "%d: chip %d: result has wrong ""job_id %d", cid, chip_id, job_id);
			  flush_spi(t2);
			  continue;
		  }
	
		  struct T2_chip *chip = &t2->chips[chip_id - 1];
		  struct work *work = chip->work[job_id - 1];
		  if (work == NULL) 
		  {
			  /* already been flushed => stale */
			  applog(LOG_WARNING, "%d: chip %d: stale nonce 0x%08x", cid, chip_id, nonce);
			  chip->stales++;
			  continue;
		  }
		  target32 = (uint32_t *)(work->target);
		  const uint32_t Htarg = le32toh(target32[7]);
		  work_nonce = (uint32_t *)(work->data + 64 + 12);
		  *work_nonce = nonce;
		  memcpy(pworkdata, work->data, 80);
	
		  for (k=0; k < 20; k++)
		  {
			  endiandata[k] = ((uint32_t*)pworkdata)[k];
			  endiandata[k] = bswap_32(endiandata[k]);
			  //applog(LOG_DEBUG,"%s: endiandata[%d] = 0x%08x", __FUNCTION__, k, endiandata[k]);
		  }
	
		  Xhash(hash1, endiandata);
		  memcpy(work->hash, hash1, 32);

		  if(*((uint32_t *)(work->hash) + 7) <= Htarg)
		  {
			  //update_work_stats(thr, work);
			  if (fulltest(hash1, work->target))
			  {
				  submit_nonce_direct(thr,work, nonce);
			  }
		  } else {
			  applog(LOG_WARNING, "%d: chip %d: invalid nonce 0x%08x", cid, chip_id, nonce);
			  chip->hw_errors++;
			  /* add a penalty of a full nonce range on HW errors */
			  nonce_ranges_processed--;
			  continue;
		  }
		  applog(LOG_INFO, "YEAH: %d: chip %d / job_id %d: nonce 0x%08x", cid, chip_id, job_id, nonce);
		  chip->nonces_found++;
	  }
	
	  uint8_t reg[REG_LENGTH];
	  /* check for completed works */
	  if(t2->work_start_delay > 0)
	  {
		  applog(LOG_INFO, "wait for pll stable");
		  t2->work_start_delay--;
	  }
	  else
	  {
		  for (i = t2->num_active_chips; i > 0; i--) 
		  {
			  uint8_t c = i;
			  if (is_chip_disabled(t2, c))
				  continue;
			  if (!cmd_READ_REG(t2, c)) 
			  {
				  disable_chip(t2, c);
				  continue;
			  }
			  else
			  {
				  //hexdump("send433: RX", t3->spi_rx, 18);
				  /* update temp database */
				  uint32_t temp = 0;
				  float    temp_f = 0.0f;
	
				  temp = 0x000003ff & ((reg[7] << 8) | reg[8]);
				  //inno_fan_temp_add(&s_fan_ctrl, cid, temp, false);
			  }
	
			  uint8_t qstate = t2->spi_rx[11] & 0x01;
			  uint8_t qbuff = 0;
			  struct work *work;
			  struct T2_chip *chip = &t2->chips[i - 1];
			  switch(qstate) 
			  {
			  
			  case 1:
				  //applog(LOG_INFO, "chip %d busy now", i);
				  break;
				  /* fall through */
			  case 0:
				  work_updated = true;
	
				  work = wq_dequeue(&t2->active_wq);
				  if (work == NULL) 
				  {
					  applog(LOG_INFO, "%d: chip %d: work underflow", cid, c);
					  break;
				  }
				  if (set_work(t2, c, work, qbuff)) 
				  {
					  chip->nonce_ranges_done++;
					  nonce_ranges_processed++;
					  applog(LOG_INFO, "set work success %d, nonces processed %d", cid, nonce_ranges_processed);
				  }
				  
				  //applog(LOG_INFO, "%d: chip %d: job done: %d/%d/%d/%d",
				  //	   cid, c,
				  //	   chip->nonce_ranges_done, chip->nonces_found,
				  //	   chip->hw_errors, chip->stales);
				  break;
			  }
		  } 
		  //inno_fan_speed_update(&s_fan_ctrl, cid);
	  }
	
	  switch(cid){
		  case 0:check_disabled_chips(t2, A1Pll1);;break;
		  case 1:check_disabled_chips(t2, A1Pll2);;break;
		  case 2:check_disabled_chips(t2, A1Pll3);;break;
		  case 3:check_disabled_chips(t2, A1Pll4);;break;
		  default:;
	  }
	
	  mutex_unlock(&t2->lock);
	
	  //board_selector->release();
	
	  if (nonce_ranges_processed < 0)
	  {
		  applog(LOG_INFO, "nonce_ranges_processed less than 0");
		  nonce_ranges_processed = 0;
	  }
	
	  if (nonce_ranges_processed != 0) 
	  {
		  applog(LOG_INFO, "%d, nonces processed %d", cid, nonce_ranges_processed);
	  }
	  /* in case of no progress, prevent busy looping */
	  //if (!work_updated)
	  //  cgsleep_ms(40);
	
	  cgtime(&t2->tvScryptCurr);
	  timersub(&t2->tvScryptCurr, &t2->tvScryptLast, &t2->tvScryptDiff);
	  cgtime(&t2->tvScryptLast);
	
	
	  switch(cgpu->device_id){
		  case 0:A1Pll = t2->pll;break;
		  case 1:A1Pll = t2->pll;break;
		  case 2:A1Pll = t2->pll;break;
		  case 3:A1Pll = t2->pll;break;
		  default:;
	  }
	
	
	  return (int64_t)(2011173.18 * A1Pll / 1000 * (t2->num_cores/9.0) * (t2->tvScryptDiff.tv_usec / 1000000.0));


}


/* queue two work items per chip in chain */
static bool T2_queue_full(struct cgpu_info *cgpu)
{
	struct T2_chain *t2 = cgpu->device_data;
	int queue_full = false;

	mutex_lock(&t2->lock);
	//applog(LOG_DEBUG, "%d, T2 running queue_full: %d/%d",
	//       t2->chain_id, t2->active_wq.num_elems, t2->num_active_chips);

	if (t2->active_wq.num_elems >= t2->num_active_chips * 2)
		queue_full = true;
	else
		wq_enqueue(&t2->active_wq, get_queued(cgpu));

	mutex_unlock(&t2->lock);

	return queue_full;
}

static void T2_flush_work(struct cgpu_info *cgpu)
{
	struct T2_chain *t2 = cgpu->device_data;
	int cid = t2->chain_id;
	//board_selector->select(cid);
	int i;

	mutex_lock(&t2->lock);
	/* stop chips hashing current work */
	if (!abort_work(t2)) 
	{
		applog(LOG_ERR, "%d: failed to abort work in chip chain!", cid);
	}
	/* flush the work chips were currently hashing */
	for (i = 0; i < t2->num_active_chips; i++) 
	{
		int j;
		struct T2_chip *chip = &t2->chips[i];
		for (j = 0; j < 4; j++)
		{
			struct work *work = chip->work[j];
			if (work == NULL)
				continue;
			applog(LOG_DEBUG, "%d: flushing chip %d, work %d: 0x%p",
			       cid, i, j + 1, work);
			work_completed(cgpu, work);
			chip->work[j] = NULL;
		}

		chip->last_queued_id = 0;

		if(!inno_cmd_resetjob(t2, i + 1))
		{
			applog(LOG_WARNING, "chip %d clear work false", i + 1);
			continue;
		}

		//applog(LOG_INFO, "chip :%d flushing queued work success", i);
	}
	/* flush queued work */
	//applog(LOG_DEBUG, "%d: flushing queued work...", cid);
	while (t2->active_wq.num_elems > 0) 
	{
		struct work *work = wq_dequeue(&t2->active_wq);
		assert(work != NULL);
		work_completed(cgpu, work);
	}
	mutex_unlock(&t2->lock);

}

static void T2_get_statline_before(char *buf, size_t len, struct cgpu_info *cgpu)
{
	struct T2_chain *t2 = cgpu->device_data;
	char temp[10];
	if (t2->temp != 0)
		snprintf(temp, 9, "%2dC", t2->temp);
	tailsprintf(buf, len, " %2d:%2d/%3d %s",
		    t2->chain_id, t2->num_active_chips, t2->num_cores,
		    t2->temp == 0 ? "   " : temp);
}

struct device_drv bitmineT2_drv = {
	.drv_id = DRIVER_bitmineA1,
	.dname = "LtcmineT2",
	.name = "BA1",
	.drv_detect = T2_detect,

	.hash_work = hash_queued_work,
	.scanwork = T2_scanwork,
	.queue_full = T2_queue_full,
	.flush_work = T2_flush_work,
	.get_statline_before = T2_get_statline_before,
};

