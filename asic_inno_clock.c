#include "config.h"

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "logging.h"
#include "miner.h"
#include "util.h"

#include "asic_inno_cmd.h"
#include "asic_inno_clock.h"

const struct PLL_Clock PLL_Clk_12Mhz[118]={
	{0,  120,	A4_PLL(1,	80, 3)}, //default
	{19, 300,	A4_PLL(1,	50, 1)},
	{49, 600,	A4_PLL(1,	50, 0)},
	{79, 900,	A4_PLL(1,	75, 0)},
	{93, 1044,	A4_PLL(1,	87, 0)},
	{98, 1092,	A4_PLL(1,	91, 0)},
};

int A1_ConfigA1PLLClock(int optPll)
{
	int i;
	int A1Pll;

	if(optPll>0)
	{
		A1Pll=0;
		if(optPll<=PLL_Clk_12Mhz[0].speedMHz) 
		{
			A1Pll=0; //found
		}
		else
		{
			for(i=1;i<118;i++)
			{
				if((optPll<PLL_Clk_12Mhz[i].speedMHz)&&(optPll>=PLL_Clk_12Mhz[i-1].speedMHz))
				{
					A1Pll=i-1; //found
					break;
				}
			}
		}

		applog(LOG_NOTICE, "A1 = %d,%d",optPll,A1Pll);
		applog(LOG_NOTICE, "A1 PLL Clock = %dMHz",PLL_Clk_12Mhz[A1Pll].speedMHz);
	}

	return A1Pll;
}


void A1_SetA1PLLClock(struct A1_chain *a1,int pllClkIdx)
{
	uint8_t i;
	struct A1_chip *chip;
	uint32_t regPll;
	uint8_t rxbuf[12];
	
	uint8_t fix_val[8] = {0x00, 0x00, 0x00, 0xA8, 0x00, 0x24, 0xFF, 0xFF}; 

	assert(a1->chips != NULL);
	assert((pllClkIdx > 0) && (pllClkIdx < A4_PLL_CLOCK_MAX));

	regPll = PLL_Clk_12Mhz[pllClkIdx].pll_reg;

	chip = &a1->chips[0];
	memcpy(chip->reg,     (uint8_t*)&regPll + 3 ,1);
	memcpy(chip->reg + 1, (uint8_t*)&regPll + 2 ,1);
	memcpy(chip->reg + 2, (uint8_t*)&regPll + 1 ,1);
	memcpy(chip->reg + 3, (uint8_t*)&regPll + 0 ,1);
	memcpy(chip->reg + 4, fix_val , 8);
	
	//chip->reg[6] = (asic_vol_set&0xff00)>>8;
	//chip->reg[7] = asic_vol_set&0x00ff;
	//chip->reg[8] = pllClkIdx;
	//chip->reg[9] = pllClkIdx;
	//applog(LOG_INFO,"pllClkIdx is %d %d", chip->reg[8],chip->reg[9]);	

	inno_cmd_write_reg(a1, ADDR_BROADCAST, chip->reg);
	usleep(100000);
	inno_cmd_read_reg(a1, ADDR_BROADCAST, rxbuf);
	hexdump("read value", rxbuf, 12);	
	
}


