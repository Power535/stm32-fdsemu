#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "main.h"
#include "sram.h"
#include "spiutil.h"

#include "xprintf.h"

#define NUM_PAGES	512

extern uint8_t writebuf[];
const uint32_t testid = 0xCAFEBABE;

void sram_read(int addr,uint8_t *buf,int len)
{
	uint8_t data[4];

	data[0] = 3;
	data[1] = (uint8_t)(addr >> 16);
	data[2] = (uint8_t)(addr >> 8);
	data[3] = (uint8_t)(addr >> 0);
	SRAM_Select();
	SRAM_Write(data, 4);
	SRAM_Read(buf, len);
	SRAM_Deselect();
}

void sram_write(int addr,uint8_t *buf,int len)
{
	uint8_t data[4];

	data[0] = 2;
	data[1] = (uint8_t)(addr >> 16);
	data[2] = (uint8_t)(addr >> 8);
	data[3] = (uint8_t)(addr >> 0);
	SRAM_Select();
	SRAM_Write(data, 4);
	SRAM_Write(buf, len);
	SRAM_Deselect();
}

static void zero_page(int page)
{
	memset(writebuf,0,256);
	sram_write(page * 256,writebuf,256);
}

static int test_page(int page, int maxerr)
{
	int i, errors = 0;

	for(i=0;i<256;i++) {
		writebuf[i] = (uint8_t)(i);
	}

	sram_write(page * 256,writebuf,256);
	memset(writebuf,0,256);
	sram_read(page * 256,writebuf,256);

	for(i=0;i<256;i++) {
		if(writebuf[i] != (uint8_t)(i)) {
			printf("sram test failed at byte %d. (wanted $%02X, got $%02X)\r\n",i + (page * 256),(uint8_t)i,writebuf[i]);
			errors++;
			if(errors >= maxerr) {
				return(errors);
			}
		}
	}
	return(errors);
}

int sram_test(void)
{
	int maxerr = 3;
	int i, error;
	
	for(i=0; i<0x200; i++) {
		zero_page(i);
	}

	for(i=0, error=0; i<0x200; i++) {
		error += test_page(i, maxerr);
		if(error >= maxerr) {
//			printf("too many errors, stopping test.\r\n");
			return(error);
		}
	}
	if(error) {
//		printf("sram test failed, %d errors.\r\n",error);
		return(error);
	}
	printf("sram test passed.\r\n");
	return(0);
}

int sram_init(void)
{
	uint8_t data[4];
	int ret;

	//put sram into sequencial mode
	SRAM_Select();
	data[0] = 1;
	data[1] = 0x40;
	SRAM_Write(data,2);
	SRAM_Deselect();
	
	sram_read(0x20000 - 4,data,4);
	
	if(memcmp(data,&testid,4) == 0) {
		return(0);
	}
  
	//perform sram test
	ret = sram_test();

	sram_write(0x20000 - 4,(uint8_t*)&testid,4);

	return(ret);
}
