#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f0xx.h"
#include "transfer.h"
#include "fifo.h"
#include "fds.h"
#include "main.h"
#include "spiutil.h"
#include "diskutil.h"
#include "sram.h"
#include "xprintf.h"

/*      64 65 64 64 64 64 64 64 65 64 64 64 64 64 64 64  deddddddeddddddd
  01a0  64 64 65 64 64 64 64 64 64 64 65 64 64 64 64 64  ddedddddddeddddd
  01b0  64 64 65 64 64 64 64 64 64 64 64 65 64 64 64 64  ddeddddddddedddd
  01c0  64 64 64 64 64 65 64 64 64 64 64 64 64 64 65 64  dddddedddddddded
  01d0  96 c8 96 64 65 64 64 96 64 64 64 96 65 64 96 64  ...dedd.ddd.ed.d
  01e0  c8 c8 c9 64 64 64 97 64 95 65 96 64 64 64 64 64  ...ddd.d.e.ddddd
  01f0  64 65 64 64 64 64 64 64 64 64 65 64 64 64 64 64  deddddddddeddddd

  0380  54 54 55 55 56 57 57 58 58 58 59 59 5b 5b 5a 5b  TTUUVWWXXXYY[[Z[
  0390  5b 5d 5d 5d 5f 5f 60 5f 5f 5e 5e 5e 5e 5f 5f 5f  []]]__`__^^^^___
  03a0  5e 5e 5e 5d 5e 5e 5f 5e 5e 5e 5e 5e 5e 5e 5d 5f  ^^^]^^_^^^^^^^]_
  03b0  5f 5e 5e 5e 5e 5e 5e 5e 5e 5e 5e 5e 5e 5e 5e 5e  _^^^^^^^^^^^^^^^
  03c0  5f 5f 5e 5e 5e 5e 5e 5f 5e 5e 5e 5d 5e 5e 5e 5e  __^^^^^_^^^]^^^^
  03d0  90 c3 90 5e 5e 5e 5e 5f 90 c2 52 85 50 83 55 56  ...^^^^_..R.P.UV
  03e0  89 53 53 85 8b 56 56 57 57 59 8a 8e 5a 5b 5b 5b  .SS..VVWWY..Z[[[
  03f0  5c 5c 5c 5b 5d 5f 5f 60 5f 5f 5e 5f 5e 5f 5e 5f  \\\[]__`__^_^_^_
*/
__forceinline uint8_t raw_to_raw03_byte(uint8_t raw)
{
	if(raw < 0x48)
		return(3);
	else if(raw < 0x78)
		return(0);
	else if(raw < 0xA8)
		return(1);
	else if(raw < 0xE0)
		return(2);
	return(3);
}

__forceinline void decode(uint8_t *dst, uint8_t src, int *outptr, uint8_t *bit)
{
	int out = *outptr;

	switch(src | (*bit << 4)) {
		case 0x11:
			out++;
		case 0x00:
			out++;
			*bit = 0;
			break;
		case 0x12:
			out++;
		case 0x01:
		case 0x10:
			dst[out / 8] |= (1 << (out & 7));
			out++;
			*bit = 1;
			break;
		default:
			out++;
			*bit = 0;
			break;
	}
	*outptr = out;
}

//for sending data to the ram adaptor
volatile int count;
volatile int bytes;
volatile int needbyte;
volatile uint32_t dataout,dataout2;
uint8_t writebuf[WRITEBUF_SIZE];
const uint8_t expand[]={ 0xaa, 0xa9, 0xa6, 0xa5, 0x9a, 0x99, 0x96, 0x95, 0x6a, 0x69, 0x66, 0x65, 0x5a, 0x59, 0x56, 0x55 };

void TIM14_IRQHandler(void)
{
	//irq acknowledge
	TIM14->SR = (uint16_t)~TIM_IT_Update;

	//output current bit
	if((dataout & 1) == 0) {
		GPIOB->BSRR = GPIO_Pin_4;
	}
	else {
		GPIOB->BRR = GPIO_Pin_4;
	}

	//shift the data byte over to the next next bit
	dataout >>= 1;

	//increment bit counter
	count++;
		
	//if we have sent all eight bits of this byte, get next byte
	if(count == 16) {
		count = 0;

		//read next byte from the page
		dataout = dataout2;
		
		//signal we need another mfm byte
		needbyte++;
	}
}

//for writes coming out of the ram adaptor
void TIM1_CC_IRQHandler(void)
{
	uint16_t ra;

	//reset timer counter
	TIM1->CNT = 0;

	//clear interrupt flag (does reading TIM->CCR reset this?)
	TIM1->SR = (uint16_t)~TIM_IT_CC4;

	//if we are writing
	if(IS_WRITE()) {
		
		//get flux transition time
		ra = TIM1->CCR4;
		
		//write to fifo
		fifo_write_byte((uint8_t)ra);
	}
}

//begin starting transfer
void start_transfer(void)
{
	int leadin;
 
	LED_RED_ON();

	//delay about 150ms
	DelayMS(160);
	
	//disable systick irq
	NVIC_DisableIRQ(SysTick_IRQn);
	
	//enable transfer irq
	NVIC_EnableIRQ(TIM14_IRQn);
	NVIC_EnableIRQ(TIM1_CC_IRQn);

	SET_READY();

	//initialize variables
	count = 0;
	bytes = 0;
	dataout = dataout2 = 0xAAAA;

	//init fifo
	fifo_init((uint8_t*)writebuf,WRITEBUF_SIZE);

	//lead-in byte counter
	leadin = DEFAULT_LEAD_IN / 8;

	//transfer lead-in
	while(IS_SCANMEDIA() && IS_DONT_STOPMOTOR()) {
		
		//if irq handler needs more data
		if(needbyte) {
			needbyte = 0;
			bytes++;
		}

		//check if enough leadin data has been sent
		if(bytes >= leadin) {
			break;
		}
	}

//	printf("lead-in transferred, %d bytes\n", bytes);

	//reset byte counter
	bytes = 0;
	count = 0;
}

//stop transfer
void stop_transfer(void)
{
	CLEAR_READY();

	//disable transfer irq
	NVIC_DisableIRQ(TIM14_IRQn);
	NVIC_DisableIRQ(TIM1_CC_IRQn);
	
	//enable systick irq
	NVIC_EnableIRQ(SysTick_IRQn);
}

//read a byte from virtual disk
__forceinline uint8_t disk_read_byte(int addr)
{
	uint8_t byte;

	//read data from sram
	SRAM_Select();
	SRAM_WriteByte(3);
	SRAM_WriteByte((uint8_t)(addr >> 16));
	SRAM_WriteByte((uint8_t)(addr >> 8));
	SRAM_WriteByte((uint8_t)(addr >> 0));
	byte = SRAM_ReadByte();
	SRAM_Deselect();
	return(byte);
}

//write a byte to the virtual disk
__forceinline void disk_write_byte(int addr,uint8_t byte)
{
	//write the byte to sram chip
	SRAM_Select();
	SRAM_WriteByte(2);
	SRAM_WriteByte((uint8_t)(addr >> 16));
	SRAM_WriteByte((uint8_t)(addr >> 8));
	SRAM_WriteByte((uint8_t)(addr >> 0));
	SRAM_WriteByte(byte);
	SRAM_Deselect();
}

__forceinline void check_needbyte(void)
{
	uint8_t byte;

	if(needbyte) {
			
		//clear flag
		needbyte = 0;
		
		//read next byte to be output
		byte = disk_read_byte(bytes);
		
		//increment the byte counter
		bytes++;

		//convert to mfm
		dataout2 = expand[byte & 0x0F];
		dataout2 |= expand[(byte & 0xF0) >> 4] << 8;
	}
}

int do_transfer(int is_loader)
{
	int writepos = -1;
	
	start_transfer();

	printf("beginning transfer of %s\n", is_loader ? "loader" : "fds disk");

	while(IS_SCANMEDIA() && IS_DONT_STOPMOTOR()) {

		//check irq handler requesting another byte
		check_needbyte();

		if(IS_WRITE()) {
			int len = 0;
			uint8_t bitval = 0;
			uint8_t decoded[4] = {0,0,0,0};
			uint8_t byte;

			LED_GREEN_OFF();

			//set dirty flag to write data back to flash
//			dirty = 1;

			//get initial write position
			if(is_loader) {
				writepos = 0xF000;
			}
			else {
				writepos = bytes;
			}

			//initialize the timer to get the flux transitions
			TIM1->CNT = 0;
			NVIC_EnableIRQ(TIM1_CC_IRQn);

			//while the write line is asserted
			while(IS_WRITE()) {

				//check irq handler requesting another byte
				check_needbyte();

				//decode data in the fifo buffer in there is any
				if(fifo_read_byte(&byte)) {

					//decode the data
					decode((uint8_t*)decoded,raw_to_raw03_byte(byte),&len,&bitval);
					
					//if we have a full byte, write it to sram
					if(len >= 8) {
						len -= 8;
						disk_write_byte(writepos++, decoded[0]);
						decoded[0] = decoded[1];
						decoded[1] = 0;
					}
				}
			}

			//stop the timer for gathering flux transitions
			NVIC_DisableIRQ(TIM1_CC_IRQn);

			//decode data in the fifo buffer
			while(fifo_read_byte(&byte)) {

				//check irq handler requesting another byte
				check_needbyte();

				decode((uint8_t*)decoded,raw_to_raw03_byte(byte),&len,&bitval);
				if(len >= 8) {
					len -= 8;
					disk_write_byte(writepos++, decoded[0]);
					decoded[0] = decoded[1];
					decoded[1] = 0;
				}
			}

			check_needbyte();

			if(len) {
				disk_write_byte(writepos, decoded[0]);
			}

			LED_GREEN_ON();
		}

		//check if insane
		if(bytes >= 0x18000) {
			printf("reached end of data block, something went wrong...\r\n");
			break;
		}
	}

	stop_transfer();

	return(writepos);
}

int fds_savesram(void);

int begin_transfer(void)
{
	int writepos;

	writepos = do_transfer(0);

	printf("transferred %d bytes\n",bytes);
//	printf("transferred (curside = %d, numsides = %d, hassaver = %d)\n",diskinfo.curside,diskinfo.numsides,diskinfo.hassaver);
	
	//ignore writes if this is game doctor image
	if(diskinfo.type == 1) {
		
		//unless it is the save disk
		if(diskinfo.hassaver != 0) {
			if(diskinfo.curside == 0x80) {
				return(1);
			}
		}
		return(0);
	}

	if(writepos != -1) {
		return(1);
	}
	return(0);
}

void hexdump(char *desc, void *addr, int len);
int skip_gap(int pos);
extern char currentfn[];

int get_block_shift(uint8_t byte);
uint8_t realign(uint8_t byte, uint8_t nextbyte, int shift);

//call at the beginning of the block, after the gap
//the first byte read must contain the "gap end" marker
int realign_block2(int pos,uint8_t *data)
{
	uint8_t byte, nextbyte, blockid;
	int i;
	int shift;
	int size;

	//read first byte to determine how many bits to shift over
	sram_read(pos++, &byte, 1);

	//determine how far to shift
	shift = get_block_shift(byte);

	//shift out the "gap end" marker and have partial (or full) block id
	byte >>= shift;

	//get next byte from sram
	sram_read(pos++, &nextbyte, 1);

	//assemble the original byte
	byte = realign(byte, nextbyte, shift);
	blockid = byte;

	//write blockid to buf
	*data = byte;
	data++;

	//put nextbyte into byte (with realignment)
	byte = nextbyte >> shift;

	//determine block size (includes block id)
	switch (blockid) {
		
	//boot game
	case 0xDB:
		size = 12 + 1;
		break;
	
	//update firmware
	case 0xDC:
		size = 12 + 1;
		break;

	default:
//		printf("unknown block %d, start = %-5d, shift is %d\n", blockid, start, shift);
		return(-1);
	}

	//copy the rest of the bytes in the block (start at 1 because we already wrote the blockid byte)
	for (i = 1; i < size; i++) {

		//read next byte from sram
		sram_read(pos++, &nextbyte, 1);

		//realign the byte
		byte = realign(byte, nextbyte, shift);

		//write byte to buf
		*data = byte;
		data++;

		//put nextbyte into byte
		byte = nextbyte >> shift;
	}

	//skip over the crc bytes and get into the next gap
	pos += 3;

//	printf("found block %d, start = %-5d, end = %-5d, shift is %d, size is %d\n", blockid, start, pos, shift, size);

	return(pos);
}

void update_firmware(void);

void begin_transfer_loader(void)
{
	int writepos;

	writepos = do_transfer(1);
	
	//if data was written, figure out what diskblock to boot from
	if(writepos != -1) {
		uint8_t *tempbuf = (uint8_t*)writebuf;
		uint8_t blockid;
		int pos = skip_gap(0xF000);

//		printf("write size = %d, write pos = %d\n",size,pos);
		realign_block2(pos,tempbuf);
//		hexdump("writebuf",(void*)tempbuf,WRITEBUF_SIZE);

		//save blockid and isolate data written
		blockid = *tempbuf;
		tempbuf++;
		tempbuf[12] = 0;

		//boot game
		if(blockid == 0xDB) {
			//load disk
			fds_open((char*)tempbuf);
			
			//if there is saver disk present (game doctor is implied) load it first
			if(diskinfo.hassaver) {
				fds_loadside(0x80);
				diskinfo.curside = 0xFF;
			}
			
			//else just load the first side
			else {
				fds_loadside(0);
			}
		}
		
		//update firmware
		if(blockid == 0xDC) {
			update_firmware();
		}

	}
	else {
		printf("transferred %d bytes\r\n",bytes);
	}
}
