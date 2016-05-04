#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "main.h"
#include "fds.h"
#include "sram.h"
#include "transfer.h"
#include "fatfs/src/ff.h"
#include "xprintf.h"
#include "build.h"

//information about the current disk we are using
diskinfo_t diskinfo;
FIL file;

const char firmware_fn[] = "magicwildcard.bin";

#define SRAM_COMMIT_DELAY	1500

#define BOOTLOADER_ADDRESS	0x08000000
#define FIRMWARE_ADDRESS	0x08001000
#define FLASH_END_ADDRESS	0x08008000
#define FLASH_PAGE_SIZE		0x400

void update_firmware(void)
{
	FRESULT fr;
	uint32_t i,j;
	uint32_t size,crc32;
	uint32_t bytesread, *ptr32;

	fr = f_open(&file, firmware_fn, FA_READ);
    if (fr != FR_OK) {
		return;
	}

	CLEAR_MEDIASET();
	CLEAR_WRITABLE();
	CLEAR_MOTORON();

	//configure crc32 unit
	CRC->DR = 0xFFFFFFFF;
	CRC->POL = 0x04C11DB7;
	CRC->IDR = 0x00;
	CRC->INIT = 0xFFFFFFFF;
	CRC->CR = CRC_CR_RESET | CRC_PolSize_32 | CRC_CR_REV_IN | CRC_CR_REV_OUT;

	size = f_size(&file);
	
	//data is firmware (padded with 0's),u32 size, u32 crc32 of 0x0000-0xEFFF, u32 magic (0xDEADBEEF)
	if(size != 0x700C) {
		f_close(&file);
		printf("invalid firmware size\n");
		NVIC_SystemReset();
		return;
	}

	ptr32 = (uint32_t*)writebuf;
	for(i=0;i<0x7000;) {
		f_read(&file,writebuf,256,&bytesread);
		for(j=0;j<(bytesread/4);j++) {
			crc32 = CRC_CalcCRC(ptr32[j]);
		}
		sram_write(i,writebuf,256);
		i += bytesread;
	}
	crc32 ^= 0xFFFFFFFF;

	f_read(&file,writebuf,12,&bytesread);
	sram_write(0xF000,writebuf,12);
	
//	printf("crc32 = %08X, stored crc32 = %08X\n",crc32,*((uint32_t*)((uint8_t*)writebuf + 4)));

	f_close(&file);
//	printf("firmware was %d bytes, crc32 is %08X\n",size,crc32);

	NVIC_SystemReset();
}

extern const char *bootloader_filename;

void update_bootloader(void)
{
	FRESULT fr;
	uint32_t i,data,bytesread;
	uint32_t size;

	fr = f_open(&file, bootloader_filename, FA_READ);
    if (fr != FR_OK) {
		return;
	}

	LED_RED_OFF();
	LED_GREEN_OFF();
	size = f_size(&file);

	memset(writebuf,0,256);
	for(i=0;i<0x1000;i+=256) {
		sram_write(i + 0x10000,writebuf,256);
	}

	for(i=0;i<size;) {
		memset(writebuf,0,256);		
		f_read(&file,writebuf,256,&bytesread);
		sram_write(i + 0x10000,writebuf,256);
		i += bytesread;
	}

	f_close(&file);
	
	FLASH_Unlock();

	//erase pages
	for(i=BOOTLOADER_ADDRESS;i<BOOTLOADER_ADDRESS+0x1000;i+=FLASH_PAGE_SIZE) {
		FLASH_ErasePage(i);
	}
	
	//program pages
	for(i=0;i<0x1000;i+=4) {
		sram_read(i + 0x10000,(uint8_t*)&data,4);
		FLASH_ProgramWord(i + BOOTLOADER_ADDRESS,data);
	}

	printf("bootloader updated\n");

	FLASH_Lock();

}

int find_disklist(void);
int insert_disklist(int block3);

//open file from sdcard
//--this doesnt keep the file open, it just opens it and checks
//--if it could be a valid fds image.

int file_exists(char *filename)
{
	FRESULT fr;

	fr = f_open(&file, filename, FA_READ);
    if (fr != FR_OK) {
		return(0);
	}	
	f_close(&file);
	return(1);
}

int fds_open(char *filename)
{
	FRESULT fr;
	int size, i, j;
	uint8_t buf[64];
	uint32_t bytesread = 0;

	//reset the diskinfo struct
	diskinfo.numsides = 0xFF;

	//try to open the file
	printf("opening '%s'\n",filename);
	fr = f_open(&file, filename, FA_READ);
    if (fr != FR_OK) {
		return (int)fr;
	}

	//get file size and read first 16 bytes
	size = f_size(&file);
	f_read(&file,buf,64,&bytesread);
	f_close(&file);
	if(bytesread != 64) {
		printf("error reading first 64 bytes of '%s'\n",filename);
		return(1);
	}

	//check for presence of fds header
	diskinfo.offset = 0;

	//detect fds format
	if (buf[0] == 'F' && buf[1] == 'D' && buf[2] == 'S' && buf[3] == 0x1A) {
		size -= 16;
		diskinfo.offset = 16;
		diskinfo.type = 0;
	}
	else if (buf[0] == 0x01 && buf[1] == 0x2A && buf[2] == 0x4E && buf[0x38] == 0x02) {
		diskinfo.type = 0;
	}

	//detect game doctor format
	else if (buf[3] == 0x01 && buf[4] == 0x2A && buf[5] == 0x4E && buf[0x3D] == 0x02) {
		diskinfo.type = 1;
	}

	//unknown format
	else {
//		hexdump("first bytes of disk image file",buf,64);
		printf("unknown disk image format\n");
		return(1);
	}
	
	//fds format
	strcpy(diskinfo.filename,filename);
	if(diskinfo.type == 0) {
		if(size % 65500) {
//			printf("error: file size is not a multiple of 65500\n");
			return(1);
		}
		diskinfo.numsides = size / 65500;
	}
	
	//doctor formats
	else {

		//find other disk sides
		j = strlen(diskinfo.filename) - 1;
		i = 0;
		while(file_exists(diskinfo.filename) != 0) {
			diskinfo.filename[j]++;
			i++;
		}

		//check for saver disk
		diskinfo.hassaver = 0;
		diskinfo.filename[j] = 'S';
		if(file_exists(diskinfo.filename) != 0) {
			diskinfo.hassaver = 1;
		}
		diskinfo.filename[j] = 'A';
		
		//save number of sides found
		diskinfo.numsides = i;
	}

	printf("file '%s' opened, %d sides (type = %d, hassaver = %d)\n",diskinfo.filename,diskinfo.numsides,diskinfo.type,diskinfo.hassaver);

	//save away disk info, it is valid disk
	diskinfo.curside = 0xFF;
	diskinfo.dirty = 0;
	return(0);
}

//close the current file opened from disk
void fds_close(void)
{
	diskinfo.numsides = 0xFF;
}

static int copy_block(uint8_t blockid, FIL *file, int addr, int size) {
	uint8_t data;
	uint32_t bytesread;
	uint32_t crc;
	int i,j,n;

	//verify block id
	f_read(file,&data,1,&bytesread);
	if(data != blockid) {
		printf("expected block id not found (wanted %X, found %X)\n",blockid,data);
		return(0);
	}

	//go back one byte
	f_lseek(file, f_tell(file) - 1);

//	printf("copy_block:  copying %d bytes to sram\n", size);

	//write block start marker
	data = 0x80;
	sram_write(addr++,&data,1);
	
	//copy the data
	crc = 0x8000;
	while(size) {
		
		//determine size of chunk to copy
		n = (size > 256) ? 256 : size;
		
		//read chunk from file, and write to sram
		f_read(file,writebuf,n,&bytesread);
		sram_write(addr,writebuf,bytesread);
		
//		hexdump("block1",fdsbuffer,n);
//		printf("copied %d bytes\n",bytesread);
		
		//calculate crc
		for(i=0;i<n;i++) {
			crc |= writebuf[i] << 16;
			for(j=0;j<8;j++) {
				if(crc & 1) {
					crc ^= 0x10810;
				}
				crc >>= 1;
			}
		}
		
		//adjust indexes as necessary
		addr += bytesread;
		size -= bytesread;
	}

	//finish calculating crc
	for(i=0;i<2;i++) {
		for(j=0;j<8;j++) {
			if(crc & 1) {
				crc ^= 0x10810;
			}
			crc >>= 1;
		}
	}
	
	//write crc to sram
	data = (uint8_t)crc;
	sram_write(addr++,&data,1);
	data = (uint8_t)(crc >> 8);
	sram_write(addr++,&data,1);
	
	//write gap to sram
	data = 0;
	for(i=0;i<GAP;i++) {
		sram_write(addr++,&data,1);
	}

	return(addr);
}

//load side from current file into sram
int fds_loadside_fds(int side)
{
	FRESULT fr;
	int addr;
	uint8_t data[4];
	uint32_t pos;

	//check if side is valid
	if(side > diskinfo.numsides) {
//		printf("invalid side chosen for disk (%d, total sides is %d), aborting fds_loadside\n",side,diskinfo.numsides);
		return(1);
	}
	
	//save the new side to the info struct
	diskinfo.curside = side;
	
	printf("opening fds '%s'\n",diskinfo.filename);
	fr = f_open(&file, diskinfo.filename, FA_READ);
    if (fr != FR_OK) {
		return (int)fr;
	}
	
	//file opened ok, clear the sram
	memset(writebuf,0,256);
	for(addr=0;addr<0x18000;addr+=256) {
		sram_write(addr,writebuf,256);
	}

	//seek to disk data start
	f_lseek(&file,diskinfo.offset + (side * 65500));
	
	//copy block 1
	addr = copy_block(1, &file, 0x100, 0x38);

	//copy block 2
	addr = copy_block(2, &file, addr, 2);
	
	//keep copying blocks 3 and 4
	while(addr) {
		
		//save block 3 address
		pos = addr;
		
		//copy block 3
		addr = copy_block(3, &file, addr, 16);

		//if previous block copy was success, copy block 4
		if(addr) {
			
			//get size of block 4
			sram_read(pos + 14,data,2);
			
			//copy block 4
			addr = copy_block(4, &file, addr, data[0] + (data[1] << 8) + 1);
		}
	}

	printf("disk side %d loaded\n",side);
	f_close(&file);

	return(0);
}

int fds_loadside_gd(int side)
{
	FRESULT fr;
	int addr, j;
	uint8_t data[4], tmp[4];
	uint32_t pos, bytesread;

	//check if side is valid
	if(side > diskinfo.numsides) {
		if(side != 0x80) {
			printf("invalid side chosen for disk (%d, total sides is %d), aborting fds_loadside\n",side,diskinfo.numsides);
			return(1);
		}
	}
	
	//save the new side to the info struct
	diskinfo.curside = side;
	
	//try to open the file
	j = strlen(diskinfo.filename) - 1;
	if(diskinfo.curside == 0x80) {
		diskinfo.filename[j] = 'S';
	}
	else {
		diskinfo.filename[j] = 'A' + side;
	}

	printf("opening gd '%s'\n",diskinfo.filename);
	fr = f_open(&file, diskinfo.filename, FA_READ);
    if (fr != FR_OK) {
		return (int)fr;
	}
	
	//file opened ok, clear the sram
	memset(writebuf,0,256);
	for(addr=0;addr<0x10000;addr+=256) {
		sram_write(addr,writebuf,256);
	}

	//seek to disk data start
	f_read(&file,tmp,3,&bytesread);
	
	//copy block 1
//	printf("copy block 1 from addr $%X (ftell = %d)\n",addr,f_tell(&file));
	addr = copy_block(1, &file, 0x100, 0x38);
	f_read(&file,tmp,2,&bytesread);
//	addr += 2;

	//copy block 2
//	printf("copy block 2 from addr $%X\n",addr);
	addr = copy_block(2, &file, addr, 2);
	f_read(&file,tmp,2,&bytesread);
//	addr += 2;
	
	//keep copying blocks 3 and 4
	while(addr > 0) {
		
		//save block 3 address
		pos = addr;
		
		//copy block 3
		addr = copy_block(3, &file, addr, 16);
		f_read(&file,tmp,2,&bytesread);
//		addr += 2;

		//if previous block copy was success, copy block 4
		if(addr > 0) {
			
			//get size of block 4
			sram_read(pos + 14,data,2);
			
			//copy block 4
			addr = copy_block(4, &file, addr, data[0] + (data[1] << 8) + 1);
			f_read(&file,tmp,2,&bytesread);
//			addr += 2;
		}
	}

	printf("disk side %d loaded\n",side);
	f_close(&file);

	return(0);
}

int fds_loadside(int side)
{
	if(diskinfo.type == 0) {
		return(fds_loadside_fds(side));
	}
	return(fds_loadside_gd(side));
}

#define FDSBUFSIZE	256

int bufpos;
int fdssize;

//write fds buffer to disk
void fds_flush(void)
{
	uint32_t wrote = 0;

	if (bufpos == 0)
		return;
	f_write(&file,writebuf,bufpos,&wrote);
	memset(writebuf, 0, FDSBUFSIZE);
	fdssize += bufpos;
	bufpos = 0;
}

//write one byte to fds buffer, and flush to disk if necessary
void fds_write(uint8_t data)
{
	writebuf[bufpos++] = data;

	if (bufpos == FDSBUFSIZE) {
		fds_flush();
	}
}

//rewind file pointer
void fds_rewind(int len)
{
	uint32_t pos;

	fds_flush();
	pos = f_tell(&file) - len;
	f_lseek(&file,pos);
}

int get_block_shift(uint8_t byte)
{
	int ret = 0;

	//if byte is bad, return
	if (byte == 0) {
		printf("invalid byte passed to get_block_shift, will return no shift\n");
//		return(-1);
	}
	while (ret < 8) {
		ret++;
		if (byte & 1) {
			break;
		}
		byte >>= 1;
	}
	return(ret & 7);
}

//byte is already aligned to where it needs to be
//nextbyte is not on 8bit boundary
//shift is how many bits to shift

//const uint8_t mask[8] = {0xFF,1,3,7,0xF,0x1F,0x3F,0x7F};
//const uint8_t bitshift[8] = {0,7,6,5,4,3,2,1};
uint8_t realign(uint8_t byte, uint8_t nextbyte, int shift)
{
/*	if(shift == 0) {
		byte = nextbyte;
	}
	else {
		byte |= (nextbyte & mask[shift]) << bitshift[shift];
	}*/
	switch (shift) {
	default:
	case 0:
		byte = nextbyte;
		break;
	case 1:
		byte |= (nextbyte & 1) << 7;
		break;
	case 2:
		byte |= (nextbyte & 3) << 6;
		break;
	case 3:
		byte |= (nextbyte & 7) << 5;
		break;
	case 4:
		byte |= (nextbyte & 0xF) << 4;
		break;
	case 5:
		byte |= (nextbyte & 0x1F) << 3;
		break;
	case 6:
		byte |= (nextbyte & 0x3F) << 2;
		break;
	case 7:
		byte |= (nextbyte & 0x7F) << 1;
		break;
	}
	return(byte);
}

//call at the beginning of the block, after the gap
//the first byte read must contain the "gap end" marker
int realign_block(int pos)
{
	static int block4size = 0;
	uint8_t byte, nextbyte, blockid;
	int i;
	int shift;
	int size;
	int start = pos;

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

	//write blockid to disk
	fds_write(byte);

	//put nextbyte into byte (with realignment)
	byte = nextbyte >> shift;

	//determine block size (includes block id)
	switch (blockid) {
	case 1:
		size = 56;
		break;
	case 2:
		size = 2;
		break;
	case 3:
		size = 16;
		break;
	case 4:
		size = block4size + 1;
		block4size = 0;
		break;
	default:
		printf("unknown block %d, start = %-5d, shift is %d\n", blockid, start, shift);
		return(-1);
	}

	//copy the rest of the bytes in the block (start at 1 because we already wrote the blockid byte)
	for (i = 1; i < size; i++) {

		//read next byte from sram
		sram_read(pos++, &nextbyte, 1);

		//realign the byte
		byte = realign(byte, nextbyte, shift);

		//if we are processing block 3, get the size for block4
		if (blockid == 3) {
			if (i == 13)
				block4size = byte;
			if (i == 14)
				block4size |= byte << 8;
		}

		//write byte to disk
		fds_write(byte);

		//put nextbyte into byte
		byte = nextbyte >> shift;
	}

	//skip over the crc bytes and get into the next gap
	pos += 3;

	printf("found block %d, start = %-5d, end = %-5d, shift is %d, size is %d\n", blockid, start, pos, shift, size);

	return(pos);
}


//use this to skip the gap period between files, after this call either disk
//has ended or it is ready for realign_block()
int skip_gap(int pos)
{
	uint8_t byte = 0;
	int size = 0;

	while (byte == 0 && pos < 0x18000) {
		sram_read(pos++, &byte, 1);

		//check if there is some kind of junk in the gap, if so, reset the gap counter
		if (byte && size < (MIN_GAP_SIZE / 8)) {
//			printf("junk in gap at %d\n", pos);
			size = 0;
			byte = 0;
		}

		//increment the gap size counter
		else {
			size++;
		}
	}

	//if there was some junk in the gap, abort the conversion
	if (size < (MIN_GAP_SIZE / 8)) {
		return(0x18000);
	}

	//if we have reached the end of the sram
	if (pos == 0x18000) {
		return(0x18000);
	}

	//return position of the end of the gap, the start of the next block
	return(pos - 1);
}



uint8_t get_bit(uint8_t *b, int bSize)
{
	static int in;
	static uint8_t bit, data, rawbit;
	static int pos;
	uint8_t ret = 0xFF;

	//reset to initial state
	if (b || bSize) {
		rawbit = 0xff;
		bit = 1;
		in = 0;
		pos = 0;
		return(0xFE);
	}

	while ((in < (0x14000 * 8)) && (ret == 0xFF)) {
		if ((in & 7) == 0) {
			sram_read(pos++, &data, 1);
		}
		bit = (bit << 7) | (1 & (data >> (in & 7)));
		switch (bit) {
		case 0x00:  //10 10
			ret = rawbit;
			rawbit = 0;
			break;
		case 0x01:  //10 01
		case 0x81:  //01 01
			rawbit++;
			ret = rawbit;
			rawbit = 0xff;
			break;
		case 0x80:  //01 10
			rawbit += 2;
			break;
		}
		in++;
	}
	return(ret);
}

uint32_t crc_byte(uint32_t crc, uint8_t byte)
{
	int i;

	crc |= byte << 16;
	for (i = 0; i<8; i++) {
		if (crc & 1) {
			crc ^= 0x10810;
		}
		crc >>= 1;
	}
	return(crc);
}

int block_decode_single(int *outP, int blockSize, char blockType) {
	int outEnd;
	int out = (*outP) * 8;
	int start;
	uint8_t bit, byte;
	uint32_t crc = 0x8000;
	static uint16_t block4size;
	uint8_t first16[16];
	int pos;

	//if blocktype is 4 then get saved data from processing block3
	if (blockType == 4) {
		blockSize = 1 + block4size;
	}

	outEnd = (*outP + blockSize + 2) * 8;

	pos = 0;

	//scan for gap end
	start = 0;
	bit = get_bit(0, 0);
	for (int zeros = 0; bit != 1 || zeros<MIN_GAP_SIZE; start++) {
		if (bit == 0) {
			zeros++;
		}
		else {
			zeros = 0;
		}
		if (bit == 0xFF) {
			printf("error finding gap end\n");
			return 0;
		}
		bit = get_bit(0, 0);
	}
//	printf("gap end found at %d\n", start);

	char bitval = 1;
	byte = 0;
	do {
		bit = get_bit(0, 0);
		switch (bit | (bitval << 4)) {
		case 0x11:
			if ((out & 7) == 7) {
				fds_write(byte);
				if (pos < 16)	first16[pos++] = byte;
				//dst[out / 8] = byte;
				crc = crc_byte(crc, byte);
				byte = 0;
			}
			out++;
		case 0x00:
			if ((out & 7) == 7) {
				fds_write(byte);
				if (pos < 16)	first16[pos++] = byte;
				//dst[out / 8] = byte;
				crc = crc_byte(crc, byte);
				byte = 0;
			}
			out++;
			bitval = 0;
			break;

		case 0x12:
			if ((out & 7) == 7) {
				fds_write(byte);
				if (pos < 16)	first16[pos++] = byte;
				//dst[out / 8] = byte;
				crc = crc_byte(crc, byte);
				byte = 0;
			}
			out++;
		case 0x01:
		case 0x10:
			byte |= 1 << (out & 7);
			if ((out & 7) == 7) {
				fds_write(byte);
				if (pos < 16)	first16[pos++] = byte;
				//dst[out / 8] = byte;
				crc = crc_byte(crc, byte);
				byte = 0;
			}
			out++;
			bitval = 1;
			break;

		default: //Unexpected value.  Keep going, we'll probably get a CRC warning
			printf("glitch(%d) @ %X(%X.%d)\r\n", bit, 0, out/8, out%8);
			if ((out & 7) == 7) {
				fds_write(byte);
				if (pos < 16)	first16[pos++] = byte;
				//dst[out / 8] = byte;
				crc = crc_byte(crc, byte);
				byte = 0;
			}
			out++;
			bitval = 0;
			break;
		}
	} while (out<outEnd);
	if (first16[0] != blockType) {
		printf("Wrong block type %X(%X)-%X(%X) (found %d, expected %d)\r\n", 0, *outP, 0, out - 1, first16[0], blockType);
		return 0;
	}

	if (blockType == 3) {
		block4size = first16[13] | (first16[14] << 8);
	}
	out = out / 8 - 2;
	fds_rewind(2);

	//messages_printf("Out%d %X(%X)-%X(%X)\r\n", blockType, start, *outP, in, out-1);

	printf("block type %d, size = %d, crc = %04X\n", blockType, blockSize, crc);
	if (crc != 0) {
		printf("Bad CRC (%04X)\r\n", crc);
	}

	*outP = out;
	return 1;

}

void bin2fds(void)
{
	int out = 0;

	get_bit(0,1);

	if (!block_decode_single(&out, 0x38, 1))
		return;
	if (!block_decode_single(&out, 2, 2))
		return;
	for(;;) {
		if (!block_decode_single(&out, 16, 3))
			break;
		if (!block_decode_single(&out, 0, 4))
			break;
	}

	fds_flush();
}

void bin2gd(void)
{
	int out = 0;

	get_bit(0,1);

	fds_write(0x00);
	fds_write(0x00);
	fds_write(0x80);

	if (!block_decode_single(&out, 0x38, 1))
		return;
	fds_write(0x00);
	fds_write(0x00);

	if (!block_decode_single(&out, 2, 2))
		return;
	fds_write(0x00);
	fds_write(0x00);

	for(;;) {
		if (!block_decode_single(&out, 16, 3))
			break;
		fds_write(0x00);
		fds_write(0x00);

		if (!block_decode_single(&out, 0, 4))
			break;
		fds_write(0x00);
		fds_write(0x00);
	}
	
	fds_flush();
}

//save sram to file (fds format)
int fds_savesram(void)
{
	FRESULT fr;
	int size;
	uint32_t wrote = 0;

	//try to open the file
	printf("fds_savesram: opening '%s', saving disk side %d\n",diskinfo.filename,diskinfo.curside);
	fr = f_open(&file, diskinfo.filename, FA_WRITE);

	//verify file was opened ok
    if (fr != FR_OK) {
		printf("fds_savesram: error opening file\n");
		return (int)fr;
	}

	//seek to current disk side
	f_lseek(&file,diskinfo.curside * 65500 + diskinfo.offset);

	fdssize = 0;
	bin2fds();

	if(fdssize < 65500) {
		
		//get remaining data size
		size = 65500 - fdssize;
		
		//zero out the buffer
		memset(writebuf,0,FDSBUFSIZE);
		
		//write zeroes to the rest of the side
		while(size > 0) {
			f_write(&file,writebuf,size > 256 ? 256 : size,&wrote);
			fdssize += wrote;
			size -= 256;
		}
	}
	else {
		printf("fds_savesram: size of data is too much\n");		
	}
	
	printf("fds_savesram: fdssize = %d\n",fdssize);

	//close file
	f_close(&file);

//	printf("fds_savesram: fds file saved to sdcard\n");
	
	return(0);
}

//save sram to file (game doctor format)
int fds_savesram_gd(void)
{
	FRESULT fr;
	uint8_t byte;
	uint32_t bytesread = 0;

/*	fr = f_open(&file, "sram.bin", FA_CREATE_NEW | FA_WRITE);
	for(i=0;i<256;i++) {
		sram_read(i * 256,writebuf,256);
		f_write(&file,writebuf,256,&bytesread);
	}
	f_close(&file);
	printf("wrote sram to sram.bin\n");*/

	//try to open the file
	printf("fds_savesram_gd: opening '%s', saving disk side %d\n",diskinfo.filename,diskinfo.curside);
	fr = f_open(&file, diskinfo.filename, FA_WRITE);

	//verify file was opened ok
    if (fr != FR_OK) {
		printf("fds_savesram_gd: error opening file\n");
		return (int)fr;
	}

	fdssize = 0;
	bin2gd();

	//close file
	f_close(&file);

	fr = f_open(&file, diskinfo.filename, FA_WRITE | FA_READ);

	//verify file was opened ok
    if (fr != FR_OK) {
		printf("fds_savesram_gd: error re-opening file\n");
		return (int)fr;
	}

	//read file number count
	f_lseek(&file,62);
	f_read(&file,&byte,1,&bytesread);
	byte |= 0x80;
	f_lseek(&file,2);
	f_write(&file,&byte,1,&bytesread);
	f_close(&file);

	printf("fds_savesram_gd: file saved, fdssize = %d\n",fdssize);
	
	return(0);
}

void fds_init(void)
{
	diskinfo.numsides = 0xFF;
}

extern volatile int SramSaveCounter;
volatile int SramDirty = 0;

void fds_tick(void)
{
	int needsave = 0;

	//check for disk flip button press
	//only allow disk flip if the current data has been saved to sdcard
	if(BUTTON_DOWN() && diskinfo.numsides != 0xFF && SramDirty == 0) {
		uint8_t newside = diskinfo.curside + 1;

		LED_RED_ON();
		CLEAR_MEDIASET();
		CLEAR_WRITABLE();
		while(BUTTON_DOWN());
		
		//check for wrap-around
		if(newside >= diskinfo.numsides) {
			
			//if this disk has a saver disk, lock onto saver disk
			if(diskinfo.hassaver) {
				newside = 0x80;
			}
			
			//wrap around to first disk
			else {
				newside = 0;
			}
		}
		printf("new disk side is %d\n",newside);
		fds_loadside(newside);
		SET_MEDIASET();
		SET_WRITABLE();
		LED_RED_OFF();
	}

	//check if ram adaptor wants to stop the motor
	if(IS_STOPMOTOR()) {
		CLEAR_MOTORON();
		return;
	}

	//ram adaptor not trying to stop the motor, check if it wants to scanmedia
	while(IS_SCANMEDIA() && IS_STOPMOTOR() == 0) {
		SET_MOTORON();
		LED_GREEN_ON();
		LED_RED_ON();
		if(diskinfo.numsides == 0xFF) {
			begin_transfer_loader();
		}
		else {
			needsave |= begin_transfer();
		}
		LED_RED_OFF();
		
		//delay 1000ms to save the sram disk
		if(needsave) {
			SramDirty = 1;
//			printf("queue sram write\n");
		}
		
		//if a save is pending, put led into orange state
		if(SramDirty != 0) {
			LED_GREEN_ON();
			LED_RED_ON();
		}
		
		//start the sram save counter and enable the systick irq
		SramSaveCounter = SRAM_COMMIT_DELAY;
		NVIC_EnableIRQ(SysTick_IRQn);
	}

	//check sram save timer
	if(SramDirty && SramSaveCounter == 0) {
//	if(needsave) {
//		CLEAR_MEDIASET();
//		CLEAR_WRITABLE();
		printf("timer up...dirty, writing...\n");
		SramDirty = 0;
		LED_RED_ON();
		LED_GREEN_OFF();
		if(diskinfo.type == 0) {
			fds_savesram();
		}
		else {
			fds_savesram_gd();
		}
		LED_GREEN_ON();
		LED_RED_OFF();
//		SET_MEDIASET();
//		SET_WRITABLE();
	}
}
