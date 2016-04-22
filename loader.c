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

#define FORCE_SORT_FILENAME "force.sort"
#define BOOTLOADER_FILENAME "magicbootloader.bin"

const char *bootloader_filename = BOOTLOADER_FILENAME;

void put_rc (FRESULT rc);

DIR Dir;	/* Directory object */
FILINFO Finfo;

//string to find to start sending the fake disklist
const uint8_t diskliststr[17] = {0x80,0x03,0x07,0x10,'D','I','S','K','L','I','S','T',0x00,0x70};	//omitted bytes: size lo, size hi, type (0=prg)

#if _USE_LFN
#define LFN_SIZE	128
#endif

#define MAX_FILES	815

extern char loader_lz4[];
extern int loader_lz4_length;

/*
	decompress lz4 data.

	buf = raw lz4 data, including 16 byte header
	cb_read = callback for reading uncompressed data
	cb_write = callback for writing uncompressed data
*/
int decompress_lz4(uint8_t(*cb_readsrc)(uint32_t), int srclen, uint8_t(*cb_read)(uint32_t), void(*cb_write)(uint32_t,uint8_t))
{
	uint8_t token, tmp;
	int inlen = 0;
	int outlen = 0;
	uint32_t offset;
	uint32_t n;

	inlen += 4;
	inlen += 7;

	//loop thru
	while (inlen < srclen) {
		token = cb_readsrc(inlen++);

		//literal part
		if ((token >> 4) & 0xF) {

			//calculate literal length
			n = (token >> 4) & 0xF;

			//length of 15 or greater
			if (n == 0xF) {
				do {
					tmp = cb_readsrc(inlen++);
					n += tmp;
				} while (tmp == 0xFF);
			}

			//write literals to output
			while (n--) {
				cb_write(outlen++, cb_readsrc(inlen++));
			}
		}

		//match part (if it is there)
		if ((inlen + 12) >= srclen) {
			break;
		}

		//get match offset
		offset = cb_readsrc(inlen++);
		offset |= cb_readsrc(inlen++) << 8;

		//calculate match length
		n = token & 0xF;

		//length of 15 or greater
		if (n == 0xF) {
			do {
				tmp = cb_readsrc(inlen++);
				n += tmp;
			} while (tmp == 0xFF);
		}

		//add 4 to match length
		n += 4;
		offset = outlen - offset;

		//copy match bytes
		while (n--) {
			tmp = cb_read(offset++);
			cb_write(outlen++, tmp);
		}
	}

	return(outlen);
}

static uint8_t lz4_readsrc(uint32_t addr)
{
	return((uint8_t)loader_lz4[addr]);
}

static uint8_t lz4_read(uint32_t addr)
{
	uint8_t ret;

	sram_read(addr,&ret,1);
	return(ret);
}

static void lz4_write(uint32_t addr, uint8_t data)
{
	sram_write(addr,&data,1);
}

int find_disklist(void)
{
	int pos = 0;
	uint8_t byte;
	uint8_t tmp[16];

	for(pos=0;pos<65500;) {

		//read a byte from the flash
		sram_read(pos,(uint8_t*)&byte,1);
		pos++;
		
		//first byte matches
		if(byte == diskliststr[0]) {
			
			//read remaining bytes
			sram_read(pos,tmp,13);
			
			//string matches?
			if(memcmp(diskliststr + 1,tmp,13) == 0) {
				return(pos);
			}
		}
	}
	return(-1);
}

static uint16_t calc_crc2(uint8_t (*readfunc)(uint32_t), uint32_t pos, int size) {
    uint32_t crc=0x8000;
    int i;
    while(size--) {
        crc |= readfunc(pos++)<<16;
        for(i=0;i<8;i++) {
            if(crc & 1) crc ^= 0x10810;
            crc>>=1;
        }
    }
    return crc;
}

__forceinline unsigned char tolower2(unsigned char ch) {
    if (ch >= 'A' && ch <= 'Z')
        ch = 'a' + (ch - 'A');
    return ch;
 }

int strcmpi(const char *s1, const char *s2) {
    const unsigned char *us1 = (const unsigned char *)s1,
                        *us2 = (const unsigned char *)s2;

    while (tolower2(*us1) == tolower2(*us2++))
        if (*us1++ == '\0')
            return (0);
    return (tolower2(*us1) - tolower2(*--us2));
}

int disklistpos = 0;

__forceinline void copy_to_temp(int n1, int n2)
{
	uint8_t s[40];

	sram_read(n2 * 40 + disklistpos, s, 40);
	sram_write(n1 * 40 + disklistpos + 0x10000, s, 40);
}

__forceinline void copy_from_temp(int n1, int n2)
{
	uint8_t s[40];

	sram_read(n1 * 40 + disklistpos + 0x10000, s, 40);
	sram_write(n2 * 40 + disklistpos, s, 40);
}

__forceinline void copy_to_temp_block(int temp_p, int src_p, int num)
{
	register int sz = num * 40;
	register int inc_p = (int)(WRITEBUF_SIZE / 40);
	register int inc = inc_p * 40;
	register int bytes;

	while(sz > 0) {
		bytes = sz > inc ? inc : sz;
		sram_read(src_p * 40 + disklistpos, writebuf, bytes);
		sram_write(temp_p * 40 + disklistpos + 0x10000, writebuf, bytes);
		src_p += inc_p;
		temp_p += inc_p;
		sz -= inc;
	}
}

__forceinline void copy_from_temp_block(int loc, int num)
{
	register int sz = num * 40;
	register int inc = (int)(WRITEBUF_SIZE / 40) * 40;
	register int bytes;

	while(sz > 0) {
		bytes = sz > inc ? inc : sz;
		sram_read(loc * 40 + disklistpos + 0x10000, writebuf, bytes);
		sram_write(loc * 40 + disklistpos, writebuf, bytes);
		loc += WRITEBUF_SIZE / 40;
		sz -= inc;
	}
}

void merge_sort(int pos, int num)
{
    int rght, rend, left;
    int i,j,k,m;
	uint8_t si[40],sj[40];
	int toggle = 0;

	disklistpos = pos;
    for (k=1; k < num; k *= 2 ) {       
        for (left=0; left+k < num; left += k*2 ) {
            rght = left + k;        
            rend = rght + k;
            if (rend > num)
				rend = num; 
            m = left;
			i = left;
			j = rght; 
            while (i < rght && j < rend) { 
				sram_read(i * 40 + pos, si, 40);
				sram_read(j * 40 + pos, sj, 40);
				if(strcmpi((char*)(si + 12),(char*)(sj + 12)) < 0) {
					sram_write(m * 40 + pos + 0x10000, si, 40);
					i++;
				}
				else {
					sram_write(m * 40 + pos + 0x10000, sj, 40);
					j++;
				}
                m++;
            }

			copy_to_temp_block(m,i,rght - i);
			m += rght - i;

			copy_to_temp_block(m,j,rend - j);
			m += rend - j;

			copy_from_temp_block(left,rend - left);

/*			while (i < rght) { 
				copy_to_temp(m,i); //b[m] = a[i];
                i++; m++;
            }
			while (j < rend) {
                copy_to_temp(m,j); //b[m] = a[j];
                j++; m++;
            }
            for (m=left; m < rend; m++) { 
				copy_from_temp(m,m); //a[m] = b[m];
            }*/

			toggle ^= 1;
			LED_RED_OFF();
			if(toggle) {
				LED_RED_ON();
			}
        }
    }
}

//extern volatile int TimeCounter;

int insert_disklist(int block3)
{
	int block4 = 0;
	int pos, num = 0;
	uint8_t byte = 0;
	uint16_t crc;
	uint32_t data32;
	int disklistsize;
	int ledtoggle = 0;
	int forcesort = 1;

	BYTE res;
	char *filename, *ptr;
	char ext[8];
	uint8_t diskinfo[44];

	pos = block3 + 17 + 2;
	while(byte == 0) {
		sram_read(pos++,&byte,1);
	}
	block4 = pos;
	
	//this now points to beginning of block4 data
	block4++;
	
	printf("block 3, 4: %d, %d\n",block3,block4);
	
	//this points to first disk list entry in block4
	pos = block4 + 40;

//	NVIC_EnableIRQ(SysTick_IRQn);
//	TimeCounter = 0;

	res = f_opendir(&Dir, "");
	if (res) {
		printf("opendir: ");
		put_rc((FRESULT)res);
		return(1);
	}
	
	for(;;) {

		//check if we need to wiggle the led's
		if((num & 0xF) == 0) {
			ledtoggle ^= 1;
			if(ledtoggle) {
				LED_GREEN_ON();
				LED_RED_OFF();
			}
			else {
				LED_GREEN_OFF();
				LED_RED_ON();
			}
		}
		
		//read next file info
		Finfo.lfname = (char*)writebuf;
		Finfo.lfsize = LFN_SIZE;
		res = f_readdir(&Dir, &Finfo);
		
		//out of files, bail
		if ((res != FR_OK) || !Finfo.fname[0]) {
			break;
		}

		//directory, skip this one
		if (Finfo.fattrib & AM_DIR) {
			continue;
		}

		//check file extension
		filename = Finfo.lfname;
		if(Finfo.lfname[0] == 0) {
			filename = Finfo.fname;
		}

		if(strcmp(filename,FORCE_SORT_FILENAME) == 0) {
			forcesort = 1;
		}

/*		if(strcmp(filename,bootloader_filename) == 0) {
			update_bootloader();
//			f_unlink(bootloader_filename);
		}*/

		ptr = strrchr(filename,'.');
		
		if(ptr != 0) {
//			printf("file: '%s', extension '%s'\n",filename,ext);
			strncpy(ext,ptr,8);
			ptr = ext;
			while(*ptr) {
				*ptr = tolower(*ptr);
				ptr++;
			}
			if(memcmp(ext,".fds",4) == 0 || memcmp(ext,".a",2) == 0) {
//				printf("++");
				num++;
				memset(diskinfo,0,40);
				memcpy(diskinfo,Finfo.fname,12);
				memcpy(diskinfo+12,filename,26);
				sram_write(pos,diskinfo,40);
				pos += 40;
			}
/*			else {
				printf("  ");
			}
			printf(" :: '%s' :: ", ext);
			printf("file: '%s' == '%s'\n",Finfo.fname,filename);
*/		}
	}
	//put led back into initial states
	LED_GREEN_ON();
	LED_RED_OFF();
	
//	printf("list generated in %d ms\n",TimeCounter);
//	TimeCounter = 0;
	
	//only sort if the number of files is less than specified number
	if(num < 200 || forcesort != 0) {
		merge_sort(block4 + 40, num);
		LED_RED_OFF();
	}

	//sort disklist
//	printf("list sorted in %d ms\n",TimeCounter);
/*	printf("sorted:\n");
	pos = block4 + 40;
	for(i=0;i<num;i++) {
		sram_read(pos,diskinfo,40);
		printf("%d:  %s\n",i,diskinfo+12);
		pos += 40;
	}*/
	

	//calculate disklistsize
	disklistsize = pos - block4;

	//make sure that the crc bytes are zero (for crc)
	memset(diskinfo,0,40);
	sram_write(pos,diskinfo,40);
	sram_write(block3 + 16,diskinfo,2);

	//put num into two bytes and write it to sram
	diskinfo[0] = (uint8_t)num;
	diskinfo[1] = (uint8_t)(num >> 8);
	printf("disks found: %d\n",num);
	sram_write(block4,diskinfo,2);
	
	//save firmware build number to sram
	diskinfo[0] = (uint8_t)BUILDNUM;
	diskinfo[1] = (uint8_t)(BUILDNUM >> 8);
	sram_write(block4+38,diskinfo,2);
	
	//check for firmware update result magic number
	sram_read(0x20000 - 8,(uint8_t*)&data32,4);
	if(data32 == 0xDEADBEEF) {
		
		//put some keys into the status area, for success updating firmware
		diskinfo[0] = (uint8_t)0xDB;
		diskinfo[1] = (uint8_t)0xDC;
		sram_write(block4+36,diskinfo,2);	

		//erase firmware update key from sram
		data32 = 0;
		sram_write(0x20000 - 8,(uint8_t*)&data32,4);
	}

	//calculate block crc and store it to sram
	crc = calc_crc2(lz4_read,block4 - 1,disklistsize + 2 + 1);
	diskinfo[0] = (uint8_t)(crc >> 0);
	diskinfo[1] = (uint8_t)(crc >> 8);
	sram_write(block4 + disklistsize,diskinfo,2);

	//fixup block3 file size
	diskinfo[0] = (uint8_t)(disklistsize >> 0);
	diskinfo[1] = (uint8_t)(disklistsize >> 8);
	sram_write(block3 + 13,diskinfo,2);

	//calculate block crc and store it to sram
	crc = calc_crc2(lz4_read,block3,16 + 2);
	diskinfo[0] = (uint8_t)(crc >> 0);
	diskinfo[1] = (uint8_t)(crc >> 8);
	sram_write(block3 + 16,diskinfo,2);

	return(0);
}

void loader_copy()
{
	int ret;

	ret = decompress_lz4(lz4_readsrc,loader_lz4_length,lz4_read,lz4_write);
//	printf("decompressed loader: %d -> %d bytes\n",loader_lz4_length,ret);
	ret = find_disklist();

	insert_disklist(ret);
}
