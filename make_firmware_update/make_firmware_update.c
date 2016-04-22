#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* crc_tab[] -- this crcTable is being build by chksum_crc32GenTab().
 *		so make sure, you call it before using the other
 *		functions!
 */
static uint32_t crc_tab[256];

/* crc32gentab() -- to a global crc_tab[256], this one will
 *                  calculate the crcTable for crc32-checksums.
 *                  it is generated to the polynom [..]
 */

void crc32_gentab()
{
	unsigned long crc,poly;
	int i,j;

	poly = 0xEDB88320L;
	for(i=0;i<256;i++) {
		crc = i;
		for(j=8;j>0;j--) {
			if(crc & 1)
				crc = (crc >> 1) ^ poly;
			else
				crc >>= 1;
		}
      crc_tab[i] = crc;
	}
}

/* crc32() -- to a given block, this one calculates the
 *            crc32-checksum until the length is
 *            reached. the crc32-checksum will be
 *            the result.
 */
uint32_t revbit(uint32_t data)
{
  int i;
  uint32_t rev = 0;
  for(i=0; i<32; i++)
    rev |= ((data >> i) & 1) << (31 - i);
  return rev;
};

uint32_t crc32(unsigned char *block,unsigned int length)
{
	register unsigned long crc;
	unsigned long i;

	crc = 0xFFFFFFFF;
	for(i=0;i<length;i++) {
		crc = ((crc >> 8) & 0x00FFFFFF) ^ crc_tab[(crc ^ *block++) & 0xFF];
	}
	return(crc ^ 0xFFFFFFFF);
}

//nestopia's implementation
static uint32_t crc32_calc(uint8_t data,uint32_t crc)
{
	return((crc >> 8) ^ crc_tab[(crc ^ data) & 0xFF]);
}

uint32_t crc32_byte(uint8_t data,uint32_t crc)
{
	return(crc32_calc(data,crc ^ 0xFFFFFFFF) ^ 0xFFFFFFFF);
}

uint32_t crc32_block(uint8_t *data,uint32_t length,uint32_t crc)
{
	unsigned char *end;

	crc ^= 0xFFFFFFFF;
	for(end=data+length;data != end;data++)
		crc = crc32_calc(*data,crc);
	return(crc ^ 0xFFFFFFFF);
}

uint8_t bootloader[0x1000];
uint8_t fw[0xF000 + 12];

int get_firmware_build(uint8_t *buf)
{
	uint8_t ident[8] = { 0xEF, 0xBE, 0xAD, 0xDE, 0xEF, 0xBE, 0xFE, 0xCA };
	int pos = 0;

	while (pos < 0x7000) {
		if (buf[pos] == ident[0]) {
			if (memcmp(buf + pos, ident, 8) == 0) {
				buf += pos;
				buf += 8;
				return(*buf | (*(buf + 1) << 8));
			}
		}
		pos++;
	}
	return(0);
}

int main(int argc,char *argv[])
{
	FILE *fin,*fout;
	char *infn = "stm32-fdsemu.bin";
	char *bootfn = "stm32-bootloader.bin";
	char *outfn = "magicwildcard.bin";
	char *flashfn = "flash-magicwildcard.bin";

	uint32_t *ptr32;
	uint32_t crc32, magic = 0xDEADBEEF;
	size_t sz,bootsz;

	crc32_gentab();

	memset(fw, 0, 0x7000 + 12);
	memset(bootloader, 0, 0x1000);

	if ((fin = fopen(bootfn, "rb")) == 0) {
		printf("error opening '%s'\n", bootfn);
		return(1);
	}

	fseek(fin, 0, SEEK_END);
	bootsz = ftell(fin);
	fseek(fin, 0, SEEK_SET);

	if (bootsz > 0x7000) {
		printf("bootloader too large\n");
		fclose(fin);
		return(1);
	}

	fread(bootloader, 1, bootsz, fin);
	fclose(fin);

	if ((fin = fopen(infn, "rb")) == 0) {
		printf("error opening '%s'\n", infn);
		return(1);
	}

	fseek(fin, 0, SEEK_END);
	sz = ftell(fin);
	fseek(fin, 0, SEEK_SET);

	if (sz > 0x7000) {
		printf("firmware too large\n");
		fclose(fin);
		return(1);
	}

	fread(fw, 1, sz, fin);
	fclose(fin);

	printf("input firmware size = %d bytes (%d bytes to pad), build = %d\n", sz, 0x7000 - sz,get_firmware_build(fw));
	printf("input bootloader size = %d bytes (%d bytes to pad)\n", bootsz, 0x1000 - bootsz);

	crc32 = crc32_block(fw,0x7000,0);

	ptr32 = (uint32_t*)(((uint8_t*)fw) + 0x7000);
	*ptr32++ = (uint32_t)sz;
	*ptr32++ = crc32;
	*ptr32++ = magic;

	printf("crc32 = %08X\n",crc32);

	fout = fopen(outfn, "wb");
	fwrite(fw, 1, 0x7000 + 12, fout);
	fclose(fout);

	fout = fopen(flashfn, "wb");
	fwrite(bootloader, 1, 0x1000, fout);
	fwrite(fw, 1, 0x7000, fout);
	fclose(fout);

	return(0);
}