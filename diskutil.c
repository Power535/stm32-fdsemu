#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "diskutil.h"
#include "fds.h"

#include "xprintf.h"

//don't include gap end
uint16_t calc_crc(uint8_t *buf, int size) {
	uint32_t crc = 0x8000;
	int i;
	while (size--) {
		crc |= (*buf++) << 16;
		for (i = 0; i<8; i++) {
			if (crc & 1) crc ^= 0x10810;
			crc >>= 1;
		}
	}
	return crc;
}

void copy_block(uint8_t *dst, uint8_t *src, int size) {
	dst[0] = 0x80;
	memcpy(dst + 1, src, size);
	uint32_t crc = calc_crc(dst + 1, size + 2);
	dst[size + 1] = crc;
	dst[size + 2] = crc >> 8;
	//	printf("copying block type %d, size = %d, crc = %04X\n", dst[1],size + 2, crc);
}

//Adds GAP + GAP end (0x80) + CRCs to .FDS image
//Returns size (0=error)
int fds_to_bin(uint8_t *dst, uint8_t *src, int dstSize) {
	int i = 0, o = 0;

	//check *NINTENDO-HVC* header
	if (src[0] != 0x01 || src[1] != 0x2a || src[2] != 0x4e) {
		printf("Not an FDS file.\n");
		return 0;
	}
	memset(dst, 0, dstSize);

	//block type 1
	copy_block(dst + o, src + i, 0x38);
	i += 0x38;
	o += 0x38 + 3 + GAP;

	//block type 2
	copy_block(dst + o, src + i, 2);
	i += 2;
	o += 2 + 3 + GAP;

	//block type 3+4...
	while (src[i] == 3) {
		int size = (src[i + 13] | (src[i + 14] << 8)) + 1;
		if (o + 16 + 3 + GAP + size + 3 > dstSize) {    //end + block3 + crc + gap + end + block4 + crc
//			printf("Out of space (%d bytes short), adjust GAP size?\n", (o + 16 + 3 + GAP + size + 3) - dstSize);
			return 0;
		}
		copy_block(dst + o, src + i, 16);
		i += 16;
		o += 16 + 3 + GAP;

		copy_block(dst + o, src + i, size);
		i += size;
		o += size + 3 + GAP;
	}
	return o;
}

int gameDoctor_to_bin(uint8_t *dst, uint8_t *src, int dstSize) {
	//check for *NINTENDO-HVC* at 0x03 and second block following CRC
	if (src[3] != 0x01 || src[4] != 0x2a || src[5] != 0x4e || src[0x3d] != 0x02) {
//		printf("Not GD format.\r\n");
		return 0;
	}
	memset(dst, 0, dstSize);

	//	messages_printf("converting image, max size = %d\r\n", dstSize);

	//block type 1
	int i = 3, o = 0;
	copy_block(dst + o, src + i, 0x38);
	i += 0x38 + 2;        //block + dummy crc
	o += 0x38 + 3 + GAP;    //gap end + block + crc + gap

							//block type 2
	copy_block(dst + o, src + i, 2);
	i += 2 + 2;
	o += 2 + 3 + GAP;

	//block type 3+4...
	while (src[i] == 3) {
		int size = (src[i + 13] | (src[i + 14] << 8)) + 1;

		if (o + 16 + 3 + GAP + size + 3 > dstSize) {    //end + block3 + crc + gap + end + block4 + crc
//			printf("Out of space (%d bytes short), adjust GAP size?\r\n", (o + 16 + 3 + GAP + size + 3) - dstSize);
			return 0;
		}
		copy_block(dst + o, src + i, 16);
		i += 16 + 2;
		o += 16 + 3 + GAP;

		copy_block(dst + o, src + i, size);
		i += size + 2;
		o += size + 3 + GAP;
	}
	return o;
}
