#ifndef __diskutil_h__
#define __diskutil_h__

#include <stdint.h>

uint16_t calc_crc(uint8_t *buf, int size);
int fds_to_bin(uint8_t *dst, uint8_t *src, int dstSize);

#endif
