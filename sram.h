#ifndef __sram_h__
#define __sram_h__

extern const uint32_t testid;

void sram_read(int addr,uint8_t *buf,int len);
void sram_write(int addr,uint8_t *buf,int len);
int sram_test(void);
int sram_init(void);

#endif
