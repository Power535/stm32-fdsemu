#ifndef __transfer_h__
#define __transfer_h__

#define WRITEBUF_SIZE	(1024)

extern uint8_t writebuf[];

int begin_transfer(void);
void begin_transfer_loader(void);

#endif
