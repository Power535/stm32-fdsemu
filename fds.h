#ifndef __fds_h__
#define __fds_h__

#include <stdint.h>

#define SET_READY()			GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define SET_MEDIASET()		GPIO_ResetBits(GPIOB,GPIO_Pin_5)
#define SET_WRITABLE()		GPIO_ResetBits(GPIOB,GPIO_Pin_3)
#define SET_MOTORON()		GPIO_SetBits(GPIOA,GPIO_Pin_15)

#define CLEAR_READY()		GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define CLEAR_MEDIASET()	GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define CLEAR_WRITABLE()	GPIO_SetBits(GPIOB,GPIO_Pin_3)
#define CLEAR_MOTORON()		GPIO_ResetBits(GPIOA,GPIO_Pin_15)

#define IS_WRITE()			((GPIOA->IDR & GPIO_Pin_10) == 0)
#define IS_SCANMEDIA()		((GPIOA->IDR & GPIO_Pin_12) == 0)
#define IS_STOPMOTOR()		((GPIOB->IDR & GPIO_Pin_7) == 0)
#define IS_DONT_STOPMOTOR()	((GPIOB->IDR & GPIO_Pin_7) != 0)

typedef struct diskinfo_s {
	char		filename[256];				//filename of disk
	uint32_t	offset;						//offset to disk data (skipping of header)
	uint8_t		numsides;					//total number of disk sides
	uint8_t		curside;					//currently selected disk side
	uint8_t		dirty;						//is current side of disk 'dirty'
	uint8_t		type;						//type of disk (0=fds, 1=gamedoctor)
	uint8_t 	hassaver;					//if this gamedoctor has saver disk
} diskinfo_t;

enum {
	DEFAULT_LEAD_IN = 28300,      //#bits (~25620 min)
	GAP = 976 / 8 - 1,                //(~750 min)
	MIN_GAP_SIZE = 0x300,         //bits
	FDSSIZE = 65500,              //size of .fds disk side, excluding header
};

extern diskinfo_t diskinfo;

int fds_open(char *filename);
void fds_close(void);
int fds_loadside(int side);
int fds_loader_copy(void);
void fds_init(void);
void fds_tick(void);
	
#endif
