#ifndef __spiutil_h__
#define __spiutil_h__

#include <stdint.h>
#include "stm32f0xx.h"

#define SRAM_Select()	GPIO_ResetBits(GPIOA,GPIO_Pin_4)
#define SRAM_Deselect()	GPIO_SetBits(GPIOA,GPIO_Pin_4);
#define SD_Select()		GPIO_ResetBits(GPIOB,GPIO_Pin_0);
#define SD_Deselect()	GPIO_SetBits(GPIOB,GPIO_Pin_0);

void SRAM_WriteByte(uint8_t Data);
uint8_t SRAM_ReadByte(void);
void SRAM_Write(uint8_t *buf, int len);
void SRAM_Read(uint8_t *buf, int len);

uint8_t SD_ReadByte(void);
void SD_WriteByte(uint8_t data);
uint8_t SD_ReadWriteByte(uint8_t data);
void SD_Write(uint8_t *buf, int len);
void SD_Read(uint8_t *buf, int len);

#endif
