#include "stm32f0xx.h"
#include "spiutil.h"

void SRAM_WriteByte(uint8_t Data)
{
  /* Wait until the transmit buffer is empty */
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
  {}
  
  /* Send the byte */
  SPI_SendData8(SPI1, Data);
  
  /* Wait to receive a byte*/
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
  {}
  
  /* Return the byte read from the SPI bus */ 
  SPI_ReceiveData8(SPI1);
}

uint8_t SRAM_ReadByte(void)
{
  /* Wait until the transmit buffer is empty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
  {}
  /* Send the byte */
  SPI_SendData8(SPI1, 0);

  /* Wait until a data is received */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
  {}

  /* Return the shifted data */
  return SPI_ReceiveData8(SPI1);
}

void SRAM_Write(uint8_t *buf, int len)
{
	while(len--) {
		SRAM_WriteByte(*buf);
		buf++;
	}
}

void SRAM_Read(uint8_t *buf, int len)
{
	while(len--) {
		*buf = SRAM_ReadByte();
		buf++;
	}
}

uint8_t SD_ReadByte(void)
{
	int i;
	uint8_t data = 0;

	for(i=0;i<8;i++) {

		//clk high
		GPIO_SetBits(GPIOB,GPIO_Pin_1);

		data <<= 1;
		if(GPIO_ReadInputData(GPIOA) & GPIO_Pin_8) {
			data |= 1;
		}

		//clk low
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	}
	return(data);
}

void SD_WriteByte(uint8_t data)
{
	int i;

	for(i=0;i<8;i++) {

		//put data bit on mosi
		if(data & 0x80) {
			GPIO_SetBits(GPIOA,GPIO_Pin_9);
		}
		else {
			GPIO_ResetBits(GPIOA,GPIO_Pin_9);
		}

		//clk high
		GPIO_SetBits(GPIOB,GPIO_Pin_1);

		//shift data for next bit
		data <<= 1;

		//clk low
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	}
}

uint8_t SD_ReadWriteByte(uint8_t data)
{
	int i;
	uint8_t indata = 0;

	for(i=0;i<8;i++) {

		//put data bit on mosi
		if(data & 0x80) {
			GPIO_SetBits(GPIOA,GPIO_Pin_9);
		}
		else {
			GPIO_ResetBits(GPIOA,GPIO_Pin_9);
		}

		//clk high
		GPIO_SetBits(GPIOB,GPIO_Pin_1);

		//read input data
		indata <<= 1;
		if(GPIO_ReadInputData(GPIOA) & GPIO_Pin_8) {
			indata |= 1;
		}

		//shift data for next bit
		data <<= 1;

		//clk low
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	}
	return(indata);
}

void SD_Write(uint8_t *buf, int len)
{
	while(len--) {
		SD_WriteByte(*buf);
		buf++;
	}
}

void SD_Read(uint8_t *buf, int len)
{
	while(len--) {
		*buf = SD_ReadByte();
		buf++;
	}
}
