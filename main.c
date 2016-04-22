/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    05-December-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "main.h"
#include "spiutil.h"
#include "sram.h"
#include "sdcard.h"
#include "fds.h"
#include "fifo.h"
#include "build.h"
#include "fatfs/src/ff.h"
#include "xprintf.h"

extern volatile int SysTickCounter;

void DelayMS(int ms);

typedef struct ident_s {
	uint32_t ident[2];
	uint16_t build;
} ident_t;

ident_t ident = {
	{0xDEADBEEF, 0xCAFEBEEF},
	BUILDNUM
};

/*void hexdump(char *desc, void *addr, int len)
{
	int i;
	unsigned char buff[17];
	unsigned char *pc = (unsigned char *)addr;

	// Output description if given.
	if (desc != NULL)
	printf("%s:\r\n", desc);

	// Process every byte in the data.
	for (i = 0; i < len; i++) {
		// Multiple of 16 means new line (with line offset).

		if ((i % 16) == 0) {
			// Just don't print ASCII for the zeroth line.
			if (i != 0)
			printf("  %s\r\n", buff);

			// Output the offset.
			printf("  %04x ", i);
		}
		// Now the hex code for the specific character.
		printf(" %02x", pc[i]);

		// And store a printable ASCII character for later.
		if ((pc[i] < 0x20) || (pc[i] > 0x7e))
		buff[i % 16] = '.';
		else
		buff[i % 16] = pc[i];
		buff[(i % 16) + 1] = '\0';
	}

	// Pad out last line if not exactly 16 characters.
	while ((i % 16) != 0) {
		printf("   ");
		i++;
	}

	// And print the final ASCII bit.
	printf("  %s\r\n", buff);
}*/

static void CLOCK_Config(void)
{
	RCC_DeInit();

	//configure pll (hsi * 12)
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_12);
	RCC_PLLCmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

	//set sysclk and hclk to pll freq
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	RCC_HCLKConfig(RCC_SYSCLK_Div1);

	//set pclk to hclk / 2
	RCC_PCLKConfig(RCC_HCLK_Div2);

	//update system clock
	SystemCoreClockUpdate();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF | RCC_AHBPeriph_CRC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_USART1 | RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
}

static void SPI_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef   SPI_InitStructure;

  /* Configure SD_SPI pins: SCK, MISO, MOSI */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure pin: SRAM CS pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure pin: SD Card CS pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//spi bit-banged sram
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

//sclk
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

//mosi
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

//miso
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect PXx to SD_SPI_SCK */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);

  /* Connect PXx to SD_SPI_MISO */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0); 

  /* Connect PXx to SD_SPI_MOSI */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);  
  
  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPI1);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;

  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  
  SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);

  SPI_Cmd(SPI1, ENABLE); /* SD_SPI enable */
  
  //clk idles low
  GPIO_ResetBits(GPIOB,GPIO_Pin_1);

	SRAM_Select();
	SD_Deselect();
	SRAM_Deselect();
}

static void USART_Config(void)
{ 
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* USARTx configured as follow:
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - Stop Bit = 1 Stop Bit
  - Parity = No Parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
  
  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  /* Configure USART Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART1, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(USART1, ENABLE);
}

void FDS_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//-ready, -mediaset, readdata, -writable
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 |  GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//motoron, red/green led
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//-stopmotor
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//-scanmedia, -write
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//disk flip button
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	//writedata
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);

	GPIO_ResetBits(GPIOF,GPIO_Pin_1);

	CLEAR_MEDIASET();
	CLEAR_WRITABLE();

	LED_GREEN_ON();
	LED_RED_OFF();
}

void TIMER_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* readdata timer configuration */
	TIM_TimeBaseStructure.TIM_Period = 248;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM14, ENABLE);
	TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);

	/* writedata timer configuration */
	/* readdata timer configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 4;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	TIM_Cmd(TIM1, ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);

	/* Setup the TIM14 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Setup the TIM1 CC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}

int outputchar(int ch)
{
	USART_SendData(USART1, (uint8_t) ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
	if(ch == '\n') {
		USART_SendData(USART1, (uint8_t) '\r');
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
	}
	return ch;
}

FATFS FatFs;				/* File system object for each logical drive */

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

int read_char(void)
{
	int ret = -1;
	
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)) {
		ret = (int)(uint8_t)USART_ReceiveData(USART1);
	}
	return(ret);
}

void update_firmware(void);

void console_tick(void)
{
	int ch;
	
	ch = read_char();
	if(ch == -1) {
		return;
	}
	
/*	if(ch == 'u') {
		update_firmware();
	}*/
/*	switch(ch) {
		case '?':
			printf("help:\n");
			printf("  i - insert disk\n");
			printf("  r - remove disk\n");
			break;
		
		case 'i':
			printf("inserting disk\n");
			SET_MEDIASET();
			SET_WRITABLE();
			break;

		case 'r':
			printf("removing disk\n");
			CLEAR_MEDIASET();
			CLEAR_WRITABLE();
			break;

		case '0':
			printf("inserting side 0\n");
			CLEAR_MEDIASET();
			CLEAR_WRITABLE();
			fds_loadside(0);
			SET_MEDIASET();
			SET_WRITABLE();
			break;

		case '1':
			printf("inserting side 1\n");
			CLEAR_MEDIASET();
			CLEAR_WRITABLE();
			fds_loadside(1);
			SET_MEDIASET();
			SET_WRITABLE();
			break;

	}*/
}

void put_rc(FRESULT rc)
{
/*	const char *str =
		"OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
		"INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
		"INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
		"LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0";
	FRESULT i;

	for (i = (FRESULT)0; i != rc && *str; i++) {
		while (*str++) ;
	}
	printf("rc=%u FR_%s\n", (UINT)rc, str);*/
	printf("rc=%d\n",(UINT)rc);
}

void loader_copy(void);

#define FIRMWARE_START_ADDR	0x08001000

void Remap_Table(void)
{
	// Copy interrupt vector table to the RAM.
	volatile uint32_t *VectorTable = (volatile uint32_t *)0x20000000;
	uint32_t ui32_VectorIndex = 0;
	__IO uint32_t tmpreg; 

	for(ui32_VectorIndex = 0; ui32_VectorIndex < 48; ui32_VectorIndex++) {
		VectorTable[ui32_VectorIndex] = *(__IO uint32_t*)((uint32_t)FIRMWARE_START_ADDR + (ui32_VectorIndex << 2));
	}

	RCC->AHBRSTR = 0xFFFFFFFF;

    //  Enable SYSCFG peripheral clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	tmpreg = RCC->APB2ENR &  RCC_APB2ENR_SYSCFGEN;

	RCC->AHBRSTR = 0x00;

	// Remap RAM into 0x0000 0000
	SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);

}

int main(void)
{
	FRESULT fr;
	RCC_ClocksTypeDef RCC_ClockFreq;

	Remap_Table();
    __enable_irq();

	xdev_out(outputchar);

	CLOCK_Config();
	USART_Config();
	SPI_Config();
	FDS_Config();
	TIMER_Config();

	printf("\nstm32-fdsemu - build %d\n",ident.build);

	RCC_GetClocksFreq(&RCC_ClockFreq);

	if (SysTick_Config(SystemCoreClock / 1000)) { 
//		printf("SysTick config error\n");
		while (1);
	}

//	printf("--HCLK: %.6f mhz\n",(double)RCC_ClockFreq.HCLK_Frequency / 1000000.0f);
//	printf("--PCLK: %.6f mhz\n",(double)RCC_ClockFreq.PCLK_Frequency / 1000000.0f);
//	printf("--SYSCLK: %.6f mhz\n",(double)RCC_ClockFreq.SYSCLK_Frequency / 1000000.0f);

	//init the sram
	sram_init();
	fds_init();

	fr = f_mount(&FatFs, "", 1);
	put_rc(fr);
	if(fr != FR_OK) {
		for(;;) {
			DelayMS(250);
			LED_GREEN_OFF();
			LED_RED_ON();
			DelayMS(250);
			LED_GREEN_ON();
			LED_RED_OFF();
		}
	}

//	printf("end init\n");
	
	//load the loader
	loader_copy();

	SET_MEDIASET();
	SET_WRITABLE();

	while (1) {
		fds_tick();
//		console_tick();
	}
}

void DelayMS(int ms)
{
	//make sure the irq is enabled
	NVIC_EnableIRQ(SysTick_IRQn);
	
	//setup starting counter
	SysTickCounter = ms;

	//wait
	while(SysTickCounter != 0);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
