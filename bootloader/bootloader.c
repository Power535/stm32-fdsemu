#include <stdio.h>
#include "main.h"

#define SetBits(GPIOx, GPIO_Pin)	(GPIOx)->BSRR = GPIO_Pin
#define ResetBits(GPIOx, GPIO_Pin)	(GPIOx)->BRR = GPIO_Pin

#define SRAM_Select()		ResetBits(GPIOA,GPIO_Pin_4)
#define SRAM_Deselect()		SetBits(GPIOA,GPIO_Pin_4);

#define LED_RED_ON()		SetBits(GPIOA,GPIO_Pin_1)
#define LED_RED_OFF()		ResetBits(GPIOA,GPIO_Pin_1)
#define LED_GREEN_ON()		SetBits(GPIOA,GPIO_Pin_0)
#define LED_GREEN_OFF()		ResetBits(GPIOA,GPIO_Pin_0)

#define FIRMWARE_ADDRESS	0x08001000
#define FLASH_END_ADDRESS	0x08008000
#define FLASH_PAGE_SIZE		0x400

__forceinline void spi_send8(uint8_t data)
{
uint32_t spixbase = 0x00;

spixbase = (uint32_t)SPI1; 
spixbase += 0x0C;

*(__IO uint8_t *) spixbase = data;
}

__forceinline uint8_t spi_receive8(void)
{
  uint32_t spixbase = 0x00;
  
  spixbase = (uint32_t)SPI1; 
  spixbase += 0x0C;
  
  return *(__IO uint8_t *) spixbase;
}

__forceinline FlagStatus spi_flagstatus(uint16_t SPI_I2S_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the status of the specified SPI flag */
  if ((SPI1->SR & SPI_I2S_FLAG) != (uint16_t)RESET)
  {
    /* SPI_I2S_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* SPI_I2S_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the SPI_I2S_FLAG status */
  return  bitstatus;
}

void SRAM_WriteBuf(uint8_t *buf, int len)
{
	while(len--) {
		while(spi_flagstatus(SPI_I2S_FLAG_TXE) == RESET) {}
		spi_send8(*buf);
		while(spi_flagstatus(SPI_I2S_FLAG_RXNE) == RESET) {}
		spi_receive8();
		buf++;
	}
}

__forceinline void SRAM_ReadBuf(uint8_t *buf, int len)
{
	while(len--) {
		while (spi_flagstatus(SPI_I2S_FLAG_TXE) == RESET) {}
		spi_send8(0);
		while (spi_flagstatus(SPI_I2S_FLAG_RXNE) == RESET) {}
		*buf = spi_receive8();
		buf++;
	}
}

void sram_read(int addr,uint8_t *buf,int len)
{
	uint8_t data[4];

	data[0] = 3;
	data[1] = (uint8_t)(addr >> 16);
	data[2] = (uint8_t)(addr >> 8);
	data[3] = (uint8_t)(addr >> 0);
	SRAM_Select();
	SRAM_WriteBuf(data, 4);
	SRAM_ReadBuf(buf, len);
	SRAM_Deselect();
}

void sram_write(int addr,uint8_t *buf,int len)
{
	uint8_t data[4];

	data[0] = 2;
	data[1] = (uint8_t)(addr >> 16);
	data[2] = (uint8_t)(addr >> 8);
	data[3] = (uint8_t)(addr >> 0);
	SRAM_Select();
	SRAM_WriteBuf(data, 4);
	SRAM_WriteBuf(buf, len);
	SRAM_Deselect();
}

__forceinline void inl_RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    RCC->APB2RSTR |= RCC_APB2Periph;
  }
  else
  {
    RCC->APB2RSTR &= ~RCC_APB2Periph;
  }
}

__forceinline void inl_SPI_RxFIFOThresholdConfig(SPI_TypeDef* SPIx, uint16_t SPI_RxFIFOThreshold)
{
  /* Clear FRXTH bit */
  SPIx->CR2 &= (uint16_t)~((uint16_t)SPI_CR2_FRXTH);

  /* Set new FRXTH bit value */
  SPIx->CR2 |= SPI_RxFIFOThreshold;
}

#define CR1_CLEAR_MASK       ((uint16_t)0x3040)
#define CR1_CLEAR_MASK2      ((uint16_t)0xFFFB)

__forceinline void inl_SPI_Init(SPI_InitTypeDef* SPI_InitStruct)
{
  uint16_t tmpreg = 0;

  /*---------------------------- SPIx CR1 Configuration ------------------------*/
  /* Get the SPIx CR1 value */
  tmpreg = SPI1->CR1;
  /* Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, CPOL and CPHA bits */
  tmpreg &= CR1_CLEAR_MASK;
  /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler
  master/slave mode, CPOL and CPHA */
  /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
  /* Set SSM, SSI bit according to SPI_NSS values */
  /* Set LSBFirst bit according to SPI_FirstBit value */
  /* Set BR bits according to SPI_BaudRatePrescaler value */
  /* Set CPOL bit according to SPI_CPOL value */
  /* Set CPHA bit according to SPI_CPHA value */
  tmpreg |= (uint16_t)((uint32_t)SPI_InitStruct->SPI_Direction | SPI_InitStruct->SPI_FirstBit |
                      SPI_InitStruct->SPI_CPOL | SPI_InitStruct->SPI_CPHA |
                      SPI_InitStruct->SPI_NSS | SPI_InitStruct->SPI_BaudRatePrescaler);  
  /* Write to SPIx CR1 */
  SPI1->CR1 = tmpreg;
  /*-------------------------Data Size Configuration -----------------------*/
  /* Get the SPIx CR2 value */
  tmpreg = SPI1->CR2;
  /* Clear DS[3:0] bits */
  tmpreg &=(uint16_t)~SPI_CR2_DS;
  /* Configure SPIx: Data Size */
  tmpreg |= (uint16_t)(SPI_InitStruct->SPI_DataSize);
  /* Write to SPIx CR2 */
  SPI1->CR2 = tmpreg;
  
  /*---------------------------- SPIx CRCPOLY Configuration --------------------*/
  /* Write to SPIx CRCPOLY */
  SPI1->CRCPR = SPI_InitStruct->SPI_CRCPolynomial;
  
  /*---------------------------- SPIx CR1 Configuration ------------------------*/
  /* Get the SPIx CR1 value */
  tmpreg = SPI1->CR1;
  /* Clear MSTR bit */
  tmpreg &= CR1_CLEAR_MASK2;
  /* Configure SPIx: master/slave mode */  
  /* Set MSTR bit according to SPI_Mode */
  tmpreg |= (uint16_t)((uint32_t)SPI_InitStruct->SPI_Mode);  
  /* Write to SPIx CR1 */
  SPI1->CR1 = tmpreg;  
  
  /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
  SPI1->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
}

__forceinline void spi_init2()
{
  uint16_t tmpreg = 0;

  /*---------------------------- SPIx CR1 Configuration ------------------------*/
  /* Get the SPIx CR1 value */
  tmpreg = SPI1->CR1;
  /* Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, CPOL and CPHA bits */
  tmpreg &= CR1_CLEAR_MASK;
  /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler
  master/slave mode, CPOL and CPHA */
  /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
  /* Set SSM, SSI bit according to SPI_NSS values */
  /* Set LSBFirst bit according to SPI_FirstBit value */
  /* Set BR bits according to SPI_BaudRatePrescaler value */
  /* Set CPOL bit according to SPI_CPOL value */
  /* Set CPHA bit according to SPI_CPHA value */
  tmpreg |= (uint16_t)((uint32_t)SPI_Direction_2Lines_FullDuplex | SPI_FirstBit_MSB |
                      SPI_CPOL_Low | SPI_CPHA_1Edge |
                      SPI_NSS_Soft | SPI_BaudRatePrescaler_2);  

/* Write to SPIx CR1 */
  SPI1->CR1 = tmpreg;
  /*-------------------------Data Size Configuration -----------------------*/
  /* Get the SPIx CR2 value */
  tmpreg = SPI1->CR2;
  /* Clear DS[3:0] bits */
  tmpreg &=(uint16_t)~SPI_CR2_DS;
  /* Configure SPIx: Data Size */
  tmpreg |= (uint16_t)(SPI_DataSize_8b);
  /* Write to SPIx CR2 */
  SPI1->CR2 = tmpreg;
  
  /*---------------------------- SPIx CRCPOLY Configuration --------------------*/
  /* Write to SPIx CRCPOLY */
  SPI1->CRCPR = 7;
  
  /*---------------------------- SPIx CR1 Configuration ------------------------*/
  /* Get the SPIx CR1 value */
  tmpreg = SPI1->CR1;
  /* Clear MSTR bit */
  tmpreg &= CR1_CLEAR_MASK2;
  /* Configure SPIx: master/slave mode */  
  /* Set MSTR bit according to SPI_Mode */
  tmpreg |= (uint16_t)((uint32_t)SPI_Mode_Master);  
  /* Write to SPIx CR1 */
  SPI1->CR1 = tmpreg;  
  
  /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
  SPI1->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
}














__forceinline void SPI_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
//	SPI_InitTypeDef   SPI_InitStructure;

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

	//sram cs, red/green led pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect PXx to SD_SPI_SCK */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);

	/* Connect PXx to SD_SPI_MISO */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0); 

	/* Connect PXx to SD_SPI_MOSI */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);  

	/* SPI configuration -------------------------------------------------------*/
//	SPI_I2S_DeInit(SPI1);
    /* Enable SPI1 reset state */
    inl_RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);
    /* Release SPI1 from reset state */
    inl_RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);

/*	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	inl_SPI_Init(&SPI_InitStructure);
*/
	spi_init2();

	inl_SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);

//	SPI_Cmd(SPI1, ENABLE); /* SD_SPI enable */
    SPI1->CR1 |= SPI_CR1_SPE;

	LED_RED_ON();

	SRAM_Select();
	SRAM_Deselect();
}

void JumptoApp(void)
{
    // disable global interrupt
    __disable_irq();
 
    // Disable all peripheral interrupts
    NVIC_DisableIRQ(SysTick_IRQn);

	NVIC->ICER[0] = 0xFFFFFFFF;
/*	NVIC_DisableIRQ(WWDG_IRQn);
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_DisableIRQ(FLASH_IRQn);
	NVIC_DisableIRQ(RCC_IRQn);
	NVIC_DisableIRQ(EXTI0_1_IRQn);
	NVIC_DisableIRQ(EXTI2_3_IRQn);
	NVIC_DisableIRQ(EXTI4_15_IRQn);
	NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
	NVIC_DisableIRQ(DMA1_Channel4_5_IRQn);
	NVIC_DisableIRQ(ADC1_IRQn);
	NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
	NVIC_DisableIRQ(TIM1_CC_IRQn);
	NVIC_DisableIRQ(TIM3_IRQn);
	NVIC_DisableIRQ(TIM14_IRQn);
	NVIC_DisableIRQ(TIM15_IRQn);
	NVIC_DisableIRQ(TIM16_IRQn);
	NVIC_DisableIRQ(TIM17_IRQn);
	NVIC_DisableIRQ(I2C1_IRQn);
	NVIC_DisableIRQ(I2C2_IRQn);
	NVIC_DisableIRQ(SPI1_IRQn);
	NVIC_DisableIRQ(SPI2_IRQn);
	NVIC_DisableIRQ(USART1_IRQn);
	NVIC_DisableIRQ(USART2_IRQn);*/

    // main app start address defined in linker file
    uint32_t MemoryAddr = (uint32_t)FIRMWARE_ADDRESS;
    uint32_t *pMem = (uint32_t *)MemoryAddr;
 
    // First address is the stack pointer initial value
    __set_MSP(*pMem); // Set stack pointer
 
    // Now get main app entry point address
    pMem++;
    void (*pMainApp)(void) = (void (*)(void))(*pMem);
 
    pMainApp();
}

uint32_t firmware_calc_crc32(void)
{
	uint32_t i, data;

	//configure crc32 unit
	CRC->DR = 0xFFFFFFFF;
	CRC->POL = 0x04C11DB7;
	CRC->IDR = 0x00;
	CRC->INIT = 0xFFFFFFFF;
	CRC->CR = CRC_CR_RESET | CRC_PolSize_32 | CRC_CR_REV_IN | CRC_CR_REV_OUT;

	//calculate the crc
	for(i=0;i<0x7000;i+=4) {
		sram_read(i,(uint8_t*)&data,4);
		CRC->DR = data;
	}
  
	return(CRC->DR ^ 0xFFFFFFFF);
}

__forceinline FLASH_Status inl_FLASH_ErasePage(uint32_t Page_Address)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Page_Address));
 
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  { 
    /* If the previous operation is completed, proceed to erase the page */
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = Page_Address;
    FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    /* Disable the PER Bit */
    FLASH->CR &= ~FLASH_CR_PER;
  }
    
  /* Return the Erase Status */
  return status;
}

__forceinline FLASH_Status inl_FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
  __IO uint32_t tmp = 0;

  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* If the previous operation is completed, proceed to program the new first 
    half word */
    FLASH->CR |= FLASH_CR_PG;
  
    *(__IO uint16_t*)Address = (uint16_t)Data;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
 
    if(status == FLASH_COMPLETE)
    {
      /* If the previous operation is completed, proceed to program the new second 
      half word */
      tmp = Address + 2;

      *(__IO uint16_t*) tmp = Data >> 16;
    
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
        
      /* Disable the PG Bit */
      FLASH->CR &= ~FLASH_CR_PG;
    }
    else
    {
      /* Disable the PG Bit */
      FLASH->CR &= ~FLASH_CR_PG;
    }
  }
   
  /* Return the Program Status */
  return status;
}

void update_firmware(void)
{
	uint32_t i,data;

//	FLASH_Unlock();
  if((FLASH->CR & FLASH_CR_LOCK) != RESET)
  {
    /* Unlocking the program memory access */
    FLASH->KEYR = FLASH_FKEY1;
    FLASH->KEYR = FLASH_FKEY2;
  }
 
	//erase pages
	for(i=FIRMWARE_ADDRESS;i<FLASH_END_ADDRESS;i+=FLASH_PAGE_SIZE) {
		inl_FLASH_ErasePage(i);
	}
	
	//program pages
	for(i=FIRMWARE_ADDRESS;i<FLASH_END_ADDRESS;i+=4) {
		sram_read(i - FIRMWARE_ADDRESS,(uint8_t*)&data,4);
		inl_FLASH_ProgramWord(i,data);
		
		//could be more safe here, read the data back and make sure it matches...if not, try to reprogram it
	}

	//write firmware update success magic number to sram location
	data = 0xDEADBEEF;
	sram_write(0x20000 - 8,(uint8_t*)&data,4);
	
//	FLASH_Lock();
	FLASH->CR |= FLASH_CR_LOCK;
}

void bootloader(void)
{
	uint32_t temp32, crc32;
	uint8_t data[2];
	int i;
	
	//reset clocks to defaults
//	RCC_DeInit();
	RCC->CR |= (uint32_t)0x00000001;
	RCC->CFGR &= (uint32_t)0x08FFB80C;
	RCC->CR &= (uint32_t)0xFEF6FFFF;
	RCC->CR &= (uint32_t)0xFFFBFFFF;
	RCC->CFGR &= (uint32_t)0xFFC0FFFF;
	RCC->CFGR2 &= (uint32_t)0xFFFFFFF0;
	RCC->CFGR3 &= (uint32_t)0xFFF0FEAC;
	RCC->CR2 &= (uint32_t)0xFFFFFFFE;
	RCC->CIR = 0x00000000;

	//update system clock
	SystemCoreClockUpdate();	

	//enable necessary peripherials
    RCC->AHBENR |= RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF | RCC_AHBPeriph_CRC;
    RCC->APB2ENR |= RCC_APB2Periph_SPI1;

	//initialize spi
	SPI_Config();

	//put sram into sequencial mode
	SRAM_Select();
	data[0] = 1;
	data[1] = 0x40;
	SRAM_WriteBuf(data,2);
	SRAM_Deselect();

	//read the magic number
	sram_read(0xF008,(uint8_t*)&temp32,4);
	if(temp32 == 0xDEADBEEF) {
	
		//calculate the crc32 of the firmware stored in sram
		crc32 = firmware_calc_crc32();
		
		//read expected crc32
		sram_read(0xF004,(uint8_t*)&temp32,4);
		
		//ensure crc32 matches
		if(crc32 == temp32) {
			LED_GREEN_ON();
			LED_RED_ON();
			update_firmware();
		}
	}
	
	//clear sram in case there was something found, but the data was bad
	temp32 = 0;
	for(i=0;i<0x10000;i+=4) {
		sram_write(i,(uint8_t*)&temp32,4);
	}
}
