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
#include "main.h"

/** @addtogroup STM32F0xx_StdPeriph_Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void JumptoApp(void);
void bootloader(void);

#define BOOTLOADER_START_ADDR 0x08000000

void Remap_Table(void)
{
	// Copy interrupt vector table to the RAM.
	volatile uint32_t *VectorTable = (volatile uint32_t *)0x20000000;
	uint32_t ui32_VectorIndex = 0;
	__IO uint32_t tmpreg; 

	for(ui32_VectorIndex = 0; ui32_VectorIndex < 48; ui32_VectorIndex++) {
		VectorTable[ui32_VectorIndex] = *(__IO uint32_t*)((uint32_t)BOOTLOADER_START_ADDR + (ui32_VectorIndex << 2));
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
	Remap_Table();
	__enable_irq();
	bootloader();
	JumptoApp();
	while (1) {}
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
