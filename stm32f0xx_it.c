/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    05-December-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f0xx_it.h"
#include "xprintf.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void HardFault_HandlerC(uint32_t stack[], uint32_t lr)
{
	printf("[hardfault]\n");
	printf("R0  = %08X\n",stack[0]);
	printf("R1  = %08X\n",stack[1]);
	printf("R2  = %08X\n",stack[2]);
	printf("R3  = %08X\n",stack[3]);
	printf("R12 = %08X\n",stack[4]);
	printf("Stacked LR  = %08X\n",stack[5]);
	printf("Stacked PC  = %08X\n",stack[6]);
	printf("Stacked PSR = %08X\n",stack[7]);
	printf("Current LR  = %08X\n",lr);
	
	for(;;);
}

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
 * @brief       Hard fault handler
 * @param[in]   stack pointer points to the dumped registers in SRAM
 * @return      None
 */
__asm void HardFault_Handler(void)
{
	MOVS	R0, #4
	MOV		R1, LR
	TST		R0, R1
	BEQ		used_msp
	MRS		R0, PSP
	B		get_lr_and_branch
used_msp
	MRS		R0, MSP
get_lr_and_branch
	MOV		R1, LR
	LDR		R2,=__cpp(HardFault_HandlerC)
	BX		R2
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void disk_timerproc (void);
volatile int SysTickCounter = 0;
volatile int SramSaveCounter = 0;
//volatile int TimeCounter = 0;
void SysTick_Handler(void)
{
	disk_timerproc();
//	TimeCounter++;
	if(SramSaveCounter) {
		SramSaveCounter--;
	}
	if(SysTickCounter) {
		SysTickCounter--;
	}
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
