/**********************************************************************************************************************/
//					File name	:	Peripheral_Config.h
/**********************************************************************************************************************/
#include "stm32f4xx.h"
#include "Peripheral_Config.h"

/**********************************************************************************************************************/
void Peripheral_MCU_Config(void)
/**********************************************************************************************************************/
{
	RCC_Configuration();
	GPIO_Configuration();
}

/**********************************************************************************************************************/
void RCC_Configuration(void)
/**********************************************************************************************************************/
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
}

/**********************************************************************************************************************/
void GPIO_Configuration(void)
/**********************************************************************************************************************/
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}


/**********************************************************************************************************************/
//													End Files
/**********************************************************************************************************************/

