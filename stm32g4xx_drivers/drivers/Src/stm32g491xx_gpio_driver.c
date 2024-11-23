/*
 * stm32g491xx_gpio_driver.c
 *
 *  Created on: Nov 17, 2024
 *      Author: pebl91
 */

#include "stm32g491xx_gpio_driver.h"

/*
 * Peripherial clock setup
 */

/****************************************************************************************************
 * @fn					- GPIO_PeriCloclkCOntrol
 *
 * @brief				- This function enabels or disables peripheral clock for given GPIO port
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- ENABLE or DISABLE macro
 *
 * @return				- none
 *
 * @Note				- none
 ****************************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */

/****************************************************************************************************
 * @fn					- GPIO_Init
 *
 * @brief				- This function gives all information needed for initialization and configuration for given GPIO pin
 *
 * @param[in]			- number of initialized pin
 *
 * @return				- none
 *
 * @Note				- none
 ****************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;	//temp. register
	//1. configure the mode of GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
		pGPIOHandle->pGPIOx->MODER &= ~(3u << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	} else
	{
		// interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear corresponding RTSR bit
			EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear corresponding FTSR bit
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			//1. configure the RTSR
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//configure the FTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASSEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. enable the exti interrupt delivery using IMR
		EXTI->IMR1 |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpedd << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3U << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting

	temp = 0;

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(3U << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; //setting

	temp = 0;
	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(2U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; //setting

	temp = 0;
	//5. configure the alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); //setting


	}
}

/****************************************************************************************************
 * @fn					- GPIO_DeInit
 *
 * @brief				- This function disables given GPIO port
 *
 * @param[in]			- base address of the GPIO peripheral
 *
 * @return				- none
 *
 * @Note				- none
 ****************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
}

/*
 * Data read and write
 */

/****************************************************************************************************
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				- This function enabels to read input data form given GPIO pin
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- number of used pin
 *
 * @return				- value of given pin
 *
 * @Note				- none
 ****************************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/****************************************************************************************************
 * @fn					- GPIO_ReadFromInputPort
 *
 * @brief				- This function enabels to read state of all pins in given GPIO port
 *
 * @param[in]			- base address of the GPIO peripheral
 *
 * @return				- value of I/O port pins
 *
 * @Note				- none
 ****************************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/****************************************************************************************************
 * @fn					- GPIO_WriteToOutputPin
 *
 * @brief				- This function enabels to set pin to LOW or HIGH state
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- number of used pin
 *
 * @return				- none
 *
 * @Note				- none
 ****************************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to pin number
		pGPIOx->ODR |= (1 << PinNumber);
	} else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/****************************************************************************************************
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				- This function enabels to write value to all the pins of GPIO port
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- value we want to send
 *
 * @return				- none
 *
 * @Note				- none
 ****************************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/****************************************************************************************************
 * @fn					- GPIO_ToggleOutputPin
 *
 * @brief				- This function toggle the given pin state (LOW - HIGH)
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- number of pin we want to toggle
 *
 * @return				- none
 *
 * @Note				- none
 ****************************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ configuration and IRS handling
 */

/****************************************************************************************************
 * @fn					- GPIO_IRQConfig
 *
 * @brief				- This function configures the interrupt settings for a specific IRQ number
 *
 * @param[in]			- number of given IRQ
 * @param[in]			- ENABLE or DISABLE
 *
 * @return				- none
 *
 * @Note				- none
 ****************************************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISERO0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISERO1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		} else if(IRQNumber >= 64  && IRQNumber < 96)
		{
			//program ISERO2 register //64 to 95
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	} else
	{
		if(IRQNumber <= 31)
		{
			//program ICERO0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICERO1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		} else if(IRQNumber >= 64  && IRQNumber < 96)
		{
			//program ICERO2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}

/****************************************************************************************************
 * @fn					- GPIO_IRQPriorityConfig
 *
 * @brief				- This function configures the interrupt settings for a specific IRQ number
 *
 * @param[in]			- number of given IRQ
 * @param[in]			- value of IRQ priority
 *
 * @return				- none
 *
 * @Note				- none
 ****************************************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. find out the ipr register
	uint8_t irpx = IRQNumber / 4;
	uint8_t irpx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * irpx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (irpx * 4)) |= (IRQPriority << shift_amount );

}
/****************************************************************************************************
 * @fn					- GPIO_IRQHandling
 *
 * @brief				- This function to handle an interrupt for a specific GPIO pin
 *
 * @param[in]			- number of given pin

 *
 * @return				- none
 *
 * @Note				- none
 ****************************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR1 |= (1 << PinNumber);
	}

}


