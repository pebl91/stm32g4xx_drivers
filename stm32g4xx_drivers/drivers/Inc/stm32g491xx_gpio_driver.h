/*
 * stm32g491xx_gpio_driver.h
 *
 *  Created on: Nov 17, 2024
 *      Author: pebl91
 */

#ifndef INC_STM32G491XX_GPIO_DRIVER_H_
#define INC_STM32G491XX_GPIO_DRIVER_H_

#include "stm32g4xx.h"

/*
 * This is a configuration strucuture for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;			/*!< possible values from @GPIO_PIN_NUMBER >*/
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpedd;			/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;	/*!< possible values from @GPIO_PIN_PUPD >*/
	uint8_t GPIO_PinOPType;			/*!< possible values from @GPIO_PIN_OP >*/
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

/*
 * This is a handle strucuture for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;				/*!< This hold base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*!< This holds GPIO pin configuration settings >*/
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBER
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 * @GPIO_PIN_MODES
 * Possible modes for GPIO pin
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_OP
 * Possible output for GPIO pin
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_SPEED
 * Possible output speed for GPIO pin
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/************************************************************************************************
 * 								APIs supported by this driver
 * 					For more information about the APIs check the function definitions
 ************************************************************************************************/

/*
 * Peripherial clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and IRS handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32G491XX_GPIO_DRIVER_H_ */
