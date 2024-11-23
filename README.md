
# STM32G4xx drivers

API of drivers layer for STM32G4xx MCU.




## Roadmap (for now)
- Implementation of SPI
- Implementation of I2C
- Implementation of UART
## Authors

- [@pebl91](https://www.github.com/pebl91)


## Feedback

If you have any feedback, please reach out to us at pawel_smusz@yahoo.com


## Usage/Examples

```C
#include "stm32g4xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 50000; i++);
}


int main(void)
{

	GPIO_Handle_t GPIO_LED;

	GPIO_LED.pGPIOx = GPIOA;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIO_LED);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_0);
		delay();
	}

	return 0;
}
