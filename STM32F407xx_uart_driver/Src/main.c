/**
  ******************************************************************************
  * @file    main.c
  * @author  Auto-generated by STM32CubeIDE
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f407xx.h"
#include "string.h"


USART_Handle_t UARTHandle;

char *msg[3] = {"Eren Cimen", "STM32FDisco", "Embedded System!!!"};

char RxBuf[1024];
uint8_t Btn_Ctrl,RxCmlt;

void Delay(uint32_t cnt)
{
	for(uint32_t i =0; i<cnt; i++);
}


void USer_Button_Init(void)
{
	/*
	 * PA0 ---> WAKE-UP Button
	 */
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOA;

	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;

	GPIO_Init(&GPIOBtn);
}

void UART_Pin_Config(void)
{
	/*
	 * PA2 (AF7) ---> USART2_TX
	 * PA3 (AF7) ---> USART2_RX
	 */
	GPIO_Handle_t UARTPin;

	UARTPin.pGPIOx = GPIOA;

	UARTPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	UARTPin.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	UARTPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	UARTPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	UARTPin.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;

	//USART2_TX Init
	UARTPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&UARTPin);

	//USART2_RX Init;
	UARTPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&UARTPin);
}

void UART2_Init(void)
{
	/*
	 * USART2
	 * Baud Rate ---------> 115200
	 * Mode --------------> RxTx
	 * Stop Bit ----------> 1
	 * Parity ------------> No Parity
	 * HW Flow Control ---> No HW Flow Control
	 * Word Length -------> 8
	 */

	UARTHandle.pUSARTx = USART2;

	UARTHandle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	UARTHandle.USART_Config.USART_Mode = USART_MODE_TXRX;
	UARTHandle.USART_Config.USART_NoOfStopBits = USART_NO_STOP_BIT_1;
	UARTHandle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	UARTHandle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	UARTHandle.USART_Config.USART_WordLength = USART_WORD_LEN_8;

	UART_Init(&UARTHandle);
}

int main(void)
{
	uint8_t cnt;
	Btn_Ctrl = RESET;
	RxCmlt = RESET;
	//Peripheral Initiation
	USer_Button_Init();
	UART_Pin_Config();
	UART2_Init();

	//GPIO Button Interrupt Enable
	GPIO_IRQITConfig(EXTI0_IRQn, ENABLE);

	//USART2 Interrupt Enable
	UART_IRQITConfig(USART2_IRQn, ENABLE);

	//USART2 Peripheral Enable
	UART_PeripheralControl(USART2, ENABLE);

	while(1)
	{
		//Wait until button is pressed
		while(! Btn_Ctrl);

		//Make sure cnt < 3
		cnt = cnt %3;

		//Reset RxBuf
		memset(RxBuf,0, sizeof(RxBuf));
		//Wait until USAR2 is ready
		while(USART_ReceiveDataIT(&UARTHandle, RxBuf,strlen(msg[cnt])) != USART_READY);

		//Send Data
		USART_SendData(&UARTHandle, (uint8_t *)msg[cnt], strlen(msg[cnt]));

		//Wait Until Receive Complete
		while(!RxCmlt);

		RxCmlt = RESET;
		Btn_Ctrl = RESET;
		cnt++;
	}
}


void EXTI0_IRQHandler(void)
{
	Delay(5000);

	GPIO_IRQ_Handling(GPIO_PIN_NO_0);

	Btn_Ctrl = SET;
}

void USART2_IRQHandler(void)
{
	UART_IRQ_Handling(&UARTHandle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{
	if(AppEv == USART_EVENT_RX_CMPLT)
	{
		RxCmlt = SET;
	}
}