/*
 * stm32f407xx_uart_driver.h
 *
 *  Created on: Jan 2, 2020
 *      Author: erenc
 */

#ifndef INC_STM32F407XX_UART_DRIVER_H_
#define INC_STM32F407XX_UART_DRIVER_H_

#include "stm32f407xx.h"


typedef struct
{
	uint8_t USART_Mode;				// <Possible Values From @USART_DeviceMode		*>
	uint32_t USART_Baud;			// <Possible Values From @USART_Baud			*>
	uint8_t USART_NoOfStopBits;		// <Possible Values From @USART_NoOfStopBits	*>
	uint8_t USART_WordLength;		// <Possible Values From @USART_WordLength		*>
	uint8_t USART_ParityControl;	// <Possible Values From @USART_ParityControl	*>
	uint8_t USART_HWFlowControl;	// <Possible Values From @USART_HWFlowControl	*>
}USART_Config_t;

typedef struct
{
	USART_TypeDef *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/*
 * @USART_NoOfStopBits
 * Possible number of stop bits
 */

#define USART_NO_STOP_BIT_1		0
#define USART_NO_STOP_BIT_0_5	1
#define USART_NO_STOP_BIT_2		2
#define USART_NO_STOP_BIT_1_5	3

/*
 * @USART_WordLength
 * Possible word Length
 */
#define USART_WORD_LEN_8		0
#define	USART_WORD_LEN_9		1

/*
 * @USART_ParityControl
 * Possible parity control configurations
 */

#define USART_PARITY_DISABLE	0
#define USART_EVEN_PARITY		1
#define USART_ODD_PARITY		2

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * USART flags
 */

#define USART_FLAG_TXE 			( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 	1
#define USART_BUSY_IN_TX 	2
#define USART_READY 		0


#define 	USART_EVENT_TX_CMPLT	0
#define		USART_EVENT_RX_CMPLT   	1
#define		USART_EVENT_IDLE      	2
#define		USART_EVENT_CTS       	3
#define		USART_EVENT_PE        	4
#define		USART_ERR_FE     		5
#define		USART_ERR_NE    	 	6
#define		USART_ERR_ORE    		7

/*******************************************************************************************************************************
 * 										APIs supported by this driver
 * 							For more information about the APIs check the function definitions
 *******************************************************************************************************************************/

/*
 * USART Peripheral Clock Setup
 */

void UART_PCLK_Control(USART_TypeDef *pUSARTx,uint8_t EnorDi);

/*
 * USART Peripheral Init and De-init
 */
void UART_Init(USART_Handle_t *pUARTHandle);
void UART_DeInit(USART_TypeDef *pUARTx);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */

void UART_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void UART_IRQPriorityConfig(uint8_t IRQNumber ,uint8_t IRQPriority);
void UART_IRQ_Handling(USART_Handle_t *pUARTHandle);


/*
 * Other Peripheral Controls
 */

void UART_PeripheralControl(USART_TypeDef *pUSARTx,uint8_t EnorDi);
uint8_t UART_GetFlagStatus(USART_TypeDef *pUSARTx, uint32_t FlagName);
void UART_ClearFlag(USART_TypeDef *pUSARTx, uint32_t FlagName);
void USART_SetBaudRate(USART_TypeDef *pUSARTx, uint32_t BaudRate);
/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);

#endif /* INC_STM32F407XX_UART_DRIVER_H_ */
