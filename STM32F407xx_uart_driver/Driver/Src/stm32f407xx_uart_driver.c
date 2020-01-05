/*
 * stm32f407xx_uart_driver.c
 *
 *  Created on: Jan 2, 2020
 *      Author: erenc
 */


#include "stm32f407xx_uart_driver.h"


/*
 * UART Peripheral Clock Setup
 */

/*************************************************************************************
 * @fn				- UART_PCLK_Control
 *
 * @brief			- This function enables or disables peripheral clock for the given UART
 *
 * @param[in]		- Base address of the UART peripheral
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void UART_PCLK_Control(USART_TypeDef *pUSARTx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
}

/*
 * UART Peripheral Init and De-init
 */

/*************************************************************************************
 * @fn				- UART_DeInit
 *
 * @brief			- De-initializes the UARTx peripheral registers to their default reset values.
 *
 * @param[in]		- Base address of the UARTx
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void UART_Init(USART_Handle_t *pUARTHandle)
{
	UART_PCLK_Control(pUARTHandle->pUSARTx, ENABLE);
	uint32_t temp = 0;

	//UART mode configuration
	if(pUARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		temp |= (0x1 << USART_CR1_RE);
		temp &= ~(0x1 << USART_CR1_TE);
	}
	else if(pUARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		temp |= (0x1 << USART_CR1_TE);
		temp &= ~(0x1 << USART_CR1_RE);
	}
	else if(pUARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		temp |= (0x1 << USART_CR1_RE);
		temp |= (0x1 << USART_CR1_TE);
	}

	//UART parity control configuration
	if(pUARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
	{
		temp &= ~(0x1 << USART_CR1_PCE);
	}
	else if(pUARTHandle->USART_Config.USART_ParityControl == USART_EVEN_PARITY)
	{
		temp |= (0x1 << USART_CR1_PCE);
		temp &= ~(0x1 << USART_CR1_PS);
	}
	else if(pUARTHandle->USART_Config.USART_ParityControl == USART_ODD_PARITY)
	{
		temp |= (0x1 << USART_CR1_PCE);
		temp |= (0x1 << USART_CR1_PS);
	}

	//UART word length configuration
	if(pUARTHandle->USART_Config.USART_WordLength == USART_WORD_LEN_8)
	{
		temp &= ~(0x1 << USART_CR1_M);
	}
	else if(pUARTHandle->USART_Config.USART_WordLength == USART_WORD_LEN_9)
	{
		temp |= (0x1 << USART_CR1_M);
	}

	pUARTHandle->pUSARTx->USART_CR1 = temp;

	temp =0;
	//UART stop bit configuration
	temp |= pUARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;
	pUARTHandle->pUSARTx->USART_CR2 = temp;

	temp=0;

	//USART hardware flow control configuration
	if ( pUARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		temp |= ( 1 << USART_CR3_CTSE);
	}
	else if (pUARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		temp |= ( 1 << USART_CR3_RTSE);
	}
	else if (pUARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		temp |= ( 1 << USART_CR3_CTSE);
		temp |= ( 1 << USART_CR3_RTSE);
	}

	pUARTHandle->pUSARTx->USART_CR3 = temp;

	USART_SetBaudRate(pUARTHandle->pUSARTx, pUARTHandle->USART_Config.USART_Baud);

}

/*************************************************************************************
 * @fn				- UART_DeInit
 *
 * @brief			- De-initializes the UARTx peripheral registers to their default reset values.
 *
 * @param[in]		- Base address of the UARTx
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void UART_DeInit(USART_TypeDef *pUARTx)
{
	if(pUARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if(pUARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if(pUARTx == USART2)
	{
		USART3_REG_RESET();
	}
	else if(pUARTx == USART2)
	{
		UART4_REG_RESET();
	}
	else if(pUARTx == USART2)
	{
		UART5_REG_RESET();
	}
	else if(pUARTx == USART2)
	{
		USART6_REG_RESET();
	}
}


/*
 * UART Peripheral Data Send
 */

/*************************************************************************************
 * @fn				- UART_SendData
 *
 * @brief			- This function send data
 *
 * @param[in]		- UART Handle structure pointer
 * @param[in]		- Data pointer which will be send
 * @param[in]		- Length of the data will be send
 *
 * @return			- None
 *
 * @Note			- This is Blocking Call
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;
	   //Loop over until "Len" number of bytes are transferred
		for(uint32_t i = 0 ; i < Len; i++)
		{
			while(! UART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

	         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LEN_9)
			{
				pdata = (uint16_t*) pTxBuffer;
				pUSARTHandle->pUSARTx->USAERT_DR = (*pdata & (uint16_t)0x01FF);

				//check for USART_ParityControl
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used in this transfer. so, 9bits of user data will be sent
					pTxBuffer++;
					pTxBuffer++;
				}
				else
				{
					//Parity bit is used in this transfer . so , 8bits of user data will be sent
					//The 9th bit will be replaced by parity bit by the hardware
					pTxBuffer++;
				}
			}
			else
			{
				pUSARTHandle->pUSARTx->USAERT_DR = (*pTxBuffer  & (uint8_t)0xFF);

				pTxBuffer++;
			}
		}
		//Wait until TC flag is set in SR
		while( ! UART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}
/*************************************************************************************
 * @fn				- USART_SendDataIT
 *
 * @brief			- This function send data
 *
 * @param[in]		- UART Handle structure pointer
 * @param[in]		- Data pointer which will be send
 * @param[in]		- Length of the data will be send
 *
 * @return			- State
 *
 * @Note			- This is Blocking Call
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Enable the TXE interrupt
		pUSARTHandle->pUSARTx->USART_CR1 |= (0x1 << USART_CR1_TXEIE);

		//Enable the TC interrupt
		pUSARTHandle->pUSARTx->USART_CR1 |= (0x1 << USART_CR1_TCIE);
	}

	return txstate;

}

/*************************************************************************************
 * @fn				- USART_ReceiveData
 *
 * @brief			- This function receive data
 *
 * @param[in]		- UART Handle structure pointer
 * @param[in]		- Data pointer which will be receive
 * @param[in]		- Length of the data will be receive
 *
 * @return			- None
 *
 * @Note			- This is Blocking Call
 */
void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		while(! UART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE))

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LEN_9)
		{
			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//read only first 9 bits.
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->USAERT_DR  & (uint16_t)0x01FF);

				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->USAERT_DR  & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else
		{
			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be user data

				//read 8 bits from DR
				 *pRxBuffer =(uint8_t) (pUSARTHandle->pUSARTx->USAERT_DR  & (uint8_t)0xFF);
			}
			else
			{
				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USAERT_DR & (uint8_t)0x7F);
			}
			pRxBuffer++;
		}
	}
}

/*************************************************************************************
 * @fn				- USART_ReceiveDataIT
 *
 * @brief			- This function send data
 *
 * @param[in]		- UART Handle structure pointer
 * @param[in]		- Data pointer which will be receive
 * @param[in]		- Length of the data will be receive
 *
 * @return			- State
 *
 * @Note			- This is Blocking Call
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->USAERT_DR;

		pUSARTHandle->pUSARTx->USART_CR1 |= (0x1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}

/*
 * IRQ Configuration and ISR handling
 */

/*************************************************************************************
 * @fn				- UART_IRQITConfig
 *
 * @brief			- This function used for enable or disable interrupt for given IRQ Number
 *
 * @param[in]		- IRQ Number
 * @param[in]		- Enable or Disable
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void UART_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber>31 && IRQNumber <= 63)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber>63 && IRQNumber <96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber>31 && IRQNumber <= 63)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber>63 && IRQNumber <96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*************************************************************************************
 * @fn				- UART_IRQPriorityConfig
 *
 * @brief			- This function used for set interrupt priority for given IRQ Number
 *
 * @param[in]		- IRQ Number
 * @param[in]		- IRQ Priority wanted to set
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void UART_IRQPriorityConfig(uint8_t IRQNumber ,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR0_BASE + iprx) = (IRQPriority << shift_amount);
}

/*************************************************************************************
 * @fn				- UART_IRQ_Handling
 *
 * @brief			- This function check the which event triggered interrupt
 *
 * @param[in]		- UART Handle structure pointer
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void UART_IRQ_Handling(USART_Handle_t *pUARTHandle)
{
	uint32_t temp1, temp2,temp3;

	uint16_t *pData;
	temp1=0;
	temp2=0;
	temp3=0;

	/*
	 * Transmission Complete Event Handler
	 */

	//Chcek for TC Flag
	temp1 = (pUARTHandle->pUSARTx->USART_SR & (0x1 << USART_SR_TC));

	//TCIE Enable Bit Control
	temp2 = (pUARTHandle->pUSARTx->USART_CR1 & (0x1 << USART_CR1_TCIE));

	if(temp1 && temp2)
	{
		//Close Transmission
		if(pUARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			if(pUARTHandle->TxLen == 0)
			{
				//Clear TC Flag
				pUARTHandle->pUSARTx->USART_SR &= ~(0x1 << USART_SR_TC);

				//Reset the App State
				pUARTHandle->TxBusyState = USART_READY;

				//Rest the Buffer to Null
				pUARTHandle->pTxBuffer = NULL;

				//Reset the Length
				pUARTHandle->TxLen =0;

				//Call App Callback
				USART_ApplicationEventCallback(pUARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}


	temp1=0;
	temp2=0;
	temp3=0;
	/*
	 * Transmit Data Register Empty Event Handler
	 */

	//Check for TXE Flag
	temp1 = (pUARTHandle->pUSARTx->USART_SR & (0x1 << USART_SR_TXE));

	//TXEIE Enable Bit Control
	temp2 = (pUARTHandle->pUSARTx->USART_CR1 & (0x1 << USART_CR1_TXEIE));

	if(temp1 && temp2)
	{
		if(pUARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Send data until TxLen Equal to 0
			if(pUARTHandle->TxLen > 0)
			{
				//Check for USART_WordLength 9 Bits or 8 Bits
				if(pUARTHandle->USART_Config.USART_WordLength == USART_WORD_LEN_9)
				{
					pData = (uint16_t *)pUARTHandle->pTxBuffer;

					pUARTHandle->pUSARTx->USAERT_DR = (*pData & (uint16_t)0x01FF);

					//Check for parity
					if(pUARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//9 Bit Data Transfered
						pUARTHandle->TxLen -=2;
						pUARTHandle->pTxBuffer++;
						pUARTHandle->pTxBuffer++;
					}
					else
					{
						//8 bit Data Transfered
						pUARTHandle->pTxBuffer++;
						pUARTHandle->TxLen --;
					}
				}
				else if(pUARTHandle->USART_Config.USART_WordLength == USART_WORD_LEN_8)
				{
					pUARTHandle->pUSARTx->USAERT_DR = (*pUARTHandle->pTxBuffer & (uint8_t)0XFF);

					pUARTHandle->pTxBuffer++;
					pUARTHandle->TxLen--;

				}
			}
			if(pUARTHandle->TxLen == 0)
			{
				pUARTHandle->pUSARTx->USART_CR1 &= ~(0x1 << USART_CR1_TXEIE);
			}
		}
	}

	temp1=0;
	temp2=0;
	temp3=0;
	/*
	 * Received Data Ready to be Read Event Handler
	 */

	//Check For RXNE Flag
	temp1 = (pUARTHandle->pUSARTx->USART_SR & (0X1 << USART_SR_RXNE));

	//RXENIE Enable Bit Control
	temp2 = (pUARTHandle->pUSARTx->USART_CR1 & (0x1 << USART_CR1_RXNEIE));

	if(temp1 && temp2)
	{
		if(pUARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUARTHandle->RxLen > 0)
			{
				if(pUARTHandle->USART_Config.USART_WordLength == USART_WORD_LEN_9)
				{
					if(pUARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//9 Bit User Data Received
						*((uint16_t *)pUARTHandle->pRxBuffer) = (pUARTHandle->pUSARTx->USAERT_DR & (uint16_t)0x01FF);

						pUARTHandle->pRxBuffer +=2;
						pUARTHandle->RxLen-=2;
					}
					else
					{
						//8 Bit User Data Received
						*(pUARTHandle->pRxBuffer) = (pUARTHandle->pUSARTx->USAERT_DR & (uint8_t)0xFF);

						pUARTHandle->pRxBuffer++;
						pUARTHandle->RxLen--;
					}
				}
				else if(pUARTHandle->USART_Config.USART_WordLength == USART_WORD_LEN_8)
				{
					if(pUARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//8 Bit User Data Received
						*(pUARTHandle->pRxBuffer) = (pUARTHandle->pUSARTx->USAERT_DR & (uint8_t)0xFF);

						pUARTHandle->RxLen--;
						pUARTHandle->pRxBuffer++;
					}
					else
					{
						*(pUARTHandle->pRxBuffer) = (pUARTHandle->pUSARTx->USAERT_DR & (uint8_t)0x7F);

						pUARTHandle->pRxBuffer++;
						pUARTHandle->RxLen--;
					}
				}
			}
			if(pUARTHandle->RxLen ==0)
			{
				pUARTHandle->pUSARTx->USART_CR1 &= ~(0x1 << USART_CR1_RXNEIE);
				pUARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	temp1=0;
	temp2=0;
	temp3=0;
	/*
	 * CTS flag Event Hanlder
	 */

	//Check for CTS Flag
	temp1 = (pUARTHandle->pUSARTx->USART_SR & (0x1 << USART_SR_CTS));

	//CTSIE Enable Bit Control
	temp2 = (pUARTHandle->pUSARTx->USART_CR3 & (0x1 << USART_CR3_CTSIE));

	//CTSE Enable Bit Control
	temp3 = (pUARTHandle->pUSARTx->USART_CR3 & (0x1 << USART_CR3_CTSE));

	if(temp1 && temp2 && temp3)
	{
		pUARTHandle->pUSARTx->USART_SR &= ~(0x1 << USART_SR_CTS);

		USART_ApplicationEventCallback(pUARTHandle, USART_EVENT_CTS);
	}

	temp1=0;
	temp2=0;
	temp3=0;
	/*
	 * Idle Line Detected Event Handler
	 */

	//Check for IDLE Flag
	temp1 = (pUARTHandle->pUSARTx->USART_SR & (0x1 << USART_SR_IDLE));

	//IDLEIE Enable Bit Control
	temp2 = (pUARTHandle->pUSARTx->USART_CR1 & (0x1 << USART_CR1_IDLEIE));

	if(temp1 && temp2)
	{
		pUARTHandle->pUSARTx->USART_SR &= ~(0x1 << USART_SR_IDLE);

		USART_ApplicationEventCallback(pUARTHandle, USART_EVENT_IDLE);
	}

	temp1=0;
	temp2=0;
	temp3=0;
	/*
	 * Overrun Error Detected Event Handler
	 */

	//Check for ORE Flag
	temp1 = (pUARTHandle->pUSARTx->USART_SR & (0x1 << USART_SR_ORE));

	//RXNEIE Enable Bit Control
	temp2 = (pUARTHandle->pUSARTx->USART_CR1 & (0x1 << USART_CR1_RXNEIE));

	if(temp1 && temp2)
	{
		//Do Not Need to Clear ORE

		USART_ApplicationEventCallback(pUARTHandle, USART_ERR_ORE);
	}

	/***************************************************************************************************************/

	temp1=0;
	temp2=0;
	temp3=0;
	/*
	 * Noise Flag, Overrun error and Framing Error in multibuffer communication Event Handler
	 */

	//EIE Enable Bit Control
	temp2 = (pUARTHandle->pUSARTx->USART_CR3 & (0x1 << USART_CR3_EIE));

	if(temp2)
	{
		if(temp1 & (0x1 << USART_SR_FE))
		{
			USART_ApplicationEventCallback(pUARTHandle, USART_ERR_FE);
		}
		else if(temp1 & (0x1 << USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUARTHandle, USART_ERR_ORE);
		}
		else if(temp1 & (0x1 << USART_SR_NF))
		{
			USART_ApplicationEventCallback(pUARTHandle, USART_ERR_NE);
		}
	}


}

/*************************************************************************************
 * @fn				- UART_PeripheralControl
 *
 * @brief			- This function used for Enable or Disable the UART Peripheral
 *
 * @param[in]		- Base address of the USARTx
 * @param[in]		- ENABLE or DISABLE
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void UART_PeripheralControl(USART_TypeDef *pUSARTx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->USART_CR1 |= (0x1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->USART_CR1 &= ~(0x1 << USART_CR1_UE);
	}
}

/*************************************************************************************
 * @fn				- UART_GetFlagStatus
 *
 * @brief			- This function check the UART_SR
 *
 * @param[in]		- Base address of the UARTx
 * @param[in]		- Flag name which will be check
 * @param[in]		-
 *
 * @return			- This function returns the flag status
 *
 * @Note			- None
 */

uint8_t UART_GetFlagStatus(USART_TypeDef *pUSARTx, uint32_t FlagName)
{
	if(pUSARTx->USART_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*************************************************************************************
 * @fn				- UART_ClearFlag
 *
 * @brief			- This function clear given USART_SR bit
 *
 * @param[in]		- Base address of the UARTx
 * @param[in]		- Flag name which will be check
 * @param[in]		-
 *
 * @return			- None
 *
 * @Note			- None
 */
void UART_ClearFlag(USART_TypeDef *pUSARTx, uint32_t FlagName)
{
	pUSARTx->USART_SR &= ~(0x1 << FlagName);
}


void USART_SetBaudRate(USART_TypeDef *pUSARTx, uint32_t BaudRate)
{
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->USART_CR1 & (1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	}
	else
	{
		//over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->USART_CR1 & ( 1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

	}
	else
	{
		//over sampling by 16
		F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->USART_BRR = tempreg;
}
