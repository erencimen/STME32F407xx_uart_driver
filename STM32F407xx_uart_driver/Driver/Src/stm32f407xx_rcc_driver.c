/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Jan 2, 2020
 *      Author: erenc
 */


#include "stm32f407xx_rcc_driver.h"



uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_Prescaler[4] = {2,4,8,16};
uint8_t APB2_Prescaler[4] = {2,4,8,16};

/*************************************************************************************
 * @fn				- RCC_GetPCLK1Value
 *
 * @brief			- This function returns APB1 bus clock
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- APB1 bus clock
 *
 * @Note			- None
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint8_t SCLK_Source, temp, ahbp, apb1;
	uint32_t System_Clk;


	SCLK_Source = ((RCC->CFGR >> RCC_CFGR_SW) & 0x3);

	if(SCLK_Source == 0)
	{
		System_Clk	= RCC_HSI;

	}
	else if(SCLK_Source == 1)
	{
		System_Clk = RCC_HSE;

	}

	else if(SCLK_Source == 2)
	{
		System_Clk = RCC_GetPLLOutputClock();
	}

	temp = (RCC->CFGR >> RCC_CFGR_HPRE) & 0xF;
	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_Prescaler[temp - 8];
	}

	temp = (RCC->CFGR >> RCC_CFGR_PPRE1) & 0x4;
	if(temp < 4)
	{
		apb1 = 1;
	}
	else
	{
		apb1 = APB1_Prescaler[temp - 4];
	}

	System_Clk = (System_Clk/ahbp)/apb1;

	return System_Clk;
}

/*************************************************************************************
 * @fn				- RCC_GetPCLK2Value
 *
 * @brief			- This function returns APB2 bus clock
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- APB2 bus clock
 *
 * @Note			- None
 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint8_t SCLK_Source, temp, ahbp, apb2;
	uint32_t System_Clk;


	SCLK_Source = ((RCC->CFGR >> RCC_CFGR_SW) & 0x3);

	if(SCLK_Source == 0)
	{
		System_Clk	= RCC_HSI;

	}
	else if(SCLK_Source == 1)
	{
		System_Clk = RCC_HSE;

	}

	else if(SCLK_Source == 2)
	{
		System_Clk = RCC_GetPLLOutputClock();
	}

	temp = (RCC->CFGR >> RCC_CFGR_HPRE) & 0xF;
	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_Prescaler[temp - 8];
	}

	temp = (RCC->CFGR >> RCC_CFGR_PPRE2) & 0x4;
	if(temp < 4)
	{
		apb2 = 1;
	}
	else
	{
		apb2 = APB2_Prescaler[temp - 4];
	}

	System_Clk = (System_Clk/ahbp)/apb2;

	return System_Clk;
}

/*************************************************************************************
 * @fn				- RCC_GetPLLOutputClock
 *
 * @brief			- This function returns APB1 bus clock on PLL mode
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- APB1 bus clock
 *
 * @Note			- None
 */
uint32_t RCC_GetPLLOutputClock(void)
{
	uint32_t PLL_Src,System_Clk;

	if(((RCC->PLLCFGR >> RCC_PLLCFGR_PLLSRC) & 0x1))
	{
		PLL_Src = RCC_HSE;
	}
	else
	{
		PLL_Src = RCC_HSI;
	}

	System_Clk = PLL_Src / ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLM) & 0x1F);

	System_Clk = System_Clk * ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLN) & 0x1FF);

	System_Clk = System_Clk / ((RCC->PLLCFGR >> RCC_PLLCFGR_PLLP) & 0x3);

	return System_Clk;

}
