/*
 * stm32f446re_uart4_tx_driver.c
 *
 *  Created on: 11 Ara 2019
 *      Author: acakbudak
 */

/*
  this driver uses PA0 for uart4_tx and all configurations are used for that pin
  this driver has an optional baudrate calculater that uses APB clocks
  
*/
#include "stm32f44re_usart_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};


void gpio_uart_config(GPIO_RegDef_t *pGPIO)
{
	//PA0=uart4_tx
	pGPIO->MODER |= (2 << 2*GPIO_PIN_NO_0);
	pGPIO->OTYPER &= ~(1 << GPIO_PIN_NO_0);
	pGPIO->OSPEEDR |= (2 << 2*GPIO_PIN_NO_0);
	pGPIO->PUPDR |= (1 << 2*GPIO_PIN_NO_0);
	pGPIO->AFR[0] |= (8 << 4*GPIO_PIN_NO_0);
	//PA1=uart_rx
	pGPIO->MODER |= (2 << 2*GPIO_PIN_NO_1);
	pGPIO->OTYPER &= ~(1 << GPIO_PIN_NO_1);
	pGPIO->OSPEEDR |= (2 << 2*GPIO_PIN_NO_1);
	pGPIO->PUPDR |= (1 << 2*GPIO_PIN_NO_1);
	pGPIO->AFR[0] |= (8 << 4*GPIO_PIN_NO_1);

}
void usartTX_init(USART_RegDef_t *pUSART,uint32_t enordi)
{
	//enabling the TX and RX engine
	if(enordi == ENABLE)
		{
			pUSART->CR1 |= (1 << 3);
			pUSART->CR1 |= (1 << 2);
		}else if(enordi == DISABLE)
				{
					pUSART->CR1 &= ~(1 << 3);
					pUSART->CR1 &= ~(1 << 2);
				}

	//to enable the USART
	pUSART->CR1 |=(1 << 13);

	//oversampling by 8
	pUSART->CR1 |= (1 << 15);

	pUSART->BRR = 0x0114;




}

void usartTX_config(USART_RegDef_t *pUSART,uint8_t wordlenght, uint32_t stopbits, uint32_t paritycontrol)
{

	//setting the word lenght of the data transmission
	if(wordlenght == 8)
	{
		pUSART->CR1 |= (1 << 12);
	}else if (wordlenght == 9)
				{
					pUSART->CR1 &= ~(1 << 12);
				}

	//configuration of the nomber of stop bits
		if(stopbits == 1)
		{
			pUSART->CR2 &= ~(1 << 12);

		}else if(stopbits == 2)
						{
							pUSART->CR2 |= (0x10 << 12);
						}

	//configuring the parity bit usage
	  if( paritycontrol == ENABLE )
		{
		  pUSART->CR1 |= (1 << 10);
		}
		else if (paritycontrol == DISABLE)
						{
							pUSART->CR1 &= ~(1 << 10);
						}


}

void usart_send(USART_RegDef_t *pUSART, uint8_t *pTxBuffer, uint32_t Len)
{

	for(uint32_t i = 0 ; i < Len; i++)
		{
			//Implement the code to wait until TXE flag is set in the SR
			while( !(pUSART->SR & 0x80) );

			//This is 8bit data transfer
			pUSART->DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
			//Implement the code to wait till TC flag is set in the SR
			while( !(pUSART->SR & 0x40) );

}


void usart_recieve(USART_RegDef_t *pUSART, uint8_t *pRxBuffer, uint32_t Len)
{
	for(uint32_t i = 0 ; i < Len; i++)
			{
				//Implement the code to wait until RXNE flag is set in the SR
				while( !(pUSART->SR & 0x10) );

				//This is 8bit data transfer
				*pRxBuffer = (uint8_t) (pUSART->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
}





/*
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
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
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << 15))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
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
  if(pUSARTx->CR1 & ( 1 << 15))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0 )
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if (clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}



	//apb1
	temp = ((RCC->CFGR >> 10 ) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 =  (SystemClk / ahbp) /apb1p;

	return pclk1;
}



/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

/*
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock=0,tmp,pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		SystemClock = 16000000;
	}else
	{
		SystemClock = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	if(tmp < 0x08)
	{
		ahbp = 1;
	}else
	{
       ahbp = AHB_PreScaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScaler[tmp-4];
	}

	pclk2 = (SystemClock / ahbp )/ apb2p;

	return pclk2;
}

uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}


*/
