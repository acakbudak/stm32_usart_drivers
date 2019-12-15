/*
 * stm32f44re_usart_driver.h
 *
 *  Created on: 11 Ara 2019
 *      Author: acakbudak
 */

#ifndef INC_STM32F446RE_USART_DRIVER_H_
#define INC_STM32F446RE_USART_DRIVER_H_

#include "stm32f446re.h"

void gpio_uart_config(GPIO_RegDef_t *pGPIO);
void usartTX_init(USART_RegDef_t *pUSART,uint32_t enordi);
void usartTX_config(USART_RegDef_t *pUSART,uint8_t wordlenght, uint32_t stopbits, uint32_t paritycontrol);
void usart_send(USART_RegDef_t *pUSART, uint8_t *pTxBuffer, uint32_t Len);
void usart_recieve(USART_RegDef_t *pUSART, uint8_t *pRxBuffer, uint32_t Len);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t  RCC_GetPLLOutputClock();

#endif /* INC_STM32F446RE_USART_DRIVER_H_ */
