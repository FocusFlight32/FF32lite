/*

FF32lite from FocusFlight, a new alternative firmware
for the Naze32 controller

Original work Copyright (c) 2013 John Ihlein

This file is part of FF32lite.

Includes code and/or ideas from:

  1)BaseFlight
  2)S.O.H. Madgwick

FF32lite is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

FF32lite is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with FF32lite. If not, see <http://www.gnu.org/licenses/>.

*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/

///////////////////////////////////////////////////////////////////////////////

#define UART_BUFFER_SIZE 2048

// Receive buffer, circular DMA
volatile uint8_t rxBuffer[UART_BUFFER_SIZE];
uint32_t rxDMAPos = 0;

volatile uint8_t txBuffer[UART_BUFFER_SIZE];
volatile uint32_t txBufferTail = 0;
volatile uint32_t txBufferHead = 0;

volatile uint8_t  txDmaEnabled = false;

///////////////////////////////////////////////////////////////////////////////

static void uartTxDMA(void)
{
    if ((txDmaEnabled == true) || (txBufferHead == txBufferTail))  // Ignore call if already active or no new data in buffer
        return;

    DMA1_Channel4->CMAR = (uint32_t) & txBuffer[txBufferTail];

    if (txBufferHead > txBufferTail)
    {
        DMA1_Channel4->CNDTR = txBufferHead - txBufferTail;
        txBufferTail = txBufferHead;
    }
    else
    {
        DMA1_Channel4->CNDTR = UART_BUFFER_SIZE - txBufferTail;
        txBufferTail = 0;
    }

    txDmaEnabled = true;

    DMA_Cmd(DMA1_Channel4, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////

void DMA1_Channel4_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);

    txDmaEnabled = false;

    uartTxDMA();
}

///////////////////////////////////////////////////////////////////////////////

enum { expandEvr = 1 };

void cliListenerCB(evr_t e)
{
    if (expandEvr)
        cliPrintF("EVR-%s %8.3fs %s (%04x)\n", evrToSeverityStr(e.evr), (float)e.time/1000., evrToStr(e.evr), e.reason);
    else
        cliPrintF("EVR:%08x %04x %04x\n", e.time, e.evr, e.reason);
}

///////////////////////////////////////////////////////////////////////////////

void cliInit(uint32_t baudRate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // UART
    // USART1_TX    PA9
    // USART1_RX    PA10

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate            = baudRate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    // Receive DMA into a circular buffer

    DMA_DeInit(DMA1_Channel5);

    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t) rxBuffer;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_BufferSize         = UART_BUFFER_SIZE;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;

    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel5, ENABLE);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    rxDMAPos = DMA_GetCurrDataCounter(DMA1_Channel5);

    // Transmit DMA

    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & USART1->DR;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;

    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

    DMA1_Channel4->CNDTR = 0;

    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(USART1, ENABLE);

    evrRegisterListener(cliListenerCB);
}

///////////////////////////////////////////////////////////////////////////////

uint16_t cliAvailable(void)
{
    return (DMA_GetCurrDataCounter(DMA1_Channel5) != rxDMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////

bool cliTransmitEmpty(void)
{
    return (txBufferTail == txBufferHead);
}

///////////////////////////////////////////////////////////////////////////////

uint8_t cliRead(void)
{
    uint8_t ch;

    ch = rxBuffer[UART_BUFFER_SIZE - rxDMAPos];
    // go back around the buffer
    if (--rxDMAPos == 0)
        rxDMAPos = UART_BUFFER_SIZE;

    return ch;
}

///////////////////////////////////////////////////////////////////////////////

uint8_t cliReadPoll(void)
{
    while (!cliAvailable());   // wait for some bytes
    return cliRead();
}

///////////////////////////////////////////////////////////////////////////////

void cliWrite(uint8_t ch)
{
    txBuffer[txBufferHead] = ch;
    txBufferHead = (txBufferHead + 1) % UART_BUFFER_SIZE;

    uartTxDMA();
}

///////////////////////////////////////////////////////////////////////////////

void cliPrint(char *str)
{
    while (*str)
        cliWrite(*(str++));
}

///////////////////////////////////////////////////////////////////////////////
// CLI Print Formatted - Print formatted string to CLI
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void cliPrintF(const char * fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start (vlist, fmt);

	vsnprintf(buf, sizeof(buf), fmt, vlist);
	cliPrint(buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
// CLI Print Binary String
///////////////////////////////////////////////////////////////////////////////

void cliPrintBinary(uint8_t *buf, uint16_t length)
{
    uint16_t i;

   for (i = 0; i < length; i++)
    {
    	txBuffer[txBufferHead] = buf[i];
    	txBufferHead = (txBufferHead + 1) % UART_BUFFER_SIZE;
    }

	uartTxDMA();
}

///////////////////////////////////////////////////////////////////////////////
