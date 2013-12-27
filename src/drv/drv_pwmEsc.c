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
// PWM ESC Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define ESC_PULSE_1MS    2000  // 1ms pulse width

static volatile uint16_t *OutputChannels[] = { &(TIM4->CCR4),
	                                           &(TIM4->CCR3),
	                                           &(TIM4->CCR2),
	                                           &(TIM4->CCR1),
	                                           &(TIM1->CCR4),
	                                           &(TIM1->CCR1),};

///////////////////////////////////////////////////////////////////////////////
// PWM ESC Initialization
///////////////////////////////////////////////////////////////////////////////

void pwmEscInit(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    // Outputs
    // ESC PWM1  TIM4_CH4  PB9
    // ESC PWM2  TIM4_CH3  PB8
    // ESC PWM3  TIM4_CH2  PB7
    // ESC PWM4  TIM4_CH1  PB6
    // ESC PWM5  TIM1_CH4  PA11
    // ESC PWM6  TIM1_CH1  PA8

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

 	GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 |
                                    GPIO_Pin_8 | GPIO_Pin_9;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    ///////////////////////////////////

    // Output timers

	TIM_TimeBaseStructure.TIM_Period            = (uint16_t)(2000000 / eepromConfig.escPwmRate) - 1;
    TIM_TimeBaseStructure.TIM_Prescaler         = 36 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

    TIM_TimeBaseInit(TIM4,  &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = ESC_PULSE_1MS;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);

    TIM_Cmd(TIM4, ENABLE);

    if (eepromConfig.mixerConfiguration == MIXERTYPE_TRI)
	{
	    TIM_TimeBaseStructure.TIM_Period = (uint16_t)(2000000 / eepromConfig.triYawServoPwmRate) - 1;
		TIM_OCInitStructure.TIM_Pulse    = eepromConfig.triYawServoMid;
	}

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// PWM ESC Write
///////////////////////////////////////////////////////////////////////////////

void pwmEscWrite(uint8_t channel, uint16_t value)
{
    *OutputChannels[channel] = value;
}

///////////////////////////////////////////////////////////////////////////////
