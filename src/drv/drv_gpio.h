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

#pragma once

///////////////////////////////////////////////////////////////////////////////
// LED Defines
////////////////////////////////////////////////////////////////////////////////

#define BEEP_GPIO   GPIOA
#define BEEP_PIN    GPIO_Pin_12

#define LED0_GPIO   GPIOB
#define LED0_PIN    GPIO_Pin_3
#define LED1_GPIO   GPIOB
#define LED1_PIN    GPIO_Pin_4

///////////////////////////////////////

#define BEEP_OFF         GPIO_ResetBits(LED0_GPIO,       BEEP_PIN)
#define BEEP_ON          GPIO_SetBits(LED0_GPIO,         BEEP_PIN)
#define BEEP_TOGGLE      GPIO_ToggleBits(LED0_GPIO,      BEEP_PIN)

#define LED0_OFF         GPIO_SetBits(LED0_GPIO,         LED0_PIN)
#define LED0_ON          GPIO_ResetBits(LED0_GPIO,       LED0_PIN)
#define LED0_TOGGLE      GPIO_ToggleBits(LED0_GPIO,      LED0_PIN)

#define LED1_OFF         GPIO_SetBits(LED1_GPIO,         LED1_PIN)
#define LED1_ON          GPIO_ResetBits(LED1_GPIO,       LED1_PIN)
#define LED1_TOGGLE      GPIO_ToggleBits(LED1_GPIO,      LED1_PIN)

///////////////////////////////////////////////////////////////////////////////
// GPIO Initialization
///////////////////////////////////////////////////////////////////////////////

void gpioInit();

///////////////////////////////////////////////////////////////////////////////
