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
// Frame Timing Defines and Variables
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////
// Frame Timing Defines
///////////////////////////////////////

#define FRAME_COUNT   1000

#define COUNT_500HZ   2         // Number of 1000 Hz frames for 500 Hz Loop
#define COUNT_100HZ   10        // Number of 1000 Hz frames for 100 Hz Loop
#define COUNT_50HZ    20        // Number of 1000 Hz frames for  50 Hz Loop
#define COUNT_10HZ    100       // Number of 1000 Hz frames for  10 Hz Loop
#define COUNT_5HZ     200       // Number of 1000 Hz frames for   5 Hz Loop
#define COUNT_1HZ     1000      // Number of 1000 Hz frames for   1 Hz Loop

///////////////////////////////////////
// Frame Timing Variables
///////////////////////////////////////

extern uint16_t frameCounter;

extern semaphore_t frame_500Hz;
extern semaphore_t frame_100Hz;
extern semaphore_t frame_50Hz;
extern semaphore_t frame_10Hz;
extern semaphore_t frame_5Hz;
extern semaphore_t frame_1Hz;

extern uint32_t deltaTime1000Hz, executionTime1000Hz, previous1000HzTime;
extern uint32_t deltaTime500Hz,  executionTime500Hz,  previous500HzTime;
extern uint32_t deltaTime100Hz,  executionTime100Hz,  previous100HzTime;
extern uint32_t deltaTime50Hz,   executionTime50Hz,   previous50HzTime;
extern uint32_t deltaTime10Hz,   executionTime10Hz,   previous10HzTime;
extern uint32_t deltaTime5Hz,    executionTime5Hz,    previous5HzTime;
extern uint32_t deltaTime1Hz,    executionTime1Hz,    previous1HzTime;

extern float dt500Hz, dt100Hz;

extern semaphore_t systemReady;

extern semaphore_t execUp;

///////////////////////////////////////////////////////////////////////////////

void systemInit(void);

///////////////////////////////////////////////////////////////////////////////

void delayMicroseconds(uint32_t us);

///////////////////////////////////////////////////////////////////////////////

void delay(uint32_t ms);

///////////////////////////////////////////////////////////////////////////////

uint32_t micros(void);

///////////////////////////////////////////////////////////////////////////////

uint32_t millis(void);

///////////////////////////////////////////////////////////////////////////////

void failureMode(uint8_t mode);

///////////////////////////////////////////////////////////////////////////////

void systemReset(bool toBootloader);

///////////////////////////////////////////////////////////////////////////////
