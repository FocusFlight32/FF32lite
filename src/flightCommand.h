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
// Process Pilot Commands Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define DEADBAND       24
#define DEADBAND_SLOPE (1000/(1000-DEADBAND))

#define ALT_DEADBAND       200
#define ALT_DEADBAND_SLOPE (1000/(1000-ALT_DEADBAND))

#define THROTTLE_WINDOW    48

extern float rxCommand[8];

extern uint8_t commandInDetent[3];
extern uint8_t previousCommandInDetent[3];

extern uint8_t channelOrder[8];

///////////////////////////////////////////////////////////////////////////////
// Flight Mode Defines and Variables
///////////////////////////////////////////////////////////////////////////////

extern uint8_t flightMode;

extern uint8_t headingHoldEngaged;

///////////////////////////////////////////////////////////////////////////////
// Arm State Variables
///////////////////////////////////////////////////////////////////////////////

extern semaphore_t armed;
extern uint8_t     armingTimer;
extern uint8_t     disarmingTimer;

///////////////////////////////////////////////////////////////////////////////
// Verical Mode State Variables
///////////////////////////////////////////////////////////////////////////////

extern uint8_t  verticalModeState;

extern uint8_t  vertRefCmdInDetent;

extern float    verticalReferenceCommand;

///////////////////////////////////////////////////////////////////////////////
// Process Flight Commands
///////////////////////////////////////////////////////////////////////////////

void processFlightCommands(void);

///////////////////////////////////////////////////////////////////////////////
