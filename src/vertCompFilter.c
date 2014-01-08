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
// Vertical Complementary Filter Defines and Variables
///////////////////////////////////////////////////////////////////////////////

float   accelZ;
float   estimationError = 0.0f;
float   hDotEstimate    = 0.0f;
float   hEstimate;
uint8_t previousExecUp  = false;

///////////////////////////////////////////////////////////////////////////////
// Vertical Complementary Filter
///////////////////////////////////////////////////////////////////////////////

void vertCompFilter(float dt)
{
    if ((execUp == true) && (previousExecUp == false))
    	hEstimate = sensors.pressureAlt50Hz;

    previousExecUp = execUp;

	if (execUp == true)
    {
    	accelZ = -earthAxisAccels[ZAXIS] + eepromConfig.compFilterB * estimationError;

        hDotEstimate += accelZ * dt;

        hEstimate += (hDotEstimate + eepromConfig.compFilterA * estimationError) * dt;

        estimationError = sensors.pressureAlt50Hz - hEstimate;
    }
}

///////////////////////////////////////////////////////////////////////////////




