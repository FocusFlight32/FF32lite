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

float           accelOneG = 9.8065;

int16_t         accelData500Hz[3];

int16andUint8_t rawAccel[3];

///////////////////////////////////////////////////////////////////////////////

float           gyroRTBias[3];

float           gyroTCBias[3];

int16_t         gyroData500Hz[3];

int16andUint8_t rawGyro[3];

uint8_t         mpuCalibrating = false;

float           mpuTemperature;

int16andUint8_t rawMpuTemperature;

///////////////////////////////////////////////////////////////////////////////

uint8_t         newPressureReading    = false;

uint8_t         newTemperatureReading = false;

///////////////////////////////////////////////////////////////////////////////