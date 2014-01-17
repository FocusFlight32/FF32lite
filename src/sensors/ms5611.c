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

#define MS5611_ADDRESS  0x77

//#define OSR  256  // 0.60 mSec conversion time (1666.67 Hz)
//#define OSR  512  // 1.17 mSec conversion time ( 854.70 Hz)
//#define OSR 1024  // 2.28 mSec conversion time ( 357.14 Hz)
//#define OSR 2048  // 4.54 mSec conversion time ( 220.26 Hz)
  #define OSR 4096  // 9.04 mSec conversion time ( 110.62 Hz)

///////////////////////////////////////

uint16andUint8_t c1, c2, c3, c4, c5, c6;

uint32andUint8_t d1;

uint32_t d1Value;

uint32andUint8_t d2;

uint32_t d2Value;

int32_t dT;

int32_t ms5611Temperature;

///////////////////////////////////////////////////////////////////////////////
// MS5611 Read Temperature Request Pressure
///////////////////////////////////////////////////////////////////////////////

void ms5611ReadTemperatureRequestPressure(void)
{
    uint8_t data[3];

    i2cRead(MS5611_ADDRESS, 0x00, 3, data);    // Request temperature read

    d2.bytes[2] = data[0];
    d2.bytes[1] = data[1];
    d2.bytes[0] = data[2];

    #if   (OSR ==  256)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x40);  // Request pressure conversion
	#elif (OSR ==  512)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x42);
	#elif (OSR == 1024)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x44);
	#elif (OSR == 2048)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x46);
	#elif (OSR == 4096)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x48);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// MS5611 Read Pressure Request Temperature
///////////////////////////////////////////////////////////////////////////////

void ms5611ReadPressureRequestTemperature(void)
{
    uint8_t data[3];

    i2cRead(MS5611_ADDRESS, 0x00, 3, data);    // Request pressure read

    d1.bytes[2] = data[0];
    d1.bytes[1] = data[1];
    d1.bytes[0] = data[2];

    #if   (OSR ==  256)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x50);   // Request temperature converison
	#elif (OSR ==  512)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x52);
	#elif (OSR == 1024)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x54);
	#elif (OSR == 2048)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x56);
	#elif (OSR == 4096)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x58);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// Calculate MS5611 Temperature
///////////////////////////////////////////////////////////////////////////////

void calculateMs5611Temperature(void)
{
    dT                = (int32_t)d2Value - ((int32_t)c5.value << 8);
    ms5611Temperature = 2000 + (int32_t)(((int64_t)dT * c6.value) >> 23);
}

///////////////////////////////////////////////////////////////////////////////
// Calculate MS5611 Pressure Altitude
///////////////////////////////////////////////////////////////////////////////

void calculateMs5611PressureAltitude(void)
{
    int64_t offset;
	int64_t offset2 = 0;

	int64_t sensitivity;
	int64_t sensitivity2 = 0;

	int64_t f;

	int32_t p;

	int32_t ms5611Temp2  = 0;

	offset      = ((int64_t)c2.value << 16) + (((int64_t)c4.value * dT) >> 7);
	sensitivity = ((int64_t)c1.value << 15) + (((int64_t)c3.value * dT) >> 8);

	if (ms5611Temperature < 2000)
	{
		ms5611Temp2  = SQR(dT) >> 31;

		f	 		 = SQR(ms5611Temperature - 2000);
		offset2      = 5 * f >> 1;
		sensitivity2 = 5 * f >> 2;

		if (ms5611Temperature < -1500)
		{
			f 			  = SQR(ms5611Temperature + 1500);
			offset2      +=  7 * f;
			sensitivity2 += 11 * f >> 1;
		}

		ms5611Temperature -= ms5611Temp2;

		offset -= offset2;
		sensitivity -= sensitivity2;
	}

	p = (((d1Value * sensitivity) >> 21) - offset) >> 15;

	sensors.pressureAlt50Hz = 44330.0f * (1.0f - pow((float)p / 101325.0f, 1.0f / 5.255f));
}

///////////////////////////////////////////////////////////////////////////////
// MS5611 Initialization
///////////////////////////////////////////////////////////////////////////////

void initMs5611(void)
{
    uint8_t data[2];

    i2cWrite(MS5611_ADDRESS, 0xFF, 0x1E);      // Reset Device

    delay(10);

    i2cRead(MS5611_ADDRESS, 0xA2, 2, data);    // Read Calibration Data C1
    c1.bytes[1] = data[0];
    c1.bytes[0] = data[1];

    i2cRead(MS5611_ADDRESS, 0xA4, 2, data);    // Read Calibration Data C2
    c2.bytes[1] = data[0];
    c2.bytes[0] = data[1];

    i2cRead(MS5611_ADDRESS, 0xA6, 2, data);    // Read Calibration Data C3
	c3.bytes[1] = data[0];
    c3.bytes[0] = data[1];

    i2cRead(MS5611_ADDRESS, 0xA8, 2, data);    // Read Calibration Data C4
	c4.bytes[1] = data[0];
    c4.bytes[0] = data[1];

    i2cRead(MS5611_ADDRESS, 0xAA, 2, data);    // Read Calibration Data C5
	c5.bytes[1] = data[0];
    c5.bytes[0] = data[1];

    i2cRead(MS5611_ADDRESS, 0xAC, 2, data);    // Read Calibration Data C6
	c6.bytes[1] = data[0];
    c6.bytes[0] = data[1];

    #if   (OSR ==  256)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x50);  // Request temperature conversion
	#elif (OSR ==  512)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x52);
	#elif (OSR == 1024)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x54);
	#elif (OSR == 2048)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x56);
	#elif (OSR == 4096)
	    i2cWrite(MS5611_ADDRESS, 0xFF, 0x58);
    #endif

    delay(10);

    ms5611ReadTemperatureRequestPressure();
    delay(10);

    ms5611ReadPressureRequestTemperature();
    delay(10);

    d1Value = d1.value;
    d2Value = d2.value;

    calculateMs5611Temperature();
    calculateMs5611PressureAltitude();
}

///////////////////////////////////////////////////////////////////////////////
