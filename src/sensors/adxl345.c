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
// ADXL345 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

// Address

#define ADXL345_ADDRESS 0x53

// Registers

#define ADXL345_OFSX        0x1E
#define ADXL345_OFSY        0x1F
#define ADXL345_OFSZ        0x20
#define ADXL345_BW_RATE     0x2C
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0      0x32

// Bits

#define MEASURE             0x08

#define FULL_RES            0x08
#define RANGE_4_G           0x01

#define DATA_RATE_1600      0x0E

///////////////////////////////////

uint8_t adxl345Calibrating = false;

///////////////////////////////////////////////////////////////////////////////
// Compute ADXL345 Runtime Data
///////////////////////////////////////////////////////////////////////////////

void computeAdxl345RTData(void)
{
    uint8_t  axis;
    uint16_t samples;
    float accelSum[3] = { 0.0f, 0.0f, 0.0f };

    adxl345Calibrating = true;

    for (samples = 0; samples < 2000; samples++)
    {
        readAdxl345();

        accelSum[XAXIS] += ((float)rawAccel[XAXIS].value - eepromConfig.accelBias[XAXIS]) * eepromConfig.accelScaleFactor[XAXIS];
        accelSum[YAXIS] += ((float)rawAccel[YAXIS].value - eepromConfig.accelBias[YAXIS]) * eepromConfig.accelScaleFactor[YAXIS];
        accelSum[ZAXIS] += ((float)rawAccel[ZAXIS].value - eepromConfig.accelBias[ZAXIS]) * eepromConfig.accelScaleFactor[ZAXIS];

        delayMicroseconds(1000);
    }

    for (axis = XAXIS; axis < 3; axis++)
    {
        accelSum[axis] = (float)accelSum[axis] / 2000.0f;
    }

    accelOneG = sqrt(SQR(accelSum[XAXIS]) + SQR(accelSum[YAXIS]) + SQR(accelSum[ZAXIS]));

    adxl345Calibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
// Read Accel
///////////////////////////////////////////////////////////////////////////////

void readAdxl345(void)
{
    uint8_t buffer[6];

    i2cRead(ADXL345_ADDRESS, ADXL345_DATAX0, 6, buffer);

    rawAccel[YAXIS].bytes[0] = buffer[0];
    rawAccel[YAXIS].bytes[1] = buffer[1];
    rawAccel[XAXIS].bytes[0] = buffer[2];
    rawAccel[XAXIS].bytes[1] = buffer[3];
    rawAccel[ZAXIS].bytes[0] = buffer[4];
    rawAccel[ZAXIS].bytes[1] = buffer[5];
}

///////////////////////////////////////////////////////////////////////////////
// Accel Initialization
///////////////////////////////////////////////////////////////////////////////

void initAdxl345(void)
{
    i2cWrite(ADXL345_ADDRESS, ADXL345_POWER_CTL, MEASURE);

    delay(10);

    i2cWrite(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, FULL_RES | RANGE_4_G);

    delay(10);

    i2cWrite(ADXL345_ADDRESS, ADXL345_BW_RATE, DATA_RATE_1600);

    delay(100);

   computeAdxl345RTData();
}

///////////////////////////////////////////////////////////////////////////////
