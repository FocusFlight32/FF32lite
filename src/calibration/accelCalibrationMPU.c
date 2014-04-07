/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the AQ32 Flight Control Board

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// Accelerometer Calibration
///////////////////////////////////////////////////////////////////////////////

void accelCalibrationMPU(void)
{
    float noseUp        = 0.0f;
	float noseDown      = 0.0f;
	float leftWingDown  = 0.0f;
	float rightWingDown = 0.0f;
	float upSideDown    = 0.0f;
    float rightSideUp   = 0.0f;

    int16_t index;

    accelCalibrating = true;

    cliPrint("\nMPU6000 Accelerometer Calibration:\n\n");

    ///////////////////////////////////

    cliPrint("Place accelerometer right side up\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        readMpu6050();

        computeMpu6050TCBias();

        rightSideUp += (float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS];
        delayMicroseconds(1000);
    }

    rightSideUp /= 5000.0f;

    cliPrint("Place accelerometer up side down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        readMpu6050();

        computeMpu6050TCBias();

        upSideDown += (float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS];
        delayMicroseconds(1000);
    }

    upSideDown /= 5000.0f;

    ///////////////////////////////////

    cliPrint("Place accelerometer left edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        readMpu6050();

        computeMpu6050TCBias();

        leftWingDown += (float)rawAccel[YAXIS].value - accelTCBias[YAXIS];
        delayMicroseconds(1000);
    }

    leftWingDown /= 5000.0f;

    cliPrint("Place accelerometer right edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        readMpu6050();

        computeMpu6050TCBias();

        rightWingDown += (float)rawAccel[YAXIS].value - accelTCBias[YAXIS];
        delayMicroseconds(1000);
    }

    rightWingDown /= 5000.0f;

    ///////////////////////////////////

    cliPrint("Place accelerometer rear edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        readMpu6050();

        computeMpu6050TCBias();

        noseUp += (float)rawAccel[XAXIS].value - accelTCBias[XAXIS];
        delayMicroseconds(1000);
    }

    noseUp /= 5000.0f;

    cliPrint("Place accelerometer front edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        readMpu6050();

        computeMpu6050TCBias();

        noseDown += (float)rawAccel[XAXIS].value - accelTCBias[XAXIS];
        delayMicroseconds(1000);
    }

    noseDown /= 5000.0f;

    ///////////////////////////////////

    eepromConfig.accelBias[ZAXIS]        = (rightSideUp + upSideDown) / 2.0f;
    eepromConfig.accelScaleFactor[ZAXIS] = (2.0f * 9.8065f) / fabsf(rightSideUp - upSideDown);

    eepromConfig.accelBias[YAXIS]        = (leftWingDown + rightWingDown) / 2.0f;
    eepromConfig.accelScaleFactor[YAXIS] = (2.0f * 9.8065f) / fabsf(leftWingDown - rightWingDown);

    eepromConfig.accelBias[XAXIS]        = (noseUp + noseDown) / 2.0f;
    eepromConfig.accelScaleFactor[XAXIS] = (2.0f * 9.8065f) / fabsf(noseUp - noseDown);

    ///////////////////////////////////

	accelCalibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
