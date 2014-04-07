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
// Accelerometer Calibration
///////////////////////////////////////////////////////////////////////////////

void accelCalibrationADXL345(void)
{
    float noseUp        = 0.0f;
    float noseDown      = 0.0f;
    float leftWingDown  = 0.0f;
    float rightWingDown = 0.0f;
    float upSideDown    = 0.0f;
    float rightSideUp   = 0.0f;

    int16_t index;

    cliPrint("\nAccelerometer Calibration:\n\n");

    ///////////////////////////////////

    cliPrint("Place accelerometer right side up\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++) {
        readAdxl345();
        rightSideUp += (float) rawAccel[ZAXIS].value;
        delayMicroseconds(1000);
    }

    rightSideUp /= 5000.0f;

    cliPrint("Place accelerometer up side down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++) {
        readAdxl345();
        upSideDown += (float) rawAccel[ZAXIS].value;
        delayMicroseconds(1000);
    }

    upSideDown /= 5000.0f;

    ///////////////////////////////////

    cliPrint("Place accelerometer left edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++) {
        readAdxl345();
        leftWingDown += (float) rawAccel[YAXIS].value;
        delayMicroseconds(1000);
    }

    leftWingDown /= 5000.0f;

    cliPrint("Place accelerometer right edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++) {
        readAdxl345();
        rightWingDown += (float) rawAccel[YAXIS].value;
        delayMicroseconds(1000);
    }

    rightWingDown /= 5000.0f;

    ///////////////////////////////////

    cliPrint("Place accelerometer rear edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++) {
        readAdxl345();
        noseUp += (float) rawAccel[XAXIS].value;
        delayMicroseconds(1000);
    }

    noseUp /= 5000.0f;

    cliPrint("Place accelerometer front edge down\n");
    cliPrint("  Send a character when ready to proceed\n\n");

    while (cliAvailable() == false);
    cliRead();

    cliPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++) {
        readAdxl345();
        noseDown += (float) rawAccel[XAXIS].value;
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
