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
// MPU Temperature Calibration
///////////////////////////////////////////////////////////////////////////////

void mpu3050Calibration(void)
{
    uint16_t sampleRate      = 1000;
    uint16_t numberOfSamples = 2000;

    float gyroBias1[3]        = { 0.0f, 0.0f, 0.0f };
    float mpu3050Temperature1 = 0.0f;

    float gyroBias2[3]        = { 0.0f, 00.f, 0.0f };
    float mpu3050Temperature2 = 0.0f;

    uint16_t index;

    mpuCalibrating = true;

    cliPortPrint("\nGyro Temperature Calibration:\n");

    ///////////////////////////////////
    // Get samples at temperature1
    ///////////////////////////////////

    cliPortPrint("\nBegin 1st Gyro Measurements...\n");
    for (index = 0; index < numberOfSamples; index++)
    {
        readMpu3050();

        gyroBias1[ROLL ]    += rawGyro[ROLL].value;
        gyroBias1[PITCH]    += rawGyro[PITCH].value;
        gyroBias1[YAW  ]    += rawGyro[YAW].value;
        mpu3050Temperature1 += ((float) rawMpuTemperature.value + 13200.0f) / 280.0f + 35.0f;

        delayMicroseconds(sampleRate);
    }

    gyroBias1[ROLL]     /= (float) numberOfSamples;
    gyroBias1[PITCH]    /= (float) numberOfSamples;
    gyroBias1[YAW]      /= (float) numberOfSamples;
    mpu3050Temperature1 /= (float) numberOfSamples;

    cliPortPrintF("\nGyro Temperature Reading: %6.2f", mpu3050Temperature1);

    cliPortPrint("\n\nEnd 1st Gyro Measurements\n");

    ///////////////////////////////////
    // Time delay for temperature
    // Stabilizaiton
    ///////////////////////////////////

    cliPortPrint("\nWaiting for 10 minutes for gyro temp to rise...\n");
    delay(600000);              // Number of mSec in 10 minutes

    ///////////////////////////////////
    // Get samples at temperature2
    ///////////////////////////////////

    cliPortPrint("\nBegin 2nd Gyro Measurements...\n");
    for (index = 0; index < numberOfSamples; index++)
    {
        readMpu3050();

        gyroBias2[ROLL]     += rawGyro[ROLL].value;
        gyroBias2[PITCH]    += rawGyro[PITCH].value;
        gyroBias2[YAW]      += rawGyro[YAW].value;
        mpu3050Temperature2 += ((float) rawMpuTemperature.value + 13200.0f) / 280.0f + 35.0f;

        delayMicroseconds(sampleRate);
    }

    gyroBias2[ROLL ]    /= (float) numberOfSamples;
    gyroBias2[PITCH]    /= (float) numberOfSamples;
    gyroBias2[YAW  ]    /= (float) numberOfSamples;
    mpu3050Temperature2 /= (float) numberOfSamples;

    cliPortPrintF("\nGyro Temperature Reading: %6.2f", mpu3050Temperature2);

    cliPortPrint("\n\nEnd 2nd Gyro Measurements\n");

    eepromConfig.gyroTCBiasSlope[ROLL ] = (gyroBias2[ROLL ] - gyroBias1[ROLL ]) / (mpu3050Temperature2 - mpu3050Temperature1);
    eepromConfig.gyroTCBiasSlope[PITCH] = (gyroBias2[PITCH] - gyroBias1[PITCH]) / (mpu3050Temperature2 - mpu3050Temperature1);
    eepromConfig.gyroTCBiasSlope[YAW  ] = (gyroBias2[YAW  ] - gyroBias1[YAW  ]) / (mpu3050Temperature2 - mpu3050Temperature1);

    eepromConfig.gyroTCBiasIntercept[ROLL ] = gyroBias2[ROLL ] - eepromConfig.gyroTCBiasSlope[ROLL ] * mpu3050Temperature2;
    eepromConfig.gyroTCBiasIntercept[PITCH] = gyroBias2[PITCH] - eepromConfig.gyroTCBiasSlope[PITCH] * mpu3050Temperature2;
    eepromConfig.gyroTCBiasIntercept[YAW  ] = gyroBias2[YAW  ] - eepromConfig.gyroTCBiasSlope[YAW  ] * mpu3050Temperature2;

    mpuCalibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
