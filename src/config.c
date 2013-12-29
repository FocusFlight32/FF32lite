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

#define FLASH_PAGE_COUNT 128

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)

// use the last KB for sensor config storage
#define FLASH_WRITE_EEPROM_CONFIG_ADDR  (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))

const char rcChannelLetters[] = "AERT1234";

float vTailThrust;

static uint8_t checkNewEEPROMConf = 4;

///////////////////////////////////////////////////////////////////////////////

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++)
    {
        s = strchr(rcChannelLetters, *c);
        if (s)
            eepromConfig.rcMap[s - rcChannelLetters] = c - input;
    }
}

///////////////////////////////////////////////////////////////////////////////

void readEEPROM(void)
{
    // Read flash

	memcpy(&eepromConfig, (char *)FLASH_WRITE_EEPROM_CONFIG_ADDR, sizeof(eepromConfig_t));

	accConfidenceDecay = 1.0f / sqrt(eepromConfig.accelCutoff);

	eepromConfig.yawDirection = constrain( eepromConfig.yawDirection, -1.0f, 1.0f );
}

///////////////////////////////////////////////////////////////////////////////

void writeEEPROM(void)
{
    FLASH_Status status;
    uint32_t i;

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(FLASH_WRITE_EEPROM_CONFIG_ADDR) == FLASH_COMPLETE)
    {
        for (i = 0; i < sizeof(eepromConfig_t); i += 4)
        {
            status = FLASH_ProgramWord(FLASH_WRITE_EEPROM_CONFIG_ADDR + i, *(uint32_t *)((char *)&eepromConfig + i));
            if (status != FLASH_COMPLETE)
                break; // TODO: fail
        }
    }

    FLASH_Lock();

    readEEPROM();
}

///////////////////////////////////////////////////////////////////////////////

void checkFirstTime(bool eepromReset)
{
    uint8_t test_val;

    test_val = *(uint8_t *)FLASH_WRITE_EEPROM_CONFIG_ADDR;

    if (eepromReset || test_val != checkNewEEPROMConf)
    {
		// Default settings
        eepromConfig.version = checkNewEEPROMConf;

	    ///////////////////////////////

        eepromConfig.accelBias[XAXIS] = 0.0f;
        eepromConfig.accelBias[YAXIS] = 0.0f;
        eepromConfig.accelBias[ZAXIS] = 0.0f;

        ///////////////////////////////

        eepromConfig.accelScaleFactor[XAXIS] = 9.8065f / 256.0f;
        eepromConfig.accelScaleFactor[YAXIS] = 9.8065f / 256.0f;
        eepromConfig.accelScaleFactor[ZAXIS] = 9.8065f / 256.0f;

        ///////////////////////////////

        eepromConfig.gyroTCBiasSlope[ROLL ] = 0.0f;
        eepromConfig.gyroTCBiasSlope[PITCH] = 0.0f;
        eepromConfig.gyroTCBiasSlope[YAW  ] = 0.0f;

	    ///////////////////////////////

	    eepromConfig.gyroTCBiasIntercept[ROLL ] = 0.0f;
	    eepromConfig.gyroTCBiasIntercept[PITCH] = 0.0f;
	    eepromConfig.gyroTCBiasIntercept[YAW  ] = 0.0f;

	    ///////////////////////////////

	    eepromConfig.magBias[XAXIS] = 0.0f;
	    eepromConfig.magBias[YAXIS] = 0.0f;
	    eepromConfig.magBias[ZAXIS] = 0.0f;

		///////////////////////////////

		eepromConfig.accelCutoff = 1.0f;

		///////////////////////////////

		eepromConfig.KpAcc = 5.0f;  // proportional gain governs rate of convergence to accelerometer
	    eepromConfig.KiAcc = 0.0f;  // integral gain governs rate of convergence of gyroscope biases
	    eepromConfig.KpMag = 5.0f;  // proportional gain governs rate of convergence to magnetometer
	    eepromConfig.KiMag = 0.0f;  // integral gain governs rate of convergence of gyroscope biases

	    ///////////////////////////////

	    eepromConfig.compFilterA =  2.000f;
	    eepromConfig.compFilterB =  1.000f;

	    ///////////////////////////////

	    eepromConfig.rateScaling     = 300.0 / 180000.0 * PI;  // Stick to rate scaling for 300 DPS

	    eepromConfig.attitudeScaling = 60.0  / 180000.0 * PI;  // Stick to att scaling for 60 degrees

	    eepromConfig.hDotScaling     = 0.003f;      // Stick to hDot scaling (3 mps)/(1000 RX PWM Steps) = 0.003

	    ///////////////////////////////

        eepromConfig.receiverType  = SPEKTRUM;

        eepromConfig.slaveSpektrum = false;

        parseRcChannels("TAER2134");

        eepromConfig.escPwmRate   = 450;
        eepromConfig.servoPwmRate = 50;

        eepromConfig.mixerConfiguration = MIXERTYPE_QUADX;
        eepromConfig.yawDirection = 1.0f;

        eepromConfig.triYawServoPwmRate = 50;
        eepromConfig.triYawServoMin     = 2000.0f;
        eepromConfig.triYawServoMid     = 3000.0f;
        eepromConfig.triYawServoMax     = 4000.0f;

        eepromConfig.midCommand   = 3000.0f;
        eepromConfig.minCheck     = (float)(MINCOMMAND + 200);
        eepromConfig.maxCheck     = (float)(MAXCOMMAND - 200);
        eepromConfig.minThrottle  = (float)(MINCOMMAND + 200);
        eepromConfig.maxThrottle  = (float)(MAXCOMMAND);

        eepromConfig.PID[ROLL_RATE_PID].B               =   1.0f;
        eepromConfig.PID[ROLL_RATE_PID].P               = 250.0f;
        eepromConfig.PID[ROLL_RATE_PID].I               = 100.0f;
        eepromConfig.PID[ROLL_RATE_PID].D               =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].iTerm           =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].windupGuard     = 100.0f;  // PWMs
        eepromConfig.PID[ROLL_RATE_PID].lastDcalcValue  =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].lastDterm       =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].lastLastDterm   =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].dErrorCalc      =   D_ERROR;
        eepromConfig.PID[ROLL_RATE_PID].type            =   OTHER;

        eepromConfig.PID[PITCH_RATE_PID].B              =   1.0f;
        eepromConfig.PID[PITCH_RATE_PID].P              = 250.0f;
        eepromConfig.PID[PITCH_RATE_PID].I              = 100.0f;
        eepromConfig.PID[PITCH_RATE_PID].D              =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].iTerm          =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].windupGuard    = 100.0f;  // PWMs
        eepromConfig.PID[PITCH_RATE_PID].lastDcalcValue =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].lastDterm      =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].lastLastDterm  =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].dErrorCalc     =   D_ERROR;
        eepromConfig.PID[PITCH_RATE_PID].type           =   OTHER;

        eepromConfig.PID[YAW_RATE_PID].B                =   1.0f;
        eepromConfig.PID[YAW_RATE_PID].P                = 350.0f;
        eepromConfig.PID[YAW_RATE_PID].I                = 100.0f;
        eepromConfig.PID[YAW_RATE_PID].D                =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].iTerm            =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].windupGuard      = 100.0f;  // PWMs
        eepromConfig.PID[YAW_RATE_PID].lastDcalcValue   =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].lastDterm        =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].lastLastDterm    =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].dErrorCalc       =   D_ERROR;
        eepromConfig.PID[YAW_RATE_PID].type             =   OTHER;

        eepromConfig.PID[ROLL_ATT_PID].B                =   1.0f;
        eepromConfig.PID[ROLL_ATT_PID].P                =   2.0f;
        eepromConfig.PID[ROLL_ATT_PID].I                =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].D                =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].iTerm            =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].windupGuard      =   0.5f;  // radians/sec
        eepromConfig.PID[ROLL_ATT_PID].lastDcalcValue   =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].lastDterm        =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].lastLastDterm    =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].dErrorCalc       =   D_ERROR;
        eepromConfig.PID[ROLL_ATT_PID].type             =   ANGULAR;

        eepromConfig.PID[PITCH_ATT_PID].B               =   1.0f;
        eepromConfig.PID[PITCH_ATT_PID].P               =   2.0f;
        eepromConfig.PID[PITCH_ATT_PID].I               =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].D               =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].iTerm           =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].windupGuard     =   0.5f;  // radians/sec
        eepromConfig.PID[PITCH_ATT_PID].lastDcalcValue  =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].lastDterm       =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].lastLastDterm   =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].dErrorCalc      =   D_ERROR;
        eepromConfig.PID[PITCH_ATT_PID].type            =   ANGULAR;

        eepromConfig.PID[HEADING_PID].B                 =   1.0f;
        eepromConfig.PID[HEADING_PID].P                 =   3.0f;
        eepromConfig.PID[HEADING_PID].I                 =   0.0f;
        eepromConfig.PID[HEADING_PID].D                 =   0.0f;
        eepromConfig.PID[HEADING_PID].iTerm             =   0.0f;
        eepromConfig.PID[HEADING_PID].windupGuard       =   0.5f;  // radians/sec
        eepromConfig.PID[HEADING_PID].lastDcalcValue    =   0.0f;
        eepromConfig.PID[HEADING_PID].lastDterm         =   0.0f;
        eepromConfig.PID[HEADING_PID].lastLastDterm     =   0.0f;
        eepromConfig.PID[HEADING_PID].dErrorCalc        =   D_ERROR;
        eepromConfig.PID[HEADING_PID].type              =   ANGULAR;

        eepromConfig.PID[HDOT_PID].B                    =   1.0f;
        eepromConfig.PID[HDOT_PID].P                    =   2.0f;
        eepromConfig.PID[HDOT_PID].I                    =   0.0f;
        eepromConfig.PID[HDOT_PID].D                    =   0.0f;
        eepromConfig.PID[HDOT_PID].iTerm                =   0.0f;
        eepromConfig.PID[HDOT_PID].windupGuard          =   5.0f;
        eepromConfig.PID[HDOT_PID].lastDcalcValue       =   0.0f;
        eepromConfig.PID[HDOT_PID].lastDterm            =   0.0f;
        eepromConfig.PID[HDOT_PID].lastLastDterm        =   0.0f;
        eepromConfig.PID[HDOT_PID].dErrorCalc           =   D_ERROR;
        eepromConfig.PID[HDOT_PID].type                 =   OTHER;

        eepromConfig.PID[H_PID].B                       =   1.0f;
        eepromConfig.PID[H_PID].P                       =   2.0f;
        eepromConfig.PID[H_PID].I                       =   0.0f;
        eepromConfig.PID[H_PID].D                       =   0.0f;
        eepromConfig.PID[H_PID].iTerm                   =   0.0f;
        eepromConfig.PID[H_PID].windupGuard             =   5.0f;
        eepromConfig.PID[H_PID].lastDcalcValue          =   0.0f;
        eepromConfig.PID[H_PID].lastDterm               =   0.0f;
        eepromConfig.PID[H_PID].lastLastDterm           =   0.0f;
        eepromConfig.PID[H_PID].dErrorCalc              =   D_ERROR;
        eepromConfig.PID[H_PID].type                    =   OTHER;

		eepromConfig.batteryCells         = 3;
		eepromConfig.voltageMonitorScale  = 11.0f / 1.0f;
		eepromConfig.voltageMonitorBias   = 0.0f;

        eepromConfig.armCount             =  50;
        eepromConfig.disarmCount          =  0;

        writeEEPROM();
	}
}

///////////////////////////////////////////////////////////////////////////////
