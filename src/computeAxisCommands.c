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

float   attCmd[3];

float   attPID[3];

float   axisPID[3];

float   rateCmd[3];

float   headingReference;

float   altitudeHoldReference;

float   throttleReference;

float   verticalVelocityCmd;

///////////////////////////////////////////////////////////////////////////////
// Compute Axis Commands
///////////////////////////////////////////////////////////////////////////////

void computeAxisCommands(float dt)
{
    if (flightMode == ATTITUDE)
    {
        attCmd[ROLL ] = rxCommand[ROLL ] * eepromConfig.attitudeScaling;
        attCmd[PITCH] = rxCommand[PITCH] * eepromConfig.attitudeScaling;
    }

    if (flightMode >= ATTITUDE)
    {
        attPID[ROLL]  = updatePID( attCmd[ROLL ],  sensors.attitude500Hz[ROLL ], dt, holdIntegrators, &eepromConfig.PID[ROLL_ATT_PID ] );
        attPID[PITCH] = updatePID( attCmd[PITCH], -sensors.attitude500Hz[PITCH], dt, holdIntegrators, &eepromConfig.PID[PITCH_ATT_PID] );
    }

    if (flightMode == RATE)
    {
        rateCmd[ROLL ] = rxCommand[ROLL ] * eepromConfig.rollAndPitchRateScaling;
        rateCmd[PITCH] = rxCommand[PITCH] * eepromConfig.rollAndPitchRateScaling;
    }
    else
    {
        rateCmd[ROLL ] = attPID[ROLL ];
        rateCmd[PITCH] = attPID[PITCH];
    }

    ///////////////////////////////////

    if (headingHoldEngaged == true)  // Heading Hold is ON
        rateCmd[YAW] = updatePID( headingReference, sensors.attitude500Hz[YAW], dt, holdIntegrators, &eepromConfig.PID[HEADING_PID] );
    else                             // Heading Hold is OFF
        rateCmd[YAW] = rxCommand[YAW] * eepromConfig.yawRateScaling;

    ///////////////////////////////////

    axisPID[ROLL ] = updatePID( rateCmd[ROLL ],  sensors.gyro500Hz[ROLL ], dt, holdIntegrators, &eepromConfig.PID[ROLL_RATE_PID ] );
    axisPID[PITCH] = updatePID( rateCmd[PITCH], -sensors.gyro500Hz[PITCH], dt, holdIntegrators, &eepromConfig.PID[PITCH_RATE_PID] );
    axisPID[YAW  ] = updatePID( rateCmd[YAW  ],  sensors.gyro500Hz[YAW  ], dt, holdIntegrators, &eepromConfig.PID[YAW_RATE_PID  ] );

    ///////////////////////////////////

	if (verticalModeState == ALT_DISENGAGED_THROTTLE_ACTIVE)            // Manual Mode is ON
        throttleCmd = rxCommand[THROTTLE];

    else
    {
		if ((verticalModeState == ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT) ||  // Altitude Hold is ON
            (verticalModeState == ALT_HOLD_AT_REFERENCE_ALTITUDE)   ||
            (verticalModeState == ALT_DISENGAGED_THROTTLE_INACTIVE))
        {

			verticalVelocityCmd = updatePID( altitudeHoldReference, hEstimate, dt, holdIntegrators, &eepromConfig.PID[H_PID] );
		}
        else                                                            // Vertical Velocity Hold is ON
        {
            verticalVelocityCmd = verticalReferenceCommand * eepromConfig.hDotScaling;
        }

    	throttleCmd = throttleReference + updatePID( verticalVelocityCmd, hDotEstimate, dt, holdIntegrators, &eepromConfig.PID[HDOT_PID] );
	}
}

///////////////////////////////////////////////////////////////////////////////
