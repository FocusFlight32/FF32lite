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

uint8_t escCalibrating = false;
char    temp;

///////////////////////////////////////////////////////////////////////////////
// ESC Calibration
///////////////////////////////////////////////////////////////////////////////

void escCalibration(void)
{
    escCalibrating = true;

    armed = false;

    cliPortPrint("\nESC Calibration:\n\n");
    cliPortPrint("!!!! CAUTION - Remove all propellers and disconnect !!!!\n");
    cliPortPrint("!!!! flight battery before proceeding any further   !!!!\n\n");
    cliPortPrint("Type 'Y' to continue, any other character exits\n\n");

    while (cliPortAvailable() == false);
    temp = cliPortRead();
    if (temp != 'Y')
    {
    	cliPortPrint("ESC Calibration Canceled!!\n\n");
    	escCalibrating = false;
    	return;
    }

    ///////////////////////////////////

    cliPortPrint("For ESC Calibration:\n");
    cliPortPrint("  Enter 'h' for Max Command....\n");
    cliPortPrint("  Enter 'm' for Mid Command....\n");
    cliPortPrint("  Enter 'l' for Min Command....\n");
    cliPortPrint("  Enter 'x' to exit....\n\n");
    cliPortPrint("For Motor Order Verification:\n");
    cliPortPrint("  Enter '0' to turn off all motors....\n");
    cliPortPrint("  Enter '1' to turn on Motor1....\n");
    cliPortPrint("  Enter '2' to turn on Motor2....\n");
    cliPortPrint("  Enter '3' to turn on Motor3....\n");
    cliPortPrint("  Enter '4' to turn on Motor4....\n");
    cliPortPrint("  Enter '5' to turn on Motor5....\n");
    cliPortPrint("  Enter '6' to turn on Motor6....\n\n");

    ///////////////////////////////////

    while(true)
    {
		while (cliPortAvailable() == false);

		temp = cliPortRead();

		switch (temp)
		{
			case 'h':
			    cliPortPrint("Applying Max Command....\n\n");
			    writeAllMotors(eepromConfig.maxThrottle);
			    break;

			case 'm':
			    cliPortPrint("Applying Mid Command....\n\n");
			    writeAllMotors(eepromConfig.midCommand);
			    break;

			case 'l':
			    cliPortPrint("Applying Min Command....\n\n");
			    writeAllMotors(MINCOMMAND);
			    break;

			case 'x':
			    cliPortPrint("Applying Min Command, Exiting Calibration....\n\n");
			    writeAllMotors(MINCOMMAND);
			    escCalibrating = false;
			    return;
			    break;

			case '0':
			    cliPortPrint("Motors at Min Command....\n\n");
			    writeAllMotors(MINCOMMAND);
			    break;

			case '1':
				cliPortPrint("Motor1 at Min Throttle....\n\n");
				pwmEscWrite(0, eepromConfig.minThrottle);
				break;

			case '2':
				cliPortPrint("Motor2 at Min Throttle....\n\n");
				pwmEscWrite(1, eepromConfig.minThrottle);
				break;

			case '3':
				cliPortPrint("Motor3 at Min Throttle....\n\n");
				pwmEscWrite(2, eepromConfig.minThrottle);
				break;

			case '4':
				cliPortPrint("Motor4 at Min Throttle....\n\n");
				pwmEscWrite(3, eepromConfig.minThrottle);
				break;

			case '5':
				cliPortPrint("Motor5 at Min Throttle....\n\n");
				pwmEscWrite(4, eepromConfig.minThrottle);
				break;

			case '6':
				cliPortPrint("Motor6 at Min Throttle....\n\n");
				pwmEscWrite(5, eepromConfig.minThrottle);
				break;

			case '?':
			    cliPortPrint("For ESC Calibration:\n");
			    cliPortPrint("  Enter 'h' for Max Command....\n");
			    cliPortPrint("  Enter 'm' for Mid Command....\n");
			    cliPortPrint("  Enter 'l' for Min Command....\n");
			    cliPortPrint("  Enter 'x' to exit....\n\n");
			    cliPortPrint("For Motor Order Verification:\n");
			    cliPortPrint("  Enter '0' to turn off all motors....\n");
			    cliPortPrint("  Enter '1' to turn on Motor1....\n");
			    cliPortPrint("  Enter '2' to turn on Motor2....\n");
			    cliPortPrint("  Enter '3' to turn on Motor3....\n");
			    cliPortPrint("  Enter '4' to turn on Motor4....\n");
			    cliPortPrint("  Enter '5' to turn on Motor5....\n");
			    cliPortPrint("  Enter '6' to turn on Motor6....\n\n");
			    break;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
