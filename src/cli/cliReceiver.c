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
// Receiver CLI
///////////////////////////////////////////////////////////////////////////////

void receiverCLI()
{
    char     rcOrderString[9];
    float    tempFloat;
    uint8_t  index;
    uint8_t  receiverQuery = 'x';
    uint8_t  validQuery    = false;

    NVIC_InitTypeDef  NVIC_InitStructure;

    cliBusy = true;

    cliPrint("\nEntering Receiver CLI....\n\n");

    while(true)
    {
        cliPrint("Receiver CLI -> ");

		while ((cliAvailable() == false) && (validQuery == false));

		if (validQuery == false)
		    receiverQuery = cliRead();

		cliPrint("\n");

		switch(receiverQuery)
		{
            ///////////////////////////

            case 'a': // Receiver Configuration
                cliPrint("\nReceiver Type:                  ");
                switch(eepromConfig.receiverType)
                {
                    case PPM:
                        cliPrint("PPM\n");
                        break;
                    case SPEKTRUM:
                        cliPrint("Spektrum\n");
                        break;
		        }

                cliPrint("Current RC Channel Assignment:  ");
                for (index = 0; index < 8; index++)
                    rcOrderString[eepromConfig.rcMap[index]] = rcChannelLetters[index];

                rcOrderString[index] = '\0';

                cliPrint(rcOrderString);  cliPrint("\n");

                cliPrintF("Secondary Spektrum:             ");

                if ((eepromConfig.slaveSpektrum == true) && false)  // HJI Inhibit Slave Spektrum on Naze32 Pro
                    cliPrintF("Installed\n");
                else
                    cliPrintF("Uninstalled\n");

                cliPrintF("Mid Command:                    %4ld\n",   (uint16_t)eepromConfig.midCommand);
				cliPrintF("Min Check:                      %4ld\n",   (uint16_t)eepromConfig.minCheck);
				cliPrintF("Max Check:                      %4ld\n",   (uint16_t)eepromConfig.maxCheck);
				cliPrintF("Min Throttle:                   %4ld\n",   (uint16_t)eepromConfig.minThrottle);
				cliPrintF("Max Thottle:                    %4ld\n\n", (uint16_t)eepromConfig.maxThrottle);

				tempFloat = eepromConfig.rateScaling * 180000.0 / PI;
				cliPrintF("Max Rate Command:               %6.2f DPS\n", tempFloat);

				tempFloat = eepromConfig.attitudeScaling * 180000.0 / PI;
                cliPrintF("Max Attitude Command:           %6.2f Degrees\n\n", tempFloat);

				cliPrintF("Arm Delay Count:                %3d Frames\n",   eepromConfig.armCount);
				cliPrintF("Disarm Delay Count:             %3d Frames\n\n", eepromConfig.disarmCount);

				validQuery = false;
				break;

            ///////////////////////////

			case 'x':
			    cliPrint("\nExiting Receiver CLI....\n\n");
			    cliBusy = false;
			    return;
			    break;

            ///////////////////////////

            case 'A': // Toggle PPM/Spektrum Satellite Receiver
            	NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
            	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
            	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
            	NVIC_InitStructure.NVIC_IRQChannelCmd                = DISABLE;

            	NVIC_Init(&NVIC_InitStructure);

            	if (eepromConfig.receiverType == PPM)
                {
                	TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE);
                	eepromConfig.receiverType = SPEKTRUM;
                    spektrumInit();
                }
                else
                {
                	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
                  	eepromConfig.receiverType = PPM;
                    ppmRxInit();
                }

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'B': // Read RC Control Order
                readStringCLI( rcOrderString, 8 );
                parseRcChannels( rcOrderString );

          	    receiverQuery = 'a';
                validQuery = true;
        	    break;

            ///////////////////////////

            case 'C': // Toggle Slave Spektrum State
                // HJI Inhibit Slave Spektrum on Naze32 Pro if (eepromConfig.slaveSpektrum == true)
                    eepromConfig.slaveSpektrum = false;
                // HJI Inhibit Slave Spektrum on Naze32 Pro else
                // HJI Inhibit Slave Spektrum on Naze32 Pro	    eepromConfig.slaveSpektrum = true;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'D': // Read RC Control Points
                eepromConfig.midCommand   = readFloatCLI();
    	        eepromConfig.minCheck     = readFloatCLI();
    		    eepromConfig.maxCheck     = readFloatCLI();
    		    eepromConfig.minThrottle  = readFloatCLI();
    		    eepromConfig.maxThrottle  = readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'E': // Read Arm/Disarm Counts
                eepromConfig.armCount    = (uint8_t)readFloatCLI();
        	    eepromConfig.disarmCount = (uint8_t)readFloatCLI();

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'F': // Read Max Rate Value
                eepromConfig.rateScaling = readFloatCLI() / 180000.0f * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'G': // Read Max Attitude Value
                eepromConfig.attitudeScaling = readFloatCLI() / 180000.0f * PI;

                receiverQuery = 'a';
                validQuery = true;
                break;

            ///////////////////////////

            case 'W': // Write EEPROM Parameters
                cliPrint("\nWriting EEPROM Parameters....\n\n");
                writeEEPROM();
                break;

			///////////////////////////

			case '?':
			   	cliPrint("\n");
			   	cliPrint("'a' Receiver Configuration Data            'A' Toggle PPM/Spektrum Receiver\n");
   		        cliPrint("                                           'B' Set RC Control Order                 BTAER1234\n");
			   	cliPrint("                                           'D' Set RC Control Points                DmidCmd;minChk;maxChk;minThrot;maxThrot\n");
			   	cliPrint("                                           'E' Set Arm/Disarm Counts                EarmCount;disarmCount\n");
			   	cliPrint("                                           'F' Set Maximum Rate Command\n");
			   	cliPrint("                                           'G' Set Maximum Attitude Command\n");
			   	cliPrint("                                           'W' Write EEPROM Parameters\n");
			   	cliPrint("'x' Exit Receiver CLI                      '?' Command Summary\n");
			   	cliPrint("\n");
	    	    break;

	    	///////////////////////////
	    }
	}

}

///////////////////////////////////////////////////////////////////////////////
