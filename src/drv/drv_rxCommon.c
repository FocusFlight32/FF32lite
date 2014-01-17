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
// TIM2 Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void TIM2_IRQHandler(void)
{
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t  chan = 0;

    if (eepromConfig.receiverType == SPEKTRUM)
    {
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    if (primarySpektrumState.spektrumTimer)
        --primarySpektrumState.spektrumTimer;

    if ((eepromConfig.slaveSpektrum == true)  && false)  // HJI Inhibit Slave Spektrum on Naze32
    {
	    if (slaveSpektrumState.spektrumTimer)
            --slaveSpektrumState.spektrumTimer;
    }
    }
    else
    {
    	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET)
        {
            last = now;
            now = TIM_GetCapture1(TIM2);
            rcActive = true;
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

        diff = now - last;

        if (diff > 2700 * 2)   // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960
        {                      // "So, if you use 2.5ms or higher as being the reset for the PPM stream start,
            chan = 0;          // you will be fine. I use 2.7ms just to be safe."
        }
        else
        {
            if (diff > 1500 && diff < 4500 && chan < 8)    // 750 to 2250 ms is our 'valid' channel range
            {
                pulseWidth[chan] = diff;
            }
            chan++;
         }
    }
}

///////////////////////////////////////////////////////////////////////////////
