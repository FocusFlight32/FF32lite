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

eepromConfig_t eepromConfig;

uint8_t        execUpCount = 0;

sensors_t      sensors;

///////////////////////////////////////////////////////////////////////////////

int main(void)
{
	///////////////////////////////////////////////////////////////////////////

	uint32_t currentTime;

    systemReady = false;

    systemInit();

    systemReady = true;

    evrPush(EVR_StartingMain, 0);

    while (1)
    {
    	evrCheck();

    	///////////////////////////////

        if (frame_50Hz)
        {
        	frame_50Hz = false;

        	currentTime      = micros();
			deltaTime50Hz    = currentTime - previous50HzTime;
			previous50HzTime = currentTime;

			processFlightCommands();

            if (eepromConfig.useMs5611 == true)
			{
				if (newTemperatureReading && newPressureReading)
                {
                    d1Value = d1.value;
                    d2Value = d2.value;

                    calculateMs5611Temperature();
                    calculateMs5611PressureAltitude();

                    newTemperatureReading = false;
                    newPressureReading    = false;
                }
           }
           else
           {
			   if (newTemperatureReading && newPressureReading)
			   {
				   uncompensatedTemperatureValue = uncompensatedTemperature.value;
				   uncompensatedPressureValue    = uncompensatedPressure.value;

				   calculateBmp085Temperature();
				   calculateBmp085PressureAltitude();

				   newTemperatureReading = false;
				   newPressureReading    = false;
			   }
           }

            sensors.pressureAlt50Hz = firstOrderFilter(sensors.pressureAlt50Hz, &firstOrderFilters[PRESSURE_ALT_LOWPASS]);

            executionTime50Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_10Hz)
        {
        	frame_10Hz = false;

        	currentTime      = micros();
			deltaTime10Hz    = currentTime - previous10HzTime;
			previous10HzTime = currentTime;

			sensors.mag10Hz[XAXIS] = -((float)rawMag[XAXIS].value * magScaleFactor[XAXIS] - eepromConfig.magBias[XAXIS]);
			sensors.mag10Hz[YAXIS] =   (float)rawMag[YAXIS].value * magScaleFactor[YAXIS] - eepromConfig.magBias[YAXIS];
			sensors.mag10Hz[ZAXIS] = -((float)rawMag[ZAXIS].value * magScaleFactor[ZAXIS] - eepromConfig.magBias[ZAXIS]);

			newMagData = false;
			magDataUpdate = true;

        	cliCom();

        	batMonTick();

        	executionTime10Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_500Hz)
        {
        	frame_500Hz = false;

       	    currentTime       = micros();
       	    deltaTime500Hz    = currentTime - previous500HzTime;
       	    previous500HzTime = currentTime;

       	    dt500Hz = (float)deltaTime500Hz * 0.000001f;  // For integrations in 500 Hz loop

            if (eepromConfig.useMpu6050 == true)
            {
			    computeMpu6050TCBias();

                sensors.accel500Hz[XAXIS] = -((float)accelData500Hz[XAXIS] - accelTCBias[XAXIS]) * MPU6050_ACCEL_SCALE_FACTOR;
                sensors.accel500Hz[YAXIS] = -((float)accelData500Hz[YAXIS] - accelTCBias[YAXIS]) * MPU6050_ACCEL_SCALE_FACTOR;
                sensors.accel500Hz[ZAXIS] = -((float)accelData500Hz[ZAXIS] - accelTCBias[ZAXIS]) * MPU6050_ACCEL_SCALE_FACTOR;

                sensors.gyro500Hz[ROLL ] =  ((float)gyroData500Hz[ROLL ] - gyroRTBias[ROLL ] - gyroTCBias[ROLL ]) * MPU6050_GYRO_SCALE_FACTOR;
                sensors.gyro500Hz[PITCH] = -((float)gyroData500Hz[PITCH] - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * MPU6050_GYRO_SCALE_FACTOR;
                sensors.gyro500Hz[YAW  ] = -((float)gyroData500Hz[YAW  ] - gyroRTBias[YAW  ] - gyroTCBias[YAW  ]) * MPU6050_GYRO_SCALE_FACTOR;
			}
			else
            {
				sensors.accel500Hz[XAXIS] = -((float)accelData500Hz[XAXIS] - eepromConfig.accelBias[XAXIS]) * eepromConfig.accelScaleFactor[XAXIS];
			    sensors.accel500Hz[YAXIS] = -((float)accelData500Hz[YAXIS] - eepromConfig.accelBias[YAXIS]) * eepromConfig.accelScaleFactor[YAXIS];
			    sensors.accel500Hz[ZAXIS] = -((float)accelData500Hz[ZAXIS] - eepromConfig.accelBias[ZAXIS]) * eepromConfig.accelScaleFactor[ZAXIS];

                // HJI sensors.accel500Hz[XAXIS] = firstOrderFilter(sensors.accel500Hz[XAXIS], &firstOrderFilters[ACCEL500HZ_X_LOWPASS]);
                // HJI sensors.accel500Hz[YAXIS] = firstOrderFilter(sensors.accel500Hz[YAXIS], &firstOrderFilters[ACCEL500HZ_Y_LOWPASS]);
                // HJI sensors.accel500Hz[ZAXIS] = firstOrderFilter(sensors.accel500Hz[ZAXIS], &firstOrderFilters[ACCEL500HZ_Z_LOWPASS]);

                computeMpu3050TCBias();

                sensors.gyro500Hz[ROLL ] =  ((float)gyroData500Hz[ROLL ]  - gyroRTBias[ROLL ] - gyroTCBias[ROLL ]) * MPU3050_GYRO_SCALE_FACTOR;
			    sensors.gyro500Hz[PITCH] = -((float)gyroData500Hz[PITCH]  - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * MPU3050_GYRO_SCALE_FACTOR;
                sensors.gyro500Hz[YAW  ] = -((float)gyroData500Hz[YAW  ]  - gyroRTBias[YAW  ] - gyroTCBias[YAW  ]) * MPU3050_GYRO_SCALE_FACTOR;
		    }

            MargAHRSupdate( sensors.gyro500Hz[ROLL],   sensors.gyro500Hz[PITCH],  sensors.gyro500Hz[YAW],
                            sensors.accel500Hz[XAXIS], sensors.accel500Hz[YAXIS], sensors.accel500Hz[ZAXIS],
                            sensors.mag10Hz[XAXIS],    sensors.mag10Hz[YAXIS],    sensors.mag10Hz[ZAXIS],
                            magDataUpdate,
                            dt500Hz );

            magDataUpdate = false;

            computeAxisCommands(dt500Hz);
            mixTable();
            //writeServos();
            writeMotors();

            executionTime500Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_100Hz)
        {
        	frame_100Hz = false;

        	currentTime       = micros();
			deltaTime100Hz    = currentTime - previous100HzTime;
			previous100HzTime = currentTime;

			dt100Hz = (float)deltaTime100Hz * 0.000001f;  // For integrations in 100 Hz loop

            sensors.accel100Hz[XAXIS] = sensors.accel500Hz[XAXIS];  // No sensor averaging so use the 500 Hz value
			sensors.accel100Hz[YAXIS] = sensors.accel500Hz[YAXIS];  // No sensor averaging so use the 500 Hz value
			sensors.accel100Hz[ZAXIS] = sensors.accel500Hz[ZAXIS];  // No sensor averaging so use the 500 Hz value

        	// HJI sensors.accel100Hz[XAXIS] = firstOrderFilter(sensors.accel100Hz[XAXIS], &firstOrderFilters[ACCEL100HZ_X_LOWPASS]);
			// HJI sensors.accel100Hz[YAXIS] = firstOrderFilter(sensors.accel100Hz[YAXIS], &firstOrderFilters[ACCEL100HZ_Y_LOWPASS]);
			// HJI sensors.accel100Hz[ZAXIS] = firstOrderFilter(sensors.accel100Hz[ZAXIS], &firstOrderFilters[ACCEL100HZ_Z_LOWPASS]);

			createRotationMatrix();
            bodyAccelToEarthAccel();
            vertCompFilter(dt100Hz);

            if (armed == true)
            {
				if ( eepromConfig.activeTelemetry == 1 )
                {
            	    // 500 Hz Accels
            	    cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.accel500Hz[XAXIS],
            	            	                       sensors.accel500Hz[YAXIS],
            	            			               sensors.accel500Hz[ZAXIS]);
                }

                if ( eepromConfig.activeTelemetry == 2 )
                {
            	    // 500 Hz Gyros
            	    cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.gyro500Hz[ROLL ],
            	            			               sensors.gyro500Hz[PITCH],
            	            					       sensors.gyro500Hz[YAW  ]);
                }

                if ( eepromConfig.activeTelemetry == 4 )
                {
            	    // 500 Hz Attitudes
            	    cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.attitude500Hz[ROLL ],
            	            			               sensors.attitude500Hz[PITCH],
            	            			               sensors.attitude500Hz[YAW  ]);
                }

                if ( eepromConfig.activeTelemetry == 8 )
                {
               	    // Vertical Variables
            	    cliPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", earthAxisAccels[ZAXIS],
            	    		                                  sensors.pressureAlt50Hz,
            	    		                                  hDotEstimate,
            	    		                                  hEstimate);
                }

                if ( eepromConfig.activeTelemetry == 16 )
                {
               	    // Vertical Variables
            	    cliPrintF("%9.4f, %9.4f, %9.4f,%1d, %9.4f, %9.4f\n", verticalVelocityCmd,
            	    		                                             hDotEstimate,
            	    		                                             hEstimate,
            	    		                                             verticalModeState,
            	    		                                             throttleCmd,
            	    		                                             eepromConfig.PID[HDOT_PID].iTerm);
                }

            }

            executionTime100Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_5Hz)
        {
        	frame_5Hz = false;

        	currentTime     = micros();
			deltaTime5Hz    = currentTime - previous5HzTime;
			previous5HzTime = currentTime;

			while (batMonVeryLowWarning > 0)
			{
				BEEP_TOGGLE;
				batMonVeryLowWarning--;
			}

        	executionTime5Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_1Hz)
        {
        	frame_1Hz = false;

        	currentTime     = micros();
			deltaTime1Hz    = currentTime - previous1HzTime;
			previous1HzTime = currentTime;

			if (execUp == false)
			    execUpCount++;

			if ((execUpCount == 5) && (execUp == false))
			{
				execUp = true;
				LED0_OFF;
				LED1_OFF;
				pwmEscInit();
			}

			while (batMonLowWarning > 0)
			{
				BEEP_TOGGLE;
				batMonLowWarning--;
			}

			executionTime1Hz = micros() - currentTime;
        }

        ////////////////////////////////
    }

    ///////////////////////////////////////////////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
