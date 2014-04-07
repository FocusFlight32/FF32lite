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

#pragma once

///////////////////////////////////////////////////////////////////////////////

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "mavlink.h"

///////////////////////////////////////

#include "pid.h"

#include "ff32_Naze32.h"

#include "drv_adc.h"
#include "drv_cli.h"
#include "drv_crc.h"
#include "drv_gpio.h"
#include "drv_i2c.h"
#include "drv_ppmRx.h"
#include "drv_pwmEsc.h"
#include "drv_pwmServo.h"
#include "drv_spektrum.h"
#include "drv_system.h"

#include "sensorCommon.h"

#include "adxl345.h"
#include "bmp085.h"
#include "hmc5883.h"
#include "mpu3050.h"
#include "mpu6050.h"
#include "ms5611.h"

#include "accelCalibrationADXL345.h"
#include "accelCalibrationMPU.h"
#include "batMon.h"
#include "cli.h"
#include "computeAxisCommands.h"
#include "config.h"
#include "coordinateTransforms.h"
#include "escCalibration.h"
#include "evr.h"
#include "firstOrderFilter.h"
#include "flightCommand.h"
#include "magCalibration.h"
#include "mavlinkStrings.h"
#include "MargAHRS.h"
#include "mixer.h"
#include "mpu3050Calibration.h"
#include "mpu6050Calibration.h"
#include "utilities.h"
#include "vertCompFilter.h"
#include "watchdogs.h"

///////////////////////////////////////////////////////////////////////////////
