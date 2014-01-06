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

#define MPU6050_ADDRESS             0x68

#define MPU6050_CONFIG              0x1A

#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03

#define MPU6050_ACCEL_SCALE_FACTOR  0.00119708f  // (1/8192) * 9.8065  (8192 LSB = 1 G)
#define MPU6050_GYRO_SCALE_FACTOR   0.00026646f  // (1/65.5) * pi/180  (65.5 LSB = 1 DPS)

///////////////////////////////////////////////////////////////////////////////
// MPU6050 Variables
///////////////////////////////////////////////////////////////////////////////

extern float accelTCBias[3];

///////////////////////////////////////////////////////////////////////////////
// MPU6050 Initialization
///////////////////////////////////////////////////////////////////////////////

void initMpu6050(void);

///////////////////////////////////////////////////////////////////////////////
// Read MPU6050
///////////////////////////////////////////////////////////////////////////////

void readMpu6050(void);

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6050 Runtime Data
///////////////////////////////////////////////////////////////////////////////

void computeMpu6050RTData(void);

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6050 Temperature Compensation Bias
///////////////////////////////////////////////////////////////////////////////

void computeMpu6050TCBias(void);

///////////////////////////////////////////////////////////////////////////////
