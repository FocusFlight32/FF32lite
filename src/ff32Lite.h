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

extern void (*telemPortPrintF)(const char * fmt, ...);

///////////////////////////////////////////////////////////////////////////////

#define __FF32LITE_VERSION "1.0"

///////////////////////////////////////////////////////////////////////////////

#ifndef PI
    #define PI  3.14159265358979f
#endif

#define TWO_PI (2.0f * PI)

#define D2R (PI / 180.0f)

#define R2D (180.0f / PI)

#define SQR(x)  ((x) * (x))

///////////////////////////////////////////////////////////////////////////////

#define ROLL     0
#define PITCH    1
#define YAW      2
#define THROTTLE 3
#define AUX1     4
#define AUX2     5
#define AUX3     6
#define AUX4     7

#define XAXIS    0
#define YAXIS    1
#define ZAXIS    2

#define MINCOMMAND  2000
#define MIDCOMMAND  3000
#define MAXCOMMAND  4000

///////////////////////////////////////////////////////////////////////////////
// Misc Type Definitions
///////////////////////////////////////////////////////////////////////////////

typedef union {
    int16_t value;
    uint8_t bytes[2];
} int16andUint8_t;

typedef union {
    int32_t value;
    uint8_t bytes[4];
} int32andUint8_t;

typedef union {
    uint16_t value;
    uint8_t bytes[2];
} uint16andUint8_t;

typedef union {
	uint32_t value;
	 uint8_t bytes[4];
} uint32andUint8_t;

///////////////////////////////////////

typedef volatile uint8_t semaphore_t;

///////////////////////////////////////////////////////////////////////////////
// Sensor Variables
///////////////////////////////////////////////////////////////////////////////

typedef struct sensors_t
{
    float accel500Hz[3];
    float accel100Hz[3];
    float attitude500Hz[3];
    float gyro500Hz[3];
    float mag10Hz[3];
    float pressureAlt50Hz;
} sensors_t;

extern sensors_t sensors;

typedef struct homeData_t
{
	float   magHeading;
} homeData_t;

extern homeData_t homeData;

///////////////////////////////////////////////////////////////////////////////
// PID Definitions
///////////////////////////////////////////////////////////////////////////////

#define NUMBER_OF_PIDS 8

#define ROLL_RATE_PID  0
#define PITCH_RATE_PID 1
#define YAW_RATE_PID   2

#define ROLL_ATT_PID   3
#define PITCH_ATT_PID  4
#define HEADING_PID    5

#define HDOT_PID       6

#define H_PID          7

///////////////////////////////////////////////////////////////////////////////
// Mixer Configurations
///////////////////////////////////////////////////////////////////////////////

enum { MIXERTYPE_TRI,
       MIXERTYPE_QUADX,
       MIXERTYPE_HEX6X,
       MIXERTYPE_FREE,
     };

///////////////////////////////////////////////////////////////////////////////
// Flight Modes
///////////////////////////////////////////////////////////////////////////////

enum { RATE, ATTITUDE };

///////////////////////////////////////////////////////////////////////////////
// Vertical States
///////////////////////////////////////////////////////////////////////////////

enum { ALT_DISENGAGED_THROTTLE_ACTIVE,
       ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT,
       ALT_HOLD_AT_REFERENCE_ALTITUDE,
       VERTICAL_VELOCITY_HOLD_AT_REFERENCE_VELOCITY,
       ALT_DISENGAGED_THROTTLE_INACTIVE };

///////////////////////////////////////////////////////////////////////////////
// MPU6000 DLPF Configurations
///////////////////////////////////////////////////////////////////////////////

enum { DLPF_256HZ, DLPF_188HZ, DLPF_98HZ, DLPF_42HZ };

///////////////////////////////////////////////////////////////////////////////
// Receiver Configurations
///////////////////////////////////////////////////////////////////////////////

enum { PPM, SPEKTRUM };

///////////////////////////////////////////////////////////////////////////////
// EEPROM
///////////////////////////////////////////////////////////////////////////////

typedef struct eepromConfig_t
{
    uint8_t version;

    uint8_t useMpu6050;

    uint8_t useMs5611;

    float accelBias[3];             // For ADXL345
    float accelScaleFactor[3];      // For ADXL345

    float accelTCBiasSlope[3];      // For MPU6050
    float accelTCBiasIntercept[3];  // For MPU6050

    float gyroTCBiasSlope[3];
    float gyroTCBiasIntercept[3];

    float magBias[3];

    float accelCutoff;

    float KpAcc;

    float KiAcc;

    float KpMag;

    float KiMag;

    float compFilterA;

    float compFilterB;

    uint8_t dlpfSetting;

    ///////////////////////////////////

    float rollAndPitchRateScaling;

    float yawRateScaling;

    float attitudeScaling;

    float hDotScaling;

    ///////////////////////////////////

    uint8_t receiverType;

    uint8_t slaveSpektrum;

    uint8_t rcMap[8];

    uint16_t escPwmRate;
    uint16_t servoPwmRate;

    float midCommand;
    float minCheck;
    float maxCheck;
    float minThrottle;
    float maxThrottle;

    ///////////////////////////////////

    uint8_t mixerConfiguration;
    float yawDirection;

    uint16_t triYawServoPwmRate;
    float    triYawServoMin;
    float    triYawServoMid;
    float    triYawServoMax;
    float    triCopterYawCmd500HzLowPassTau;

    uint8_t  freeMixMotors;

    float    freeMix[6][3];

    ///////////////////////////////////

    PIDdata_t PID[NUMBER_OF_PIDS];

    ///////////////////////////////////

    uint8_t batteryCells;
    float   voltageMonitorScale;
    float   voltageMonitorBias;

    float   batteryLow;
    float   batteryVeryLow;
    float   batteryMaxLow;

    ///////////////////////////////////

    uint8_t armCount;
    uint8_t disarmCount;

    ///////////////////////////////////

    uint16_t activeTelemetry;

    uint8_t  mavlinkEnabled;

	///////////////////////////////////

    uint8_t verticalVelocityHoldOnly;

    ///////////////////////////////////

    uint8_t  CRCFlags;
    uint32_t CRCAtEnd[1];

    ///////////////////////////////////

} eepromConfig_t;

enum crcFlags { CRC_HistoryBad = 1 };

extern eepromConfig_t eepromConfig;

///////////////////////////////////////////////////////////////////////////////
