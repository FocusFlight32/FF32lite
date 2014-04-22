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

// Cycle counter stuff - these should be defined by CMSIS, but they aren't
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  ((volatile uint32_t *)0xE0001004)
#define CYCCNTENA   (1 << 0)

///////////////////////////////////////////////////////////////////////////////

// Cycles per microsecond
static volatile uint32_t usTicks = 0;

///////////////////////////////////////////////////////////////////////////////

// Current uptime for 1kHz systick timer. will rollover after 49 days.
// Hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickCycleCounter = 0;

///////////////////////////////////////////////////////////////////////////////
// Cycle Counter
///////////////////////////////////////////////////////////////////////////////

static void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;

    // enable DWT access
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // enable the CPU cycle counter
    DWT_CTRL |= CYCCNTENA;
}

///////////////////////////////////////
// Frame Timing Variables
///////////////////////////////////////

uint16_t frameCounter = 0;

semaphore_t frame_500Hz = false;
semaphore_t frame_100Hz = false;
semaphore_t frame_50Hz  = false;
semaphore_t frame_10Hz  = false;
semaphore_t frame_5Hz   = false;
semaphore_t frame_1Hz   = false;

uint32_t deltaTime1000Hz, executionTime1000Hz, previous1000HzTime;
uint32_t deltaTime500Hz,  executionTime500Hz,  previous500HzTime;
uint32_t deltaTime100Hz,  executionTime100Hz,  previous100HzTime;
uint32_t deltaTime50Hz,   executionTime50Hz,   previous50HzTime;
uint32_t deltaTime10Hz,   executionTime10Hz,   previous10HzTime;
uint32_t deltaTime5Hz,    executionTime5Hz,    previous5HzTime;
uint32_t deltaTime1Hz,    executionTime1Hz,    previous1HzTime;

float dt500Hz, dt100Hz;

semaphore_t systemReady = false;

semaphore_t execUp = false;

///////////////////////////////////////////////////////////////////////////////
// SysTick
///////////////////////////////////////////////////////////////////////////////

void SysTick_Handler(void)
{
    uint32_t currentTime;

    sysTickCycleCounter = *DWT_CYCCNT;
    sysTickUptime++;

    watchDogsTick();

    if ((systemReady        == true ) &&
    	(cliBusy            == false) &&
    	(accelCalibrating   == false) &&
    	(escCalibrating     == false) &&
    	(magCalibrating     == false) &&
    	(mpuCalibrating     == false))

    {
        frameCounter++;
        if (frameCounter > FRAME_COUNT)
            frameCounter = 1;

        ///////////////////////////////

        currentTime = micros();
        deltaTime1000Hz = currentTime - previous1000HzTime;
        previous1000HzTime = currentTime;

        ///////////////////////////////

        if ((frameCounter % COUNT_500HZ) == 0)
        {
            frame_500Hz = true;

            if (eepromConfig.useMpu6050 == true)
            {
            	readMpu6050();
            }
            else
            {
            	readAdxl345();
            	readMpu3050();
            }

            accelData500Hz[XAXIS] = rawAccel[XAXIS].value;
            accelData500Hz[YAXIS] = rawAccel[YAXIS].value;
            accelData500Hz[ZAXIS] = rawAccel[ZAXIS].value;

            gyroData500Hz[ROLL ] = rawGyro[ROLL ].value;
			gyroData500Hz[PITCH] = rawGyro[PITCH].value;
            gyroData500Hz[YAW  ] = rawGyro[YAW  ].value;
        }

        ///////////////////////////////

        if ((frameCounter % COUNT_100HZ) == 0)
        {
            frame_100Hz = true;

            if (eepromConfig.useMs5611 == true)
            {
                if (!newTemperatureReading)
    			{
    				ms5611ReadTemperatureRequestPressure();
    			    newTemperatureReading = true;
    			}
    			else
    			{
    			    ms5611ReadPressureRequestTemperature();
    			    newPressureReading = true;
    			}
            }
            else
            {
            	if (!newTemperatureReading)
                {
					bmp085ReadTemperatureRequestPressure();
					newTemperatureReading = true;
				}
                else
                {
					bmp085ReadPressureRequestTemperature();
                    newPressureReading = true;
				}
            }
        }

        ///////////////////////////////

        if ((frameCounter % COUNT_50HZ) == 0)
        {
            frame_50Hz = true;
        }

        ///////////////////////////////

        if (((frameCounter + 1) % COUNT_10HZ) == 0)
            newMagData = readMag();

        if ((frameCounter % COUNT_10HZ) == 0)
            frame_10Hz = true;

        ///////////////////////////////

        if ((frameCounter % COUNT_5HZ) == 0)
            frame_5Hz = true;

        ///////////////////////////////

        if ((frameCounter % COUNT_1HZ) == 0)
            frame_1Hz = true;

        ///////////////////////////////////

        executionTime1000Hz = micros() - currentTime;

        ///////////////////////////////
    }
}

///////////////////////////////////////////////////////////////////////////////
// System Time in Microseconds
//
// Note: This can be called from within IRQ Handlers, so uses LDREX/STREX.
// If a higher priority IRQ or DMA or anything happens the STREX will fail
// and restart the loop. Otherwise the same number that was read is harmlessly
// written back.
///////////////////////////////////////////////////////////////////////////////

uint32_t micros(void)
{
    register uint32_t oldCycle, cycle, timeMs;

    do
    {
        timeMs = __LDREXW(&sysTickUptime);
        cycle = *DWT_CYCCNT;
        oldCycle = sysTickCycleCounter;
    }
    while ( __STREXW( timeMs , &sysTickUptime ) );

    return (timeMs * 1000) + (cycle - oldCycle) / usTicks;
}

///////////////////////////////////////////////////////////////////////////////
// System Time in Milliseconds
///////////////////////////////////////////////////////////////////////////////

uint32_t millis(void)
{
    return sysTickUptime;
}

///////////////////////////////////////////////////////////////////////////////
// System Initialization
///////////////////////////////////////////////////////////////////////////////

void checkResetType(void)
{
    uint32_t rst = RCC->CSR;

    evrPush(( rst & (RCC_CSR_PORRSTF | RCC_CSR_PINRSTF | RCC_CSR_SFTRSTF) ) ? EVR_NormalReset : EVR_AbnormalReset , rst >> 24 );

    RCC_ClearFlag();
}

///////////////////////////////////////

void systemInit(void)
{
    RCC_ClocksTypeDef rccClocks;

    ///////////////////////////////////

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);

    // Turn on peripherial clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,   ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,     ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2,     ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,   ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,   ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,   ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,   ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,   ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,   ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

        ///////////////////////////////////////////////////////////////////////////

    checkFirstTime(false);
	readEEPROM();

    if (eepromConfig.receiverType == SPEKTRUM)
		checkSpektrumBind();

	checkResetType();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 2 bits for pre-emption priority, 2 bits for subpriority

	initMixer();

	///////////////////////////////////

	cliPortAvailable         = &uart1Available;
	cliPortPrint             = &uart1Print;
	cliPortPrintF            = &uart1PrintF;
	cliPortRead              = &uart1Read;

    //gpsPortClearBuffer       = &uart2ClearBuffer;
    //gpsPortNumCharsAvailable = &uart2NumCharsAvailable;
    //gpsPortPrintBinary       = &uart2PrintBinary;
    //gpsPortRead              = &uart2Read;

	mavlinkPortPrintBinary   = &uart1PrintBinary;

	telemPortPrintF          = &uart1PrintF;

	///////////////////////////////////

	uart1Init(115200);
    gpioInit();
    adcInit();

    LED0_ON;

    delay(10000);  // 10 seconds of 20 second delay for sensor stabilization

    #ifdef __VERSION__
        cliPortPrintF("\ngcc version " __VERSION__ "\n");
    #endif

    cliPortPrintF("\nFF32lite Firmware V%s, Build Date " __DATE__ " "__TIME__" \n", __FF32LITE_VERSION);

    if ((RCC->CR & RCC_CR_HSERDY) != RESET)
    {
        cliPortPrint("\nRunning on external HSE clock....\n");
    }
    else
    {
        cliPortPrint("\nERROR: Running on internal HSI clock....\n");
    }

    RCC_GetClocksFreq(&rccClocks);

    cliPortPrintF("\nADCCLK-> %2d MHz\n",   rccClocks.ADCCLK_Frequency / 1000000);
    cliPortPrintF(  "HCLK->   %2d MHz\n",   rccClocks.HCLK_Frequency   / 1000000);
    cliPortPrintF(  "PCLK1->  %2d MHz\n",   rccClocks.PCLK1_Frequency  / 1000000);
    cliPortPrintF(  "PCLK2->  %2d MHz\n",   rccClocks.PCLK2_Frequency  / 1000000);
    cliPortPrintF(  "SYSCLK-> %2d MHz\n\n", rccClocks.SYSCLK_Frequency / 1000000);

    if (eepromConfig.useMpu6050 == true)
    	cliPortPrint("Using MPU6050....\n\n");
    else
    	cliPortPrint("Using ADXL345/MPU3050....\n\n");

    if (eepromConfig.useMs5611 == true)
    	cliPortPrint("Using MS5611....\n\n");
    else
    	cliPortPrint("Using BMP085....\n\n");

    if (eepromConfig.receiverType == PPM)
    	cliPortPrint("Using PPM Receiver....\n\n");
    else
    	cliPortPrint("Using Spektrum Satellite Receiver....\n\n");

    delay(10000);  // Remaining 10 seconds of 20 second delay for sensor stabilization - probably not long enough..

    LED1_ON;

    batteryInit();

    if (eepromConfig.receiverType == SPEKTRUM)
        spektrumInit();
    else
        ppmRxInit();

    i2cInit(I2C2);

    initFirstOrderFilter();
    initPID();

    if (eepromConfig.useMpu6050 == true)
    	initMpu6050();
    else
   {
    	initAdxl345();
    	initMpu3050();
   }

    initMag();

    if (eepromConfig.useMs5611 == true)
    	initMs5611();
    else
    	initBmp085();
}

///////////////////////////////////////////////////////////////////////////////
// Delay Microseconds
///////////////////////////////////////////////////////////////////////////////

void delayMicroseconds(uint32_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = *DWT_CYCCNT;

    for (;;) {
        register uint32_t current_count = *DWT_CYCCNT;
        uint32_t elapsed_us;

        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;

        // convert to microseconds
        elapsed_us = elapsed / usTicks;
        if (elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;

        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Delay Milliseconds
///////////////////////////////////////////////////////////////////////////////

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

///////////////////////////////////////////////////////////////////////////////
// System Reset
///////////////////////////////////////////////////////////////////////////////

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(bool toBootloader)
{
    if (toBootloader) {
        // 1FFFF000 -> 20000200 -> SP
        // 1FFFF004 -> 1FFFF021 -> PC
        *((uint32_t *) 0x20004FF0) = 0xDEADBEEF;        // 20KB STM32F103
    }
    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t) 0x04;
}

///////////////////////////////////////////////////////////////////////////////
