/**
 * @file hal.c
 *
 * @brief Example of a hardware abstraction layer
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "hal.h"
/* Following statements relate to use of TI Driver implementation */
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CMSP432E4.h>
extern I2C_Handle g_I2Chandle;

//****************************************************************************
//
// Internal variables
//
//****************************************************************************
// Flag to indicate if an ALERT/RDY interrupt has occurred
static volatile bool flag_nALERT_INTERRUPT = false;
//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************
void InitGPIO(void);
void InitI2C(void);

//****************************************************************************
//
// External Functions (prototypes declared in hal.h)
//
//****************************************************************************
/**
 * @brief getALERTinterruptStatus()
 * Gets the current status of ALERT/RDY interrupt flag.
 *
 * @ return boolean status of flag_nALERT_INTERRUPT.
 */
bool getALERTinterruptStatus(void)
{
   return flag_nALERT_INTERRUPT;
}
/**
 * @brief setALERTinterruptStatus(void)
 * Sets the value of the ALERT/RDY interrupt flag.
 *
 * @param[in] value where status is set with true; false clears the status.
 *
 * @return none
 */
void setALERTinterruptStatus(const bool value)
{
    flag_nALERT_INTERRUPT = value;
}
/**
 *
 * @brief enableALERTinterrupt()
 * Enables or disables the ALERT/RDY interrupt.
 *
 * @param[in] intEnable Where interrupt is enabled with true; false disables the interrupt.
 *
 * @return none
 */
void enableALERTinterrupt(const bool intEnable)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    if(intEnable)
    {
        flag_nALERT_INTERRUPT = false;
        GPIO_clearInt(ALERT_CONST);
        SysCtlDelay(10);
        GPIO_enableInt(ALERT_CONST);
    }
    else GPIO_disableInt(ALERT_CONST);
}
/**
 *
 * @brief InitADC()
 * Initializes MCU peripherals for interfacing with the ADC.
 *
 * @return none
 */
void InitADC(void)
{
    // IMPORTANT: Make sure device is powered before setting GPIOs pins to HIGH state.

    // Initialize GPIOs pins used by ADS1x15
    InitGPIO();
    /* Call driver init functions */
    InitI2C();
    // Run ADC startup function (in ADS1115.c)
    adcStartup();
}

//****************************************************************************
//
// Timing functions
//
//****************************************************************************

/**
 *
 * @brief delay_ms()
 * Provides a timing delay with 'ms' resolution.
 *
 * @param[in] delay_time_ms Is the number of milliseconds to delay.
 *
 * @return none
 */
void delay_ms(const uint32_t delay_time_ms)
{
    /* --- INSERT YOUR CODE HERE --- */

    const uint32_t cycles_per_loop = 3;
    uint32_t delayTime = getSysClockHz() / (cycles_per_loop * 1000u);
    delayTime = delayTime * delay_time_ms;
    MAP_SysCtlDelay( delayTime );
}
/**
 *
 * @brief InitI2C()
 * Configures the MCU's I2C peripheral, for interfacing with target devices.
 *
 * @return none
 */
void InitI2C(void)
{
    /* --- INSERT YOUR CODE HERE --- */

    //
    // Enabling I2C2 peripheral devices.
    // testfunction();

    //
    // Configuring the pin muxing for I2C2 functions.
    //

    //
    // Enabling and initializing the I2C2 master module.
    //

    /* The following code is based on a TI Drivers implementation */

    /* Call driver init functions */
    I2C_init();
}