/**
 * @file ads1115.c
 *
 * @brief This file contains all basic communication and device setup for the ADS1115 device family.
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
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

#include <evm/ADS1115.h>

const char *adcRegisterNames[NUM_REGISTERS] = {"DATA", "CONFIG", "LO_THRESH", "HI_THRESH"};

//****************************************************************************
//
// Internal variables
//
//****************************************************************************

// Array used to recall device register map configurations */
static uint16_t registerMap[NUM_REGISTERS];

//****************************************************************************
//
// Function Definitions
//
//****************************************************************************
/**
 *
 * @brief restoreRegisterDefaults()
 * Updates the registerMap[] array to its default values.
 *
 * NOTES:
 * - If the MCU keeps a copy of the ADS1x15 register settings in memory,
 * then it is important to ensure that these values remain in sync with the
 * actual hardware settings. In order to help facilitate this, this function
 * should be called after powering up or resetting the device.
 *
 * - Reading back all of the registers after resetting the device can
 * accomplish the same result.
 *
 * @return none
 */
static void restoreRegisterDefaults(void)
{
    registerMap[CONFIG_ADDRESS]       = CONFIG_DEFAULT;
	/* The threshold register do not exist on the TLA2024, ADS1013 or ADS1113 */
    registerMap[LO_THRESH_ADDRESS]    = LO_THRESH_DEFAULT;
    registerMap[HI_THRESH_ADDRESS]    = HI_THRESH_DEFAULT;
}

/**
 *
 * @brief adcStartup()
 * Example start up sequence for the ADS1115.
 *
 * Before calling this function, the device must be powered and
 * the I2C/GPIO pins of the MCU must have already been configured.
 *
 * @return none
 */
void adcStartup(void)
{
    //
    // (OPTIONAL) Provide additional delay time for power supply settling
    //
    delay_ms(50);

    //
    // (REQUIRED) Initialize internal 'registerMap' array with device default settings
    //
    restoreRegisterDefaults();

    //
    // (OPTIONAL) Read back all registers and Check STATUS register (if exists) for faults
    //
    registerMap[CONFIG_ADDRESS] = readSingleRegister(CONFIG_ADDRESS);
}

/**
 *
 * @brief readSingleRegister()
 * Reads the contents of a single register at the specified address.
 *
 * @param[in] address Is the 8-bit address of the register to read.
 *
 * @return The 16-bit register read result as an unsigned value, but conversion
 * is binary 2's complement.
 */
uint16_t readSingleRegister(uint8_t address)
{
    //
    // Check that the register address is in range
    //
    assert(address <= MAX_REGISTER_ADDRESS);
    uint16_t regValue = 0;
    uint8_t regRXdata[4] = {0};
    int8_t retStatus;
    retStatus = receiveI2CData(address, regRXdata, 2);
    if(retStatus != false)
    {
        /* I2C communication error took place...Insert a handling routine here */
        return 0;
    }
    regValue =  combineBytes(regRXdata[0], regRXdata[1]);
    registerMap[address] = regValue;
    return regValue;
}

/**
 *
 * @brief readSingleRegister()
 * Reads the contents of a single register at the specified address.
 *
 * @param[in] address Is the 8-bit address of the register to read.
 *
 * @return The 16-bit register read result as an unsigned value, but conversion
 * is binary 2's complement.
 */
uint16_t readSingleRegister(uint8_t address)
{
    //
    // Check that the register address is in range
    //
    assert(address <= MAX_REGISTER_ADDRESS);
    uint16_t regValue = 0;
    uint8_t regRXdata[4] = {0};
    int8_t retStatus;
    retStatus = receiveI2CData(address, regRXdata, 2);
    if(retStatus != false)
    {
        /* I2C communication error took place...Insert a handling routine here */
        return 0;
    }
    regValue =  combineBytes(regRXdata[0], regRXdata[1]);
    registerMap[address] = regValue;
    return regValue;
}