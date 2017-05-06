/*!
 * \file   VL53L0X_i2c_platform.c
 * \brief  Code function defintions for ChibiOS HAL
 * \author Romain Reignier
 * \date 14/01/2017
 *
 */

#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"
#include <string.h>

#define STATUS_OK    0
#define STATUS_ERROR 1


static const I2CDriver* i2cDriver;
static i2cflags_t errors = 0;
static const systime_t timeout = OSAL_MS2ST(400);

int VL53L0X_i2c_init(const I2CDriver* driver, const I2CConfig* config)
{
    i2cDriver = driver;
    i2cStart(i2cDriver, config);
    return STATUS_OK;
}

int32_t VL53L0X_comms_close(void)
{
    i2cStop(i2cDriver);
    return STATUS_OK;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    // Size of 20 chosen randomly...
    uint8_t buff[20];
    // Add index at the beginning of the buffer
    buff[0] = index;
    memcpy(buff + 1, pdata, count);
    i2cAcquireBus(i2cDriver);
    msg_t status = i2cMasterTransmitTimeout(i2cDriver, address, buff, count + 1, NULL, 0, timeout);
    i2cReleaseBus(i2cDriver);
    if(status != MSG_OK)
    {
        errors = i2cGetErrors(i2cDriver);
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    i2cAcquireBus(i2cDriver);
    msg_t status = i2cMasterTransmitTimeout(i2cDriver, address, &index, 1, pdata, count, timeout);
    i2cReleaseBus(i2cDriver);
    if(status != MSG_OK)
    {
        errors = i2cGetErrors(i2cDriver);
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    return VL53L0X_write_multi(address, index, &data, 1);
}

int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    uint8_t buff[2];
    buff[1] = data & 0xFF;
    buff[0] = data >> 8;
    return VL53L0X_write_multi(address, index, buff, 2);
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    uint8_t buff[4];
    buff[3] = data & 0xFF;
    buff[2] = data >> 8;
    buff[1] = data >> 16;
    buff[0] = data >> 24;
    return VL53L0X_write_multi(address, index, buff, 4);
}

int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    return VL53L0X_read_multi(address, index, pdata, 1);
}

int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    uint8_t buff[2];
    int32_t status = VL53L0X_read_multi(address, index, buff, 2);
    uint16_t tmp;
    tmp = buff[0];
    tmp <<= 8;
    tmp |= buff[1];
    *pdata = tmp;
    return status;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    uint8_t buff[4];
    int32_t status = VL53L0X_read_multi(address, index, buff, 4);
    uint32_t tmp;
    tmp = buff[0];
    tmp <<= 8;
    tmp |= buff[1];
    tmp <<= 8;
    tmp |= buff[2];
    tmp <<= 8;
    tmp |= buff[3];
    *pdata = tmp;
    return status;
}
