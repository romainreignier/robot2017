/*
    ChibiOS/PCA9685 - Copyright (C) 2013
                 Jarek Zok <jarek.zok@fwioo.pl>

    This file is part of ChibiOS/PCA9685.

    ChibiOS/PCA9685 is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/PCA9685 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>

#ifdef _cplusplus
extern "C" {
#endif
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#ifdef _cplusplus
}
#endif

#include <cmath>
#include <cstdlib>

#include "PwmBase.hpp"
#include "PCA9685.hpp"


/**
 * PCA9685 class implements methods to access the chip.
 *
 * @param I2CDriver *driver I2C driver used to communicate with the chip
 * @param const I2CConfig *config I2C configuration structure
 * @param uint8_t address Address of the chip, default space starts at 0x40
 * @param uint16_t freq Default frequency of PWM
 * @param uint8_t acquirebus - set I2C bus exclusive for thread
 *
 */
PCA9685::PCA9685(I2CDriver *driver, const I2CConfig *config, uint8_t address, float freq, bool acquirebus) {
    this->i2caddres = address;
    this->driver = driver;
    this->pwm_frequency = freq;
    this->config = config;
    this->acquire = acquirebus; //Prevents from accessing the bus from more than one threads
}

void PCA9685::begin() {
    i2cStart(this->driver, this->config);
    this->reset();
    this->setFreq(this->pwm_frequency);
}

// /**
//  * Static constructor of the class. Some of the parameters are defined in header.
//  * PCA9685_ADDRESS address of the chip (0x40).
//  * PCA9685_DEFI2C_DRIVER as I2C driver (I2CD2 by default)
//  * PCA9685_FREQ as default frequency used
//  * PCA9685_I2C_CONFIG is static structure with proper I2C parameters set.
//  */
// PCA9685::PCA9685() {
//     this->i2caddres = PCA9685_ADDRESS;
//     this->driver = &PCA9685_DEFI2C_DRIVER;
//     this->pwm_frequency = PCA9685_FREQ;
//     this->config = &PCA9685_I2C_CONFIG;
//     this->acquire = true;
//
//     i2cStart(this->driver, this->config);
//     this->reset();
//     this->setFreq(this->pwm_frequency);
// }
//
// /**
//  * Releases bus and device
//  *
//  *
//  */
// PCA9685::~PCA9685() {
//     i2cStop(this->driver);
// }
// 
// void* PCA9685::operator new(size_t size) {
//     return chCoreAlloc(size);
// }
// 
// void PCA9685::operator delete(void *mem) {
// 
// }

/**
 * Send reset to chip, wait for oscillator to stabilize.
 *
 */
void PCA9685::reset(void) {
    this->writereg(PCA9685_MODE1, 0x00);
    chThdSleepMilliseconds(1);//Wait for oscillator stabilisation
}

/**
 * Set PWM frequency in Hz.
 * @param uint16_t freq - 40Hz min. to 10KHz max, if external 50MHz clock for the chip is provided
 *
 * @return uint16_t previous frequency value
 *
 */
void PCA9685::setFreq(float freq) {

    //According to NXP documentation (7.3.5 PWM frequency PRE_SCALE):
    // prescale = round((osc_clock / (4096 x rate)) - 1)
    // where osc_clock - default oscillator clock set to 25Mhz, requested rate is in Hz
    freq *= 0.95;
    uint8_t prescale = (uint8_t) floor((PCA9685_CLOCK / (4096 * freq)) - 1);
    if (prescale < 3) //Shouldn't be less than 3
        prescale = 3;

    uint8_t oldmode = this->readreg(PCA9685_MODE1); //Preserve old mode
    uint8_t newmode = (oldmode & !PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP; // Don't change any MODE1 bit except set SLEEP bit 1 and RESTART bit goes 0
    this->writereg(PCA9685_MODE1, newmode); // go to sleep, shutting the oscillator off
    this->writereg(PCA9685_PRESCALE, prescale); // set the prescaler
    this->writereg(PCA9685_MODE1, oldmode);
    osalThreadSleepMicroseconds(500); // Wait 0.5ms for oscillator stabilisation
    this->writereg(PCA9685_MODE1, oldmode | PCA9685_MODE1_ALLCALL | PCA9685_MODE1_AI | PCA9685_MODE1_RESTART);
    this->pwm_frequency = freq;
}

float PCA9685::getFreq(void) {
    return this->pwm_frequency;
}

void PCA9685::setChannel(uint16_t channel) {
    this->pwm_channel = channel;
}

void PCA9685::setPWM(float duty) {
    float pulsetime;
    uint16_t pulsewidth, on=0, off=0;
    uint32_t currentpwm;


    pulsetime = ((1.0/this->pwm_frequency) / 100.0)*duty;
    pulsewidth = floor(pulsetime / ((1.0/this->pwm_frequency) / 4096.0));

    currentpwm = this->getPWM(this->pwm_channel);
    on = currentpwm & 0x0000ffff;
    off = on + pulsewidth;

    this->setPWM(this->pwm_channel, on, off);

}

/**
 * Set PWM period for given channel.
 * @param uint8_t channel [0-15] Channel number. 0 to 15
 * @param uint16_t on - internal chip counter should count to this value, to turn channel on
 * @param uint16_t off - internal chip counter should count to this value, to turn channel off
 *
 * Both on and off parameters don't say anything about real period which depend on current frequency.
 * Thanks to, you can overlap on and off values or even set different phase shift on each
 * channel independently.
 */
void PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off) {

  this->pwm_channel = channel;
  this->txbuff[0] = LED0_ON_L+4*channel;
  this->txbuff[1] = on & 0x00FF;
  this->txbuff[2] = on >> 8;
  this->txbuff[3] = off & 0x00FF;
  this->txbuff[4] = off >> 8;
  this->tmo = MS2ST(4);

  if (this->acquire) i2cAcquireBus(this->driver);
  this->status = i2cMasterTransmitTimeout(this->driver, this->i2caddres, this->txbuff, 5, this->rxbuff, 0, this->tmo);
  if (this->acquire) i2cReleaseBus(this->driver);
}

/**
 * Set PWM for more than one channel. All channels in given range will be changed
 * @param uint8_t channel [0-15] Starting channel number.
 * @param const uint16_t *on Array of values of "on" for subsequent channel
 * @param const uint16_t *off Array of values of "off" for subsequent channel
 * @param uint8_t count Number of channels to set.
 *
 * You can use this function to setup PWM for more than one channel at once.
 * The function will set auto increment bit in MODE1. If sum of counter and channel
 * is greater than 15, it will overwrite PWM of overlapping channels.
 *
 */
void PCA9685::setPWM(uint8_t channel, const uint16_t *on, const uint16_t *off, uint8_t count) {
    uint16_t txbuff[33];

    if (count>15)
        count = 15;
    uint8_t j=0;
    txbuff[0] = LED0_ON_L+4*channel;
    for (uint8_t i=1; i<(16 - count); i+=2) {
        txbuff[i] = on[j];
        txbuff[i+1] = off[j+1];
        j++;
    }
    this->tmo = MS2ST(4);
    this->pwm_channel = channel;

    uint8_t oldmode = this->readreg(PCA9685_MODE1); //Preserve old mode
    this->writereg(PCA9685_MODE1, oldmode | PCA9685_MODE1_AI); //Set auto increment
    this->status = i2cMasterTransmitTimeout(this->driver, this->i2caddres, (uint8_t*) txbuff, (count+1) * sizeof(uint8_t), this->rxbuff, 0, this->tmo);
    this->writereg(PCA9685_MODE1, oldmode); //Restore old mode
}

uint8_t PCA9685::getPWM(void) {
    return this->pwm_duty;
}

/**
 * Set multiple channels at once. Only channels given in set will be changed all other will be preserved.
 *
 * @param const PWMSet *set array of PWMSet structures
 * @param uint8_t length of set array
 */
void PCA9685::burstPWM(const PWMSet *set, uint8_t length){
    uint32_t onoff[17];
    uint8_t i, *txbuff;

    onoff[0] = LED0_ON_L << 24;//Move starting channel number to most significant byte of first index
    for (i=1; i<17; i++) {
        onoff[i] = this->getPWM(i);
    }

    for (i=0; i<(length < 16 ? length : 16); i++) {
        onoff[set[i].channel + 1] = set[i].off << 16 | set[i].on;
    }

    txbuff = (uint8_t *) onoff;
    txbuff += 3; //Transmission buffer points to the first channel

    uint8_t oldmode = this->readreg(PCA9685_MODE1); //Preserve old mode
    this->writereg(PCA9685_MODE1, oldmode | PCA9685_MODE1_AI); //Set auto increment
    this->status = i2cMasterTransmitTimeout(this->driver, this->i2caddres, (uint8_t*) txbuff, 33, this->rxbuff, 0, this->tmo);
    this->writereg(PCA9685_MODE1, oldmode); //Restore old mode
}


/**
 * Returns PWM value for given channel
 * @param uint8_t channel
 *
 * @return uint32_t upper half of it is set to OFF register value for given channel, lower half is set for ON register value
 *
 */
uint32_t PCA9685::getPWM(uint8_t channel) {
    uint32_t pwm = 0;

    this->setChannel(channel);
    pwm = (this->readreg((LED0_ON_L+4*channel) + 2) & 0x0F) << 8;
    pwm |= this->readreg((LED0_ON_L+4*channel) + 3);
    pwm = pwm << 16; //Shift all by 16 bits, first half contains on period counter
    pwm |= (this->readreg((LED0_ON_L+4*channel) + 1) & 0x0F) << 8;
    pwm |= this->readreg(LED0_ON_L+4*channel); //The other half will contain off period counter

    return pwm;
}

/*
 * Write byte to register
 */
void PCA9685::writereg(uint8_t reg, uint8_t data) {
    this->txbuff[0] = reg;
    this->txbuff[1] = data;
    this->tmo = MS2ST(4);

    if (this->acquire) i2cAcquireBus(this->driver);
    this->status = i2cMasterTransmitTimeout(this->driver, this->i2caddres, this->txbuff, 2, this->rxbuff, 0, this->tmo);
    if (this->acquire) i2cReleaseBus(this->driver);
}

/*
 * Read data from chip register
 */
uint8_t PCA9685::readreg(uint8_t reg) {
    if (reg < 70 || reg > 249)
    {
        this->tmo = MS2ST(4);
        if (this->acquire) i2cAcquireBus(this->driver);
        this->status = i2cMasterTransmitTimeout(this->driver, this->i2caddres, &reg, 1, this->rxbuff, 1, this->tmo);
        if (this->acquire) i2cReleaseBus(this->driver);
    }
    return this->rxbuff[0];
}

/**
 * Returns given register value. If register number is one of LEDx_ON_L or LEDx_OFF_L it will return value of
 * L byte as first and H byte as second one.
 * @param uint8_t reg - register number
 * @return uint16_t - register value
 */
uint16_t PCA9685::getRegisterValue(uint8_t reg)
{
    uint16_t res = 0;
    //Led on/off registers are four 8 bit registers for Low and High bits
    if ((reg >=6 && reg <= 68) && ((reg-6) % 2)==0)
    {
        res = (this->readreg(reg + 1) & 0x0F) << 8;
        res |= this->readreg(reg);
        return res;
    }
    res = this->readreg(reg);
    return res;

}

/**
 * Status of last I2C operation
 * @return i2cflags_t - status
 */
i2cflags_t PCA9685::getStatus() {
    if (this->status == MSG_RESET)
        return i2cGetErrors(this->driver);
    else if (this->status == MSG_TIMEOUT)
        return I2C_TIMEOUT;
    return I2C_NO_ERROR;
}

/**
 * Change chip address. You can change I2C addres.
 * @param uint8_t address
 */
uint8_t PCA9685::setAddress(uint8_t address) {
    uint8_t t;
    t = this->i2caddres;
    this->i2caddres = address;
    return t;
}

/**
 * Returns current address of the PCA9685
 * @return uint8_t address
 */
uint8_t PCA9685::getAddress() {
    return this->i2caddres;
}

/**
 * Acquire I2C bus exclusively during transmit
 * @param uint8_t exclusive - true for making I2C bus exclusive for a thread, false if otherwise
 */
void PCA9685::acquireBus(uint8_t exclusive) {
    this->acquire = exclusive;
}
