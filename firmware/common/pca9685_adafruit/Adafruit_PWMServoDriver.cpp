/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_PWMServoDriver.h>

#include <math.h>

#define min(a,b) ((a)<(b)?(a):(b))

static const I2CConfig i2cCfg = {
	OPMODE_I2C,
	400000,
	FAST_DUTY_CYCLE_2,
};

Adafruit_PWMServoDriver::Adafruit_PWMServoDriver(I2CDriver* driver, uint8_t addr) {
  _driver = driver;
  _i2caddr = addr;
}

void Adafruit_PWMServoDriver::begin(void) {
 i2cStart(_driver, &i2cCfg);
 reset();
}


void Adafruit_PWMServoDriver::reset(void) {
 write8(PCA9685_MODE1, 0x0);
 osalThreadSleepMilliseconds(1);
}

void Adafruit_PWMServoDriver::setPWMFreq(float freq) {
  freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  uint8_t prescale = floor(prescaleval + 0.5);
  
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  osalThreadSleepMilliseconds(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                          // This is why the beginTransmission below was not working.
}

void Adafruit_PWMServoDriver::setPWM(uint8_t num, uint16_t on, uint16_t off) {
  uint8_t txBuf[] = {
  (uint8_t)(LED0_ON_L+4*num),
  (uint8_t)(on),
  (uint8_t)(on>>8),
  (uint8_t)(off),
  (uint8_t)(off>>8)
  };
  i2cAcquireBus(_driver);
  i2cMasterTransmitTimeout(_driver, _i2caddr, txBuf, 5, NULL, 0, OSAL_MS2ST(4));
  i2cReleaseBus(_driver);
}

// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void Adafruit_PWMServoDriver::setPin(uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  val = min(val, 4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, val);
    }
  }
}

uint8_t Adafruit_PWMServoDriver::read8(uint8_t addr) {
  uint8_t rxBuf[1];
  i2cAcquireBus(_driver);
  i2cMasterTransmitTimeout(_driver, _i2caddr, &addr, 1, rxBuf, 1, OSAL_MS2ST(4));
  i2cReleaseBus(_driver);
  return rxBuf[0];
}

void Adafruit_PWMServoDriver::write8(uint8_t addr, uint8_t d) {
  uint8_t txBuf[2];
  txBuf[0] = addr;
  txBuf[1] = d;
  i2cAcquireBus(_driver);
  i2cMasterTransmitTimeout(_driver, _i2caddr, txBuf, 2, NULL, 0, OSAL_MS2ST(4));
  i2cReleaseBus(_driver);
}
