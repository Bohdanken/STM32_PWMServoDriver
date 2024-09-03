#include "STM32_PWMServoDriver.h"

/*!
 *  @brief  Constructor to initialize the I2C and address.
 */
STM32_PWMServoDriver::STM32_PWMServoDriver(I2C_HandleTypeDef *i2cHandle, uint8_t addr)
    : _i2caddr(addr), _i2c(i2cHandle) {}

/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale
 *  Sets External Clock (Optional)
 *  @return true if successful, otherwise false
 */
bool STM32_PWMServoDriver::begin(uint8_t prescale) {
  reset();

  // set the default internal frequency
  setOscillatorFrequency(FREQUENCY_OSCILLATOR);

  if (prescale) {
    setExtClk(prescale);
  } else {
    // set a default frequency
    setPWMFreq(1000);
  }

  return true;
}

/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
void STM32_PWMServoDriver::reset() {
  write8(PCA9685_MODE1, MODE1_RESTART);
  HAL_Delay(10);
}

/*!
 *  @brief  Puts board into sleep mode
 */
void STM32_PWMServoDriver::sleep() {
  uint8_t awake = read8(PCA9685_MODE1);
  uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
  write8(PCA9685_MODE1, sleep);
  HAL_Delay(5); // wait until cycle ends for sleep to be active
}

/*!
 *  @brief  Wakes board from sleep
 */
void STM32_PWMServoDriver::wakeup() {
  uint8_t sleep = read8(PCA9685_MODE1);
  uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
  write8(PCA9685_MODE1, wakeup);
}

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
void STM32_PWMServoDriver::setExtClk(uint8_t prescale) {
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep, turn off internal oscillator

  // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
  // use the external clock.
  write8(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

  write8(PCA9685_PRESCALE, prescale); // set the prescaler

  HAL_Delay(5);
  // clear the SLEEP bit to start
  write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
void STM32_PWMServoDriver::setPWMFreq(float freq) {
  if (freq < 1)
    freq = 1;
  if (freq > 3500)
    freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

  float prescaleval = ((_oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN)
    prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX)
    prescaleval = PCA9685_PRESCALE_MAX;
  uint8_t prescale = (uint8_t)prescaleval;

  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  write8(PCA9685_MODE1, newmode);                             // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  HAL_Delay(5);
  // This sets the MODE1 register to turn on auto increment.
  write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either open drain or push-pull / totem pole.
 *  Warning: LEDs with integrated zener diodes should only be driven in open-drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
void STM32_PWMServoDriver::setOutputMode(bool totempole) {
  uint8_t oldmode = read8(PCA9685_MODE2);
  uint8_t newmode;
  if (totempole) {
    newmode = oldmode | MODE2_OUTDRV;
  } else {
    newmode = oldmode & ~MODE2_OUTDRV;
  }
  write8(PCA9685_MODE2, newmode);
}

/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
uint8_t STM32_PWMServoDriver::readPrescale(void) {
  return read8(PCA9685_PRESCALE);
}

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  off If true, returns PWM OFF value, otherwise PWM ON
 *  @return requested PWM output value
 */
uint16_t STM32_PWMServoDriver::getPWM(uint8_t num, bool off) {
  uint8_t reg = PCA9685_LED0_ON_L + 4 * num;
  if (off)
    reg += 2;
  uint8_t buffer[2];
  HAL_I2C_Mem_Read(_i2c, _i2caddr << 1, reg, 1, buffer, 2, HAL_MAX_DELAY);
  return uint16_t(buffer[0]) | (uint16_t(buffer[1]) << 8);
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
void STM32_PWMServoDriver::setPWM(uint8_t num, uint16_t on, uint16_t off) {
  uint8_t buffer[5];
  buffer[0] = PCA9685_LED0_ON_L + 4 * num;
  buffer[1] = on;
  buffer[2] = on >> 8;
  buffer[3] = off;
  buffer[4] = off >> 8;
  HAL_I2C_Master_Transmit(_i2c, _i2caddr << 1, buffer, 5, HAL_MAX_DELAY);
}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
 * on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *   @param  num One of the PWM output pins, from 0 to 15
 *   @param  val The number of ticks out of 4096 to be active, should be a value
 * from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
void STM32_PWMServoDriver::setPin(uint8_t num, uint16_t val, bool invert) {
  val = (val > 4095) ? 4095 : val;
  if (invert) {
    if (val == 0) {
      setPWM(num, 4096, 0);
    } else if (val == 4095) {
      setPWM(num, 0, 4096);
    } else {
      setPWM(num, 0, 4095 - val);
    }
  } else {
    if (val == 4095) {
      setPWM(num, 4096, 0);
    } else if (val == 0) {
      setPWM(num, 0, 4096);
    } else {
      setPWM(num, 0, val);
    }
  }
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
void STM32_PWMServoDriver::writeMicroseconds(uint8_t num, uint16_t Microseconds) {
  double pulse = Microseconds;
  double pulselength;
  pulselength = 1000000; // 1,000,000 us per second

  // Read prescale
  uint16_t prescale = readPrescale();

  prescale += 1;
  pulselength *= prescale;
  pulselength /= _oscillator_freq;

  pulse /= pulselength;

  setPWM(num, 0, pulse);
}

/*!
 *  @brief  Getter for the internally tracked oscillator used for freq
 * calculations
 *  @returns The frequency the PCA9685 thinks it is running at (it cannot
 * introspect)
 */
uint32_t STM32_PWMServoDriver::getOscillatorFrequency(void) {
  return _oscillator_freq;
}

/*!
 *  @brief Setter for the internally tracked oscillator used for freq
 * calculations
 *  @param freq The frequency the PCA9685 should use for frequency calculations
 */
void STM32_PWMServoDriver::setOscillatorFrequency(uint32_t freq) {
  _oscillator_freq = freq;
}

/******************* Low level I2C interface */
uint8_t STM32_PWMServoDriver::read8(uint8_t addr) {
  uint8_t buffer[1];
  HAL_I2C_Mem_Read(_i2c, _i2caddr << 1, addr, 1, buffer, 1, HAL_MAX_DELAY);
  return buffer[0];
}

void STM32_PWMServoDriver::write8(uint8_t addr, uint8_t d){
  uint8_t buffer[2] = {addr, d};
  HAL_I2C_Master_Transmit(_i2c, _i2caddr << 1, buffer, 2, HAL_MAX_DELAY);
}
