/*!
 *  @file STM32_PWMServoDriver.h
 *
 *  This is a library for a 16-channel PWM & Servo driver using the PCA9685 for STM32.
 *
 *  These drivers use I2C to communicate, requiring 2 pins.
 *  For STM32, this will use HAL I2C functions.
 */

#ifndef _STM32_PWMServoDriver_H
#define _STM32_PWMServoDriver_H

#include "main.h"
// REGISTER ADDRESSES
// These registers are used to control different functionalities of the PCA9685 chip
#define PCA9685_MODE1 0x00      /**< Mode Register 1: Controls sleep, restart, and other basic settings */
#define PCA9685_MODE2 0x01      /**< Mode Register 2: Controls output driver mode and inversion settings */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1: Allows additional I2C addresses */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2: Allows additional I2C addresses */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3: Allows additional I2C addresses */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address: Broadcast address for controlling multiple devices */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte: Sets the low byte of the on time for PWM channel 0 */
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte: Sets the high byte of the on time for PWM channel 0 */
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte: Sets the low byte of the off time for PWM channel 0 */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte: Sets the high byte of the off time for PWM channel 0 */
// etc. All 16 channels have similar registers: LED15_OFF_H would be at address 0x45

// Global registers for controlling all PWM channels simultaneously
#define PCA9685_ALLLED_ON_L 0xFA  /**< Load all the LEDn_ON registers, low byte */
#define PCA9685_ALLLED_ON_H 0xFB  /**< Load all the LEDn_ON registers, high byte */
#define PCA9685_ALLLED_OFF_L 0xFC /**< Load all the LEDn_OFF registers, low byte */
#define PCA9685_ALLLED_OFF_H 0xFD /**< Load all the LEDn_OFF registers, high byte */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency: Controls the frequency of the PWM signal */
#define PCA9685_TESTMODE 0xFF     /**< Test mode register: Used for factory testing, typically not used by developers */

// MODE1 bits (used in PCA9685_MODE1 register)
#define MODE1_ALLCAL 0x01  /**< Respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< Respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< Respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< Respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled: Moves to the next register automatically */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock: Enables use of an external clock */
#define MODE1_RESTART 0x80 /**< Restart enabled: Allows restarting the oscillator */

// MODE2 bits (used in PCA9685_MODE2 register)
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1 0x02 /**< Active LOW output enable input - high impedance */
#define MODE2_OUTDRV 0x04 /**< Totem pole structure vs open-drain: Controls output drive mode */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP: Controls when outputs change state */
#define MODE2_INVRT 0x10  /**< Output logic state inverted: Inverts the output signal */

// Default settings for the PCA9685
#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Internal oscillator frequency in the PCA9685, used for calculating PWM frequency */

#define PCA9685_PRESCALE_MIN 3   /**< Minimum prescale value for the frequency */
#define PCA9685_PRESCALE_MAX 255 /**< Maximum prescale value for the frequency */

/*!
 *  @brief  Class that stores state and functions for interacting with the PCA9685 PWM chip.
 */
class STM32_PWMServoDriver {
public:
  /*!
   *  @brief  Constructor for the class. Initializes the driver with the I2C handle and optional address.
   *  @param  i2cHandle A pointer to the I2C handle (configured in STM32CubeMX).
   *  @param  addr The I2C address of the PCA9685 (default is 0x40).
   */
  STM32_PWMServoDriver(I2C_HandleTypeDef *i2cHandle, uint8_t addr = PCA9685_I2C_ADDRESS);

  /*!
   *  @brief  Initializes communication with the PCA9685 and sets up the PWM frequency.
   *  @param  prescale Optional prescale value to set the PWM frequency directly.
   *  @return true if initialization is successful, otherwise false.
   */
  bool begin(uint8_t prescale = 0);

  /*!
   *  @brief  Resets the PCA9685, bringing it back to its default state.
   */
  void reset();

  /*!
   *  @brief  Puts the PCA9685 into sleep mode to reduce power consumption.
   */
  void sleep();

  /*!
   *  @brief  Wakes the PCA9685 from sleep mode, allowing it to start generating PWM signals again.
   */
  void wakeup();

  /*!
   *  @brief  Sets the external clock source for the PCA9685.
   *  @param  prescale The prescale value for the external clock frequency.
   */
  void setExtClk(uint8_t prescale);

  /*!
   *  @brief  Sets the PWM frequency for the entire chip.
   *  @param  freq The desired PWM frequency in Hz.
   */
  void setPWMFreq(float freq);

  /*!
   *  @brief  Configures the output mode (totem pole or open drain).
   *  @param  totempole True for totem pole output, false for open drain.
   */
  void setOutputMode(bool totempole);

  /*!
   *  @brief  Gets the current PWM setting for a specific channel.
   *  @param  num The PWM channel (0-15).
   *  @param  off If true, returns the OFF time, otherwise the ON time.
   *  @return The current setting for the requested time (ON/OFF).
   */
  uint16_t getPWM(uint8_t num, bool off = false);

  /*!
   *  @brief  Sets the PWM signal for a specific channel.
   *  @param  num The PWM channel (0-15).
   *  @param  on The step (0-4095) when the signal turns on.
   *  @param  off The step (0-4095) when the signal turns off.
   */
  void setPWM(uint8_t num, uint16_t on, uint16_t off);

  /*!
   *  @brief  Helper function to set a PWM signal based on a single value.
   *  @param  num The PWM channel (0-15).
   *  @param  val The number of steps out of 4096 to be active (0 to 4095).
   *  @param  invert If true, inverts the output signal.
   */
  void setPin(uint8_t num, uint16_t val, bool invert = false);

  /*!
   *  @brief  Reads the current prescale value from the PCA9685.
   *  @return The prescale value.
   */
  uint8_t readPrescale(void);

  /*!
   *  @brief  Sets the PWM signal based on a duration in microseconds.
   *  @param  num The PWM channel (0-15).
   *  @param  Microseconds The duration for the PWM signal in microseconds.
   */
  void writeMicroseconds(uint8_t num, uint16_t Microseconds);

  /*!
   *  @brief  Sets the oscillator frequency used for PWM calculations.
   *  @param  freq The oscillator frequency in Hz.
   */
  void setOscillatorFrequency(uint32_t freq);

  /*!
   *  @brief  Gets the current oscillator frequency used for PWM calculations.
   *  @return The oscillator frequency in Hz.
   */
  uint32_t getOscillatorFrequency(void);

private:
  uint8_t _i2caddr; /**< Stores the I2C address of the PCA9685 */
  I2C_HandleTypeDef *_i2c; /**< Pointer to the I2C handle */
  uint32_t _oscillator_freq; /**< Stores the oscillator frequency used for PWM calculations */

  /*!
   *  @brief  Reads an 8-bit value from a specified register.
   *  @param  addr The register address to read from.
   *  @return The 8-bit value read from the register.
   */
  uint8_t read8(uint8_t addr);

  /*!
   *  @brief  Writes an 8-bit value to a specified register.
   *  @param  addr The register address to write to.
   *  @param  d The 8-bit data to write.
   */
  void write8(uint8_t addr, uint8_t d);
};

#endif
