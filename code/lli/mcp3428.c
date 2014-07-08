/** 
 *  @defgroup sensor MCP3428
 *  @code #include <mcp3428.h> @endcode
 * 
 *  @brief MCP3428 interface functions
 *
 *  This is used to measure the battery voltage on the main packs
 *
 *  @author Nick Østergaard nickoe@es.aau.dk
 */
#include <stdio.h>
#include "i2cmaster.h"
#include "mcp3428.h"

uint8_t mcp_general_call_reset(uint8_t bank) {
  uint8_t s = 0;                         // status indicator
  s = i2c_start(bank+I2C_WRITE);     // set device address and write mode
  i2c_write(0x06);                   // issue general call reset (see datasheet for magic number)
  i2c_stop();                        // set stop conditon = release bus
  return s;
}

/**
 @brief Read ADC channel

 @param   bank (1 or 2) and channel number (1, 2, 3 or 4)
 @retval  measured value as int16_t
 */
int16_t mcp_read(uint8_t bank, uint8_t ch) {
  uint8_t adcupper = 0;
  uint8_t adclower = 0;

  // config
  i2c_start(bank+I2C_WRITE); // 0xD8 is the starboard connector (Bank 1)
  i2c_write(0b10001000);

  // MCP3428 read data addr 0x100
  i2c_start(bank+I2C_READ);      // set adc address for reading
  adcupper = i2c_readAck();      // read 2nd byte (upper data byte)
  adclower = i2c_readNak();      // read 3rd byte (lower data byte)
  i2c_stop();                    // set stop conditon = release bus

  return (adcupper << 8 | adclower);
}



