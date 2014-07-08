#ifndef MCP3428_H
#define MCP3428_H
/**
 * @file
 * @author Nick Østergaard
 *
 * @section DESCRIPTION
 * This is the header file for the MCP3428 interface used for
 * the battery monitor (BM).
 */

#define MCP3428addr 0xD0  // General ADC address
#define BANK1       0xD8  // Starboard connector
#define BANK2       0xD4  // Port connector

/* Function prototypes */

/**
 @brief MCP3428 general call reset

 @param   bank (BANK1 or BANK2)
 @retval  0 device accessible
 @retval  1 failed to access device
 */
uint8_t mcp_general_call_reset(uint8_t bank);

/**
 @brief Read ADC channel

 @param   bank (BANK1 or BANK2) and channel number (1, 2, 3 or 4)
 @retval  measured value as int16_t
 */
int16_t mcp_read(uint8_t bank, uint8_t ch);

#endif // MCP3428_H 
