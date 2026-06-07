/**
 * AuriolDecoder - 433MHz Auriol sensor decoder library
 * 
 * Decodes certain Auriol temperature (e.g. IAN 375672_2104, here labelled as "Type 1") and 
 * temperature/humidity (e.g. IAN 384583_2107, here labelled "Type 2") sensors.
 */

#ifndef AURIOL_DECODER_H
#define AURIOL_DECODER_H

#include <stdint.h>

/**
 * Callback signature for decoded sensor packets
 * 
 * Parameters:
 *   sensor_id      - Sensor ID (8-bit value, range 0x00-0xFF)
 *   channel        - Sensor channel (0 for Type 1; 1,2 or 3 for Type 2)
 *   temperature_c  - Temperature in degrees Celsius (float)
 *   humidity_pct   - Humidity percentage (0-100), or 0 for Type 1 sensors
 *   battery_ok     - Battery status (true = OK, false = Low)
 */
typedef void (*AuriolDecoderCallback)(uint16_t sensor_id, uint8_t channel, float temperature_c, uint8_t humidity_pct, bool battery_ok);

/**
 * Initialize the Auriol decoder
 * 
 * Sets up the ISR handler on the specified RF data pin and registers the callback
 * for decoded sensor packets.
 * 
 * Parameters:
 *   rf_data_pin - GPIO pin number for RF data input (e.g., GPIO2 for ESP-01S)
 *   callback    - Function pointer to call when a valid packet is decoded
 * 
 * Note: Caller is responsible for setting up RF power control (module-specific)
 */
void auriolDecoderSetup(uint8_t rf_data_pin, AuriolDecoderCallback callback);

/**
 * Process pending RF data
 * 
 * Must be called regularly in the main loop to decode captured RF pulses.
 * Performs profile detection, packet assembly, consensus validation,
 * and callback invocation.
 */
void auriolDecoderProcess(void);

/**
 * Reset decoder state
 * 
 * Clears all internal state machines, useful for diagnostics or sensor switching.
 */
void auriolDecoderReset(void);

#endif // AURIOL_DECODER_H
