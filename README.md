# AuriolDecoder

A 433MHz Auriol sensor decoder library for ESP8266, featuring consensus-based validation and automatic profile detection.

## Features

- **Dual Sensor Support**: Decodes both temperature-only (e.g. ) and temperature/humidity (e.g. ) Auriol sensors
- **Automatic Profile Detection**: Detects and locks onto sensor type within 12-24 samples
- **Consensus Validation**: Requires 2 identical transmissions for data acceptance, preventing single-bit corruption
- **Duplicate Suppression**: 1500ms window prevents repeated transmission of identical payloads
- **Callback-Based**: Clean, event-driven API with minimal main loop coupling

## Protocol Support

### Type 1 Sensors (32-bit payload)

For example, labelled as IAN 375672_2104.

- **Temperature Range**: -40.0°C to +60.0°C (0.1°C resolution)
- **Battery Status**: OK/Low
- **Sensor ID**: 8-bit identifier

### Type 2 Sensors (36-bit payload)

For example, labelled as IAN 384583_2107, "RC Weather station with BBQ sensor 4-LD5882".

- **Temperature Range**: 0.0°C to 409.5°C (0.1°C resolution)
- **Humidity Range**: 0-100%
- **Channels**: 1-4 (multi-channel support)
- **Battery Status**: OK/Low
- **Sensor ID**: 12-bit identifier

## Installation

### PlatformIO

Add to `platformio.ini`:
```ini
lib_deps =
    AuriolDecoder@^1.0.0
```

### Manual

1. Clone the repository into your `lib/` directory
2. Include in your project with `#include <AuriolDecoder.h>`

## Quick Start

```cpp
#include <AuriolDecoder.h>

// Mandatory: Required by the library
#define RF_DATA_PIN 2

// Optional: Project Specific
#define RF_POWER_PIN 3

// Callback invoked when sensor packet is decoded
void onSensorPacket(uint16_t sensor_id, float temp, float humidity, bool battery_ok) {
  Serial.printf("Sensor 0x%02X: %.1f°C, %d%% Humidity, Battery: %s\n",
    sensor_id, temp, (int)humidity, battery_ok ? "OK" : "Low");
}

void setup() {
  Serial.begin(115200);
  
  // Initialize RF power control (module-specific)
  pinMode(RF_POWER_PIN, OUTPUT);
  digitalWrite(RF_POWER_PIN, LOW);  // Enable RF module
  
  // Initialize decoder
  auriolDecoderSetup(RF_DATA_PIN, onSensorPacket);
}

void loop() {
  // Process RF data - call regularly in main loop
  auriolDecoderProcess();
  yield();  // Yield to ESP8266 core tasks
}
```

## API Reference

### `void auriolDecoderSetup(uint8_t rf_data_pin, AuriolDecoderCallback callback)`

Initialize the decoder with RF data pin and packet callback.

**Parameters:**
- `rf_data_pin`: GPIO pin number for RF data input (typically GPIO2 for ESP-01S)
- `callback`: Function pointer invoked when valid packet is decoded

**Note:** Caller is responsible for RF module power control (GPIO pin management).

### `void auriolDecoderProcess(void)`

Process pending RF pulses from ISR queue. Must be called regularly in main loop.

Performs:
- Profile lock accumulation
- Symbol classification
- Packet assembly
- Consensus validation
- Callback invocation

### `void auriolDecoderReset(void)`

Reset internal decoder state (optional). Useful for diagnostics or sensor switching.

### Callback Signature

```cpp
void callback(uint16_t sensor_id, uint8_t channel, float temperature_c, float humidity_pct, bool battery_ok)
```

**Parameters:**
- `sensor_id`: Sensor identifier
- `channel`: Sensor channel (0 for type 1; 1,2 or 3 for type 2)
- `temperature_c`: Temperature in degrees Celsius
- `humidity_pct`: Humidity percentage (0-100, or 0 for Type 1 sensors)
- `battery_ok`: Battery status (true = OK, false = Low)

**NOTE:** The `sensor_id` changes every time the sender is restarted, e.g. after changing the battery.

## Hardware Setup (ESP-01S Example)

| Function | GPIO | Purpose |
|----------|------|---------|
| RF Data | GPIO2 | Connects to 433MHz receiver data line |
| RF Power | GPIO3 | Enables/disables receiver module (LOW=on) |
| Serial TX | GPIO1 | Debug output (optional) |

## Performance

- **Memory**: ~2KB RAM (state machine + ISR queue)
- **CPU Overhead**: <5% with typical sensor update rate (~10 sec intervals)
- **Latency**: ~1 second (waiting for 2nd transmission consensus)
- **Reception Range**: 30-100m outdoors, 5-20m indoors

## Technical Details

### Modulation
- **Frequency**: 433 MHz
- **Encoding**: On-Off Keying (OOK) with pulse-width modulation
- **Symbol Classification**: Duration-based (short/long pulse detection)

### Type 1 RF Timing
- Bit 0: ~2400 µs (±20% tolerance)
- Bit 1: ~4400 µs (±20% tolerance)

### Payload Structure

```c
uint8_t b0 = (raw32 >> 24) & 0xFF;  // Sensor ID + Battery
uint8_t b1 = (raw32 >> 16) & 0xFF;  // Temperature (mixed) + ???
uint8_t b2 = (raw32 >> 8) & 0xFF;   // Temperature (complete)
uint8_t b3 = raw32 & 0xFF;          // Checksum
```

| Byte | Bits | Assignment | Status |
|------|------|------------|--------|
| b0[7] | Battery flag | Assigned |  |
| b0[6:0] | Sensor ID (7 bits) | Assigned |  |
| **b1[7:4]** | **High nibble** | **UNASSIGNED** | 4 bits unused |
| b1[3:0] | Temperature (upper 4 bits) | Assigned |  |
| b2[7:0] | Temperature (lower 8 bits) | Assigned |  |
| b3[7:0] | Checksum | Assigned (unchecked) |  |

**Unassigned: 4 bits in b1 high nibble** (purpose unknown—possibly reserved or manufacturer-specific)


### Type 2 RF Timing
- Bit 0: ~1400 µs (±20% tolerance)
- Bit 1: ~2400 µs (±20% tolerance)

### Payload Structure

```c
uint8_t n0 = (raw36 >> 32) & 0x0F;  // Sensor ID upper
uint8_t n1 = (raw36 >> 28) & 0x0F;  // Sensor ID lower
uint8_t n2 = (raw36 >> 24) & 0x0F;  // Control byte
uint8_t n3 = (raw36 >> 20) & 0x0F;  // Temp upper
uint8_t n4 = (raw36 >> 16) & 0x0F;  // Temp middle
uint8_t n5 = (raw36 >> 12) & 0x0F;  // Temp lower
uint8_t n7 = (raw36 >> 4) & 0x0F;   // Humidity upper
uint8_t n8 = raw36 & 0x0F;          // Humidity lower
// NOTE: n6 is COMPLETELY SKIPPED
```

| Nibble | Bits | Assignment | Status |
|--------|------|------------|--------|
| n0 | Sensor ID upper | Assigned |  |
| n1 | Sensor ID lower | Assigned |  |
| n2[3] | Battery flag | Assigned |  |
| n2[2] | **Unused** | **UNASSIGNED** | 1 bit unknown |
| n2[1:0] | Channel offset (0-2) | Assigned |  |
| n3[3:0] | Temperature upper | Assigned |  |
| n4[3:0] | Temperature middle | Assigned |  |
| n5[3:0] | Temperature lower | Assigned |  |
| **n6[3:0]** | **SKIPPED ENTIRELY** | **UNASSIGNED** | 4 bits not extracted |
| n7[3:0] | Humidity upper | Assigned |  |
| n8[3:0] | Humidity lower | Assigned |  |

**Unassigned: n2 bit 2 (1 bit) + complete n6 nibble (4 bits) = 5 total bits**


### Profile Locking
1. First 12-24 symbols compared against Type 1 and Type 2 timing templates
2. Profile with lower accumulated error selected
3. Lock maintained until 3 consecutive gaps (no sensor activity)

### Consensus Model
- Identical packets counted across transmissions
- Decoded only on 2nd identical transmission (prevents corruption)
- 1500ms duplicate suppression window per (profile, payload) tuple

## Known Limitations

- No checksum validation (relies on consensus for integrity)
- Channel information not exposed in callback for Type 2 sensors
- Type 1 battery reporting is binary (OK/Low only)
- New sensors require 2 consecutive transmissions before first report

## Debugging

To monitor decoder state or disable automatic features:
1. Call `auriolDecoderReset()` to clear state
2. Implement callback logging for payload tracking
3. Use Serial output to verify sensor reception

## License

Licensed under the GNU General Public License. See LICENSE file for details.

## Contributing

Contributions welcome! Please submit issues and pull requests to the GitHub repository.

## Support

For issues, feature requests, or protocol documentation, visit:
https://github.com/kev-m/AuriolDecoder
