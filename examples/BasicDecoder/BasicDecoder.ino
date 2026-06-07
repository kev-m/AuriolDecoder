/**
 * BasicDecoder - AuriolDecoder library example
 * 
 * Simple example demonstrating how to use the AuriolDecoder library
 * to receive and process Auriol 433MHz sensor packets.
 * 
 * Hardware Setup (ESP-01S):
 * - GPIO2: RF data input
 * - GPIO3: RF module power control
 * - Serial TX (GPIO1): Debug output (optional)
 */

#include <AuriolDecoder.h>

// Pin configuration
#define RF_DATA_PIN 2
#define RF_POWER_PIN 3

// Callback function - called when a valid Auriol packet is decoded
void onSensorPacket(uint16_t sensor_id, float temperature_c, float humidity_pct, bool battery_ok)
{
    // Format and display the decoded packet
    Serial.printf("[AURIOL] ID: 0x%02X | Temp: %.1f°C | Hum: %d%% | Battery: %s\n",
                  sensor_id, 
                  temperature_c, 
                  (int)humidity_pct,
                  battery_ok ? "OK" : "Low");
}

void setup()
{
    // Initialize serial for debug output
    Serial.begin(115200);
    delay(100);
    
    Serial.println("\n\n=== AuriolDecoder Basic Example ===\n");
    Serial.println("Initializing RF receiver...");
    
    // Initialize RF module power control
    pinMode(RF_POWER_PIN, OUTPUT);
    digitalWrite(RF_POWER_PIN, LOW);  // Enable RF module (active low)
    
    delay(100);  // Allow RF module to stabilize
    
    // Initialize decoder with RF data pin and callback function
    auriolDecoderSetup(RF_DATA_PIN, onSensorPacket);
    
    Serial.println("RF decoder initialized.");
    Serial.println("Listening for Auriol sensor packets...\n");
}

void loop()
{
    // Process pending RF data
    // This must be called regularly to process captured pulses
    auriolDecoderProcess();
    
    // Yield to ESP8266 core to maintain WiFi and other functions
    yield();
}
