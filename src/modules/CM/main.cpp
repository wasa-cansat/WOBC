#define DEBUG_BME280

#include <library/wobc.h>
#include <components/Pressure/Pressure.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

// Settings
constexpr uint8_t module_id = 0x4d;
constexpr uint8_t unit_id = 0x43;

// Core
core::SerialBus serial_bus(Serial);

interface::WatchIndicator<unsigned> status_indicator(42, kernel::packetCount());
interface::WatchIndicator<unsigned> error_indicator(41, kernel::errorCount());

// Components
component::Pressure pressure(Wire, unit_id);

// Direct sensor access for debugging
Adafruit_BME280 bme;

// Debug variables
unsigned long lastDebugTime = 0;
const unsigned long debugInterval = 1000; // 1 second interval for debug output

void setup() {
  Serial.begin(115200);
  Serial.println("CM Module initialized");

  Wire.begin(17, 16);
  
  // Initialize the pressure component
  pressure.begin();
  Serial.println("Pressure component started");
  
  // Direct sensor initialization for debugging
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  } else {
    Serial.println("BME280 sensor initialized successfully");
  }
}

void loop() {
  // Let the pressure component run internally
  
  // Debug output for pressure readings
  unsigned long currentTime = millis();
  if (currentTime - lastDebugTime > debugInterval) {
    lastDebugTime = currentTime;
    
    Serial.println("=== Pressure Sensor Data ===");
    
    // Read directly from the sensor for debugging
    float temperature = bme.readTemperature();
    float pressure_hPa = bme.readPressure() / 100.0F;
    float altitude_m = bme.readAltitude(1013.25); // Standard pressure
    float humidity = bme.readHumidity();
    
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C");
    
    Serial.print("Pressure: ");
    Serial.print(pressure_hPa);
    Serial.println(" hPa");
    
    Serial.print("Approximate altitude: ");
    Serial.print(altitude_m);
    Serial.println(" m");
    
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    
    Serial.println("==========================");
  }
}