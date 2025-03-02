#define DEBUG_BME280

#include <library/wobc.h>
#include <components/Pressure/Pressure.h>

// Settings
constexpr uint8_t module_id = 0x4d;
constexpr uint8_t unit_id = 0x43;

// Core
core::SerialBus serial_bus(Serial);

interface::WatchIndicator<unsigned> status_indicator(42, kernel::packetCount());
interface::WatchIndicator<unsigned> error_indicator(41, kernel::errorCount());

// Components
component::Pressure pressure(Wire, unit_id);

void setup() {
  Serial.begin(115200);
  Serial.println("CM Module initialized");

  Wire.begin(17, 16);

}

void loop() {


}