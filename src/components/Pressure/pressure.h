#pragma once

#include <Adafruit_BME280.h>
#include <library/process/component.h>
#include <library/wobc.h>

namespace component {

class Pressure : public process::Component {
public:
  static constexpr uint8_t component_id = 0x10;
  static constexpr uint8_t telemetry_id = 0x01;
  
  Pressure();
  
  void begin();
  void loop() override;
  
private:
  Adafruit_BME280 bme_;
  unsigned long lastReadingTime_ = 0;
  const unsigned long readingInterval_ = 2000; // Read every 2 seconds
  
  // Environmental data
  float temperature_ = 0.0f;
  float pressure_ = 0.0f;
  float altitude_ = 0.0f;
  float humidity_ = 0.0f;
  
  bool initializeSensor();
  void readSensorData();
  void sendTelemetry();
};

} // namespace component
