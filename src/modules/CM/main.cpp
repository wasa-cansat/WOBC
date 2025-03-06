// Simplified pressure sender implementation using the correct WOBC API

#include <Arduino.h>
#include <library/wobc.h>

// Settings
constexpr uint8_t module_id = 0x4d;
constexpr uint8_t unit_id = 0x43;

// Core components
core::CANBus can_bus(44, 43);
core::SerialBus serial_bus(Serial);

// Status indicators
interface::WatchIndicator<unsigned> status_indicator(42, kernel::packetCount());
interface::WatchIndicator<unsigned> error_indicator(41, kernel::errorCount());

// Basic sender component
class PressureSender : public process::Component {
public:
  static constexpr uint8_t component_id = 0x10;
  static constexpr uint8_t telemetry_id = 0x01;
  
  PressureSender() : process::Component("PressureSender", component_id), 
                    timer_(*this) {}
  
  void begin() {
    process::Component::begin();
    start(timer_);
  }

private:
  // Data sender timer
  class DataTimer : public process::Timer {
  public:
    static constexpr unsigned interval_ms = 1000; // 1 second interval
    
    DataTimer(PressureSender& sender) : process::Timer("DataTimer", interval_ms), sender_(sender) {}
    
  protected:
    void callback() override {
      // Generate mock values
      float temperature = 25.0f + (float)random(-50, 51) / 10.0f;
      float pressure = 1013.25f + (float)random(-100, 101) / 10.0f;
      float altitude = 50.0f + (float)random(-200, 201) / 10.0f;
      float humidity = 45.0f + (float)random(-50, 51) / 10.0f;
      
      // Create packet - using the correct API
      wcpp::Packet packet = sender_.newPacket(64); // Allocate enough space for our data
      
      // Fix: Set packet headers properly using component_id constant
      packet.telemetry(telemetry_id, PressureSender::component_id);
      
      // Fix: Set values using setFloat32 to match receiver's getFloat32
      auto temp_entry = packet.append("Tmp");
      temp_entry.setFloat32(temperature);
      
      auto press_entry = packet.append("Prs");
      press_entry.setFloat32(pressure);
      
      auto alt_entry = packet.append("Alt");
      alt_entry.setFloat32(altitude);
      
      auto hum_entry = packet.append("Hum");
      hum_entry.setFloat32(humidity);
      
      // Send the packet
      sender_.sendPacket(packet);
      
      // Log the data
      sender_.LOG("\nSent: T=%.1f°C, P=%.1fhPa, A=%.1fm, H=%.1f%%\n",
          temperature, pressure, altitude, humidity);
          
      // Debug output
      // Serial.println("=== Sent Pressure Data ===");
      // Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
      // Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
      // Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m"); 
      // Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
      // Serial.println("==========================");
    }
    
    PressureSender& sender_;
  };
  
  DataTimer timer_;
};

// Instance of the pressure sender
PressureSender pressure_sender;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\n\nCM Module Starting...");
  
  // Initialize random
  randomSeed(analogRead(0));
  
  // Initialize kernel with default module behavior
  if (!kernel::begin(module_id)) return;
  
  // Initialize indicators
  status_indicator.begin();
  error_indicator.begin();
  
  // Initialize buses
  can_bus.begin();
  serial_bus.begin();
  
  // Start sender component
  pressure_sender.begin();
  
  Serial.println("CM Module Ready");
}

void loop() {
  // Update indicators
  status_indicator.update();
  error_indicator.update();
}