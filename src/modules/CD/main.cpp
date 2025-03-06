// Pressure data receiver implementation using the WOBC API

#include <Arduino.h>
#include <library/wobc.h>

// Settings
constexpr uint8_t module_id = 0x44; // 'D' in ASCII
constexpr uint8_t unit_id = 0x43;   // 'C' in ASCII (same unit as CM)

// Core components
core::CANBus can_bus(44, 43);
core::SerialBus serial_bus(Serial);

// Status indicators
interface::WatchIndicator<unsigned> status_indicator(42, kernel::packetCount());
interface::WatchIndicator<unsigned> error_indicator(41, kernel::errorCount());

// Pressure Receiver Component
class PressureReceiver : public process::Component {
public:
  static constexpr uint8_t component_id = 0x20;
  static constexpr uint8_t listener_queue_size = 8;
  static constexpr uint8_t target_packet_id = 0x22;  // Based on observed packets

  PressureReceiver() : process::Component("PressureReceiver", component_id),
                        display_timer_(*this) {}

  void begin() {
    process::Component::begin();
    
    // Configure the listener specifically for packet ID 0x22
    pressure_listener_.packet(target_packet_id);
    
    // Start listener with queue size
    listen(pressure_listener_, listener_queue_size);
    
    // Start timer to display received data stats
    start(display_timer_);
    
    Serial.println("\n---------------------------------------");
    Serial.println("PressureReceiver started successfully");
    Serial.println("Listening for packet ID 0x22");
    Serial.println("---------------------------------------\n");
  }

private:
  // Packet listener
  Listener pressure_listener_;

  // Data values
  float last_temperature = 0.0f;
  float last_pressure = 0.0f;
  float last_altitude = 0.0f;
  float last_humidity = 0.0f;
  unsigned packet_count = 0;
  unsigned long last_received = 0;

  // Helper method to dump packet bytes for debugging
  void dumpPacketData(const wcpp::Packet& packet) {
    Serial.print("Packet data (hex): ");
    const uint8_t* buffer = packet.getBuf();
    for (size_t i = 0; i < packet.size(); i++) {
      if (buffer[i] < 16) Serial.print("0");  // Add leading zero
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  void loop() override {
    // Process any received packets
    while (pressure_listener_) {
      const wcpp::Packet& packet = pressure_listener_.pop();
      
      // Record reception time and count
      last_received = millis();
      packet_count++;
      
      // Clear debug line for better readability
      Serial.println("\n--- PACKET RECEIVED ---");
      Serial.print("ID: 0x"); Serial.print(packet.packet_id(), HEX);
      Serial.print(", Component: 0x"); Serial.print(packet.component_id(), HEX);
      Serial.print(", Size: "); Serial.println(packet.size());
      
      // Display raw packet data for debugging
      dumpPacketData(packet);
      
      // Try to read data directly from byte positions instead of using keys
      bool data_extracted = false;
      
      // For a packet with ID 0x22 and size 7, let's try to interpret the bytes directly
      // The format appears to be a specific binary structure
      if (packet.packet_id() == target_packet_id && packet.size() >= 7) {
        // Extract data based on byte positions
        const uint8_t* bytes = packet.getBuf();
        bool valid_data = (bytes != nullptr);
        
        if (valid_data) {
          // Extract values based on the specific binary format
          // This is an example approach - adjust based on your protocol
          
          // Different approach to extract data - treat as two 16-bit values and one 8-bit value
          int16_t temp_raw = (bytes[1] << 8) | bytes[0];
          int16_t press_raw = (bytes[3] << 8) | bytes[2];
          uint8_t hum_raw = bytes[4];
          
          // Convert to actual values (scale factors are examples)
          last_temperature = temp_raw / 10.0f;   // Example: 10 bits resolution
          last_pressure = press_raw / 10.0f;     // Example: 10 bits resolution
          last_humidity = hum_raw / 2.0f;        // Example: 2% resolution
          last_altitude = 44330.0f * (1 - pow((last_pressure / 1013.25f), 0.1903f)); // Calculate altitude
          
          data_extracted = true;
          
          Serial.println("DATA EXTRACTED:");
          Serial.print("Raw values: temp="); Serial.print(temp_raw);
          Serial.print(", press="); Serial.print(press_raw);
          Serial.print(", hum="); Serial.println(hum_raw);
          
          Serial.println("DATA SUMMARY:");
          Serial.print("T="); Serial.print(last_temperature); Serial.print("°C, ");
          Serial.print("P="); Serial.print(last_pressure); Serial.print("hPa, ");
          Serial.print("A="); Serial.print(last_altitude); Serial.print("m, ");
          Serial.print("H="); Serial.print(last_humidity); Serial.println("%");
        }
      }
      
      if (!data_extracted) {
        Serial.println("FAILED to extract data from packet - incompatible format!");
      }
      
      Serial.println("---------------------");
    }
  }

  // Display timer for periodic info display
  class DisplayTimer : public process::Timer {
  public:
    static constexpr unsigned interval_ms = 10000; // 10 second interval - increased to reduce output
    
    DisplayTimer(PressureReceiver& receiver) : process::Timer("DisplayTimer", interval_ms), receiver_(receiver) {}
    
  protected:
    void callback() override {
      // Calculate time since last packet
      unsigned long delta = millis() - receiver_.last_received;
      
      // Display status information
      Serial.println("\n============ STATUS REPORT ============");
      Serial.print("Total packets received: "); Serial.println(receiver_.packet_count);
      
      if (receiver_.last_received > 0) {
        Serial.print("Last received: "); Serial.print(delta / 1000.0f); Serial.println(" seconds ago");
        Serial.print("Current values: ");
        Serial.print("T="); Serial.print(receiver_.last_temperature); Serial.print("°C, ");
        Serial.print("P="); Serial.print(receiver_.last_pressure); Serial.print("hPa, ");
        Serial.print("A="); Serial.print(receiver_.last_altitude); Serial.print("m, ");
        Serial.print("H="); Serial.print(receiver_.last_humidity); Serial.println("%");
      } else {
        Serial.println("No data received yet");
      }
      Serial.println("======================================\n");
    }
    
    PressureReceiver& receiver_;
  };
  
  DisplayTimer display_timer_;
};

// Instance of the pressure receiver
PressureReceiver pressure_receiver;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to start
  Serial.println("\n\n\nCD Module Starting...");
  
  // Initialize kernel with default module behavior
  if (!kernel::begin(module_id)) {
    Serial.println("ERROR: Kernel failed to initialize!");
    return;
  }
  
  // Initialize indicators
  status_indicator.begin();
  error_indicator.begin();
  
  // Initialize buses
  can_bus.begin();
  serial_bus.begin();
  
  // Start receiver component
  pressure_receiver.begin();
  
  Serial.println("CD Module Ready - Waiting for pressure data");
}

void loop() {
  // Update indicators
  status_indicator.update();
  error_indicator.update();
}
