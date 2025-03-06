#include "gps.h"
#include "TinyGPSPlus.h"
#include <Arduino.h>

namespace component {
GPS::GPS(driver::GenericSerialClass& serial, uint32_t baud)
  : process::Component("GPS", component_id),
  serial_(serial), baud_(baud) {    
}

void GPS::setup(){
  serial_.begin(baud_);
}

void GPS::loop() {
  int start = millis();
  while(serial_.available() && millis()-start<500) {
    if (gps_.encode(serial_.read())) {
      wcpp::Packet packet = newPacket(64);
      float current_lat = gps_.location.lat();
      float current_lng = gps_.location.lng();

      //packet.telemetry(telemetry_id, component_id, kernel::unit_id(), 0xFF, 1234);  // リモート用
      packet.telemetry(telemetry_id, component_id);  // ローカル用
      packet.append("LA").setFloat64(current_lat);
      packet.append("LO").setFloat64(current_lng);
      packet.append("AL").setInt((int)gps_.altitude.meters());
      packet.append("DB").setFloat16(TinyGPSPlus::distanceBetween(current_lat, current_lng, destination_lat, destination_long));
      packet.append("CT").setFloat16(TinyGPSPlus::courseTo(current_lat, current_lng, destination_lat, destination_long));
      //packet.append("UT").setint();
      sendPacket(packet);
    }
  }
}

} // namespace component