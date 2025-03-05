#include "gps.h"
#include <Arduino.h>

namespace component {
GPS::GPS(driver::GenericSerialClass& serial, uint32_t baud)
  : process::Component("GPS", component_id),
  serial_(serial), baud_(baud) {    
}

void GPS::setup(){
  serial_.begin(baud_);
}

float GPS::distanceBetween(float lat, float lng) {
  float deltaLat = radians(lat - destination_lat);
  float deltaLong = radians(lng - destination_long);
  float a = sin(deltaLat / 2) * sin(deltaLat / 2) + cos(radians(destination_lat)) * cos(radians(lat)) * sin(deltaLong / 2) * sin(deltaLong / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371000 * c;

}

float GPS::courseTo(float lat, float lng) {
  float deltaLong = radians(lng - destination_long);
  float y = sin(deltaLong) * cos(radians(lat));
  float x = cos(radians(destination_lat)) * sin(radians(lat)) - sin(radians(destination_lat)) * cos(radians(lat)) * cos(deltaLong);
  return degrees(atan2(y, x));
}


void GPS::loop() {
  int start = millis();
  while(serial_.available() && millis()-start<500) {
    if (gps_.encode(serial_.read())) {
      wcpp::Packet packet = newPacket(64);
      float current_lat = gps_.location.lat();
      float current_lng = gps_.location.lng();

      packet.telemetry(telemetry_id, component_id, kernel::unit_id(), 0xFF, 1234);
      packet.append("LA").setFloat64(current_lat);
      packet.append("LO").setFloat64(current_lng);
      packet.append("AL").setInt((int)gps_.altitude.meters());
      packet.append("DD").setFloat16(gps_.distanceBetween(current_lat, current_lng));
      packet.append("CT").setFloat16(gps_.courseTo(current_lat, current_lng));
      //packet.append("UT").setint();
      sendPacket(packet);
    }
  }
}

} // namespace component