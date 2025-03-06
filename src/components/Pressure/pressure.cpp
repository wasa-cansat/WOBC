#include "pressure.h"

namespace component {

Pressure::Pressure(TwoWire& wire, uint8_t unit_id, unsigned sample_freq_hz)
  : process::Component("Pressure", component_id),
    wire_(wire),
    unit_id_(unit_id),
    sample_timer_(*this, bme, unit_id, 1000 / sample_freq_hz) {
}

void Pressure::setup() {
  start(sample_timer_);
  storeOnCommand('Q'); // 高度規正値の設定コマンド
  Wire.begin();
  while(!bme.begin()){
    LOG("Could not find BME280");
    delay(1000);
  }
}

Pressure::SampleTimer::SampleTimer(Pressure& pressure_ref, Adafruit_BME280& bme_ref, uint8_t unit_id_ref, unsigned interval_ms)
  : process::Timer("Pressure", interval_ms),
    bme_(bme_ref), pressure_(pressure_ref), unit_id_(unit_id_ref) { 
}

void Pressure::SendCommand() {
  // コマンドを送信する関数
  wcpp::Packet packet2 = newPacket(64);
  packet2.command(command_id, tocomponent_id);
  packet2.append("SM").setInt((int)1);  // SM:Separate Mechanism 1:Separate
  // ... TODO
  sendPacket(packet2);
}

void Pressure::SampleTimer::callback() { // Timerで定期的に実行される関数

  // 高度規正値を不揮発メモリから読み込み
  double sealevel_Pa = 1013.25;
  wcpp::Packet qnh = loadPacket('Q'); 
  if (qnh) {
    auto e = qnh.find("Sp");
    if (e) sealevel_Pa = (*e).getFloat32();
  }

  // 圧力から高度を計算
  float presAlt(NAN);
  presAlt = bme_.readAltitude(sealevel_Pa);  // 0～10km程度の高度であればライブラリの関数で十分
/*
  // テレメトリを送信
  wcpp::Packet packet1 = newPacket(64);  // 1個だけなら16バイトのパケットを作成，５個なら64バイトのパケットを作成
  packet1.telemetry(telemetry_id, component_id(), unit_id_, 0xFF, 1234);
  //packet1.append("Sp").setInt((int)sealevel_Pa);
  //packet1.append("PR").setInt((int)pres/100);
  //packet1.append("TE").setInt((int)temp);
  //packet1.append("HU").setInt((int)hum);
  packet1.append("PA").setInt((int)presAlt);  // 計算された高度を追加
  // ... TODO
  sendPacket(packet1);
*/
  
  // 分離判定
  minAlt = min(minAlt, presAlt);
  maxAlt = max(maxAlt, presAlt);
  float diffAlt = maxAlt - minAlt;
  if (diffAlt > 30 && maxAlt -presAlt > diffAlt-3) {
    pressure_.SendCommand();
  }
}

Pressure::Pressure() 
  : process::Component("Pressure", component_id) {
}

void Pressure::begin() {
  process::Component::begin();
  
  // Initialize sensor
  if (initializeSensor()) {
    LOG("BME280 sensor initialized successfully");
  } else {
    LOG("Failed to initialize BME280 sensor");
  }
}

bool Pressure::initializeSensor() {
  // For CD module that only receives data, we don't actually need the sensor
  // This is just a stub implementation to avoid the compilation error
  // If you want to add actual sensor functionality later, uncomment the code below
  /*
  if (!bme_.begin(0x76)) {  // Try the default I2C address
    if (!bme_.begin(0x77)) { // Try alternate address
      return false;
    }
  }
  */
  return true;
}

void Pressure::loop() {
  unsigned long currentTime = millis();
  
  // In a receiver-only module, we don't need to read actual sensor data
  // This is placeholder code for if you later want to add sensor reading functionality
  
  /*
  // Check if it's time to read sensor data
  if (currentTime - lastReadingTime_ >= readingInterval_) {
    lastReadingTime_ = currentTime;
    
    readSensorData();
    sendTelemetry();
  }
  */
}

void Pressure::readSensorData() {
  // In a receiver-only module, this function wouldn't be used
  // Placeholder for future implementation if needed
  /*
  temperature_ = bme_.readTemperature();
  pressure_ = bme_.readPressure() / 100.0F; // Convert Pa to hPa
  altitude_ = bme_.readAltitude(1013.25); // Standard pressure at sea level
  humidity_ = bme_.readHumidity();
  */
}

void Pressure::sendTelemetry() {
  // In a receiver-only module, this function wouldn't be used
  // Placeholder for future implementation if needed
  /*
  wcpp::Packet packet;
  packet.module_id(module_id());
  packet.component_id(component_id);
  packet.packet_id(telemetry_id);
  
  packet.set("Tmp", temperature_);    // Temperature in °C
  packet.set("Prs", pressure_);       // Pressure in hPa
  packet.set("Alt", altitude_);       // Altitude in meters
  packet.set("Hum", humidity_);       // Humidity in %
  
  telemetry(packet);
  */
}

} // namespace component
