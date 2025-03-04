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
  float minAlt(NAN);
  float maxAlt(NAN);
  float diffAlt(NAN);
  minAlt = min(minAlt, presAlt);
  maxAlt = max(maxAlt, presAlt);
  diffAlt = maxAlt - minAlt;
  if (maxAlt - minAlt > 30 && maxAlt -presAlt > diffAlt-3) {
    pressure_.SendCommand();
  }
}

}
