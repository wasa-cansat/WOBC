#include <library/wobc.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <cmath>

namespace component {

class Pressure: public process::Component {
public:
  static const uint8_t component_id = 0x25; // TBD
  static const uint8_t tocomponent_id = 0x26; // TBD
  static const uint8_t telemetry_id = 'E'; // TBD
  static const uint8_t command_id = 'F'; // TBD
  static const int max_index = 32;  // p と coe の最大インデックス

  Pressure(TwoWire& wire, uint8_t unit_id, unsigned sample_freq_hz = 10);

protected:
  TwoWire& wire_;
  Adafruit_BME280 bme;
  uint8_t unit_id_;

  struct PressureData {
    int pressure;
    int altitude;
  } p[max_index + 1];

  struct Coefficients {
    float a;
    float b;
    float c;
  } coe[max_index + 1];

  void setup() override;
  
  void SendCommand();  // 送り先のコンポーネントID

  class SampleTimer: public process::Timer {
  public:
    SampleTimer(Pressure& pressure_ref, Adafruit_BME280& bme_ref, uint8_t unit_id_ref, unsigned interval_ms);

  protected:
    void callback() override;

  private:
    Adafruit_BME280& bme_;
    Pressure& pressure_;
    uint8_t unit_id_;
  } sample_timer_;
};

}
