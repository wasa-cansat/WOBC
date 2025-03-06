#include <library/wobc.h>
#include <Wire.h>
#include "lib/BMI088/BMI088.h"
#include "lib/QMC5883L/QMC5883LCompass.h"

namespace component{

class IMU : public process::Component{
public:
    static const uint8_t component_id = 20;
    static const uint8_t telemetry_id = 'I';
    static const uint8_t command_id = 'C';

    int status;  // 状態
    uint8_t ctrl_component_id;  // 送り先のコンポーネントID

    float b1;  // 楕円パラメータ
    float b2;
    float b3;
    float t1;  // 固有値
    float t2;  
    float t3;  
    float p11;  // 固有値ベクトル1
    float p12;
    float p13;
    float p21;  // 固有値ベクトル2
    float p22;
    float p23;
    float p31;  // 固有値ベクトル3
    float p32;
    float p33;
    float q1;  // 生データ
    float q2;
    float q3;
    float qc1;  // キャリブレーション後
    float qc2;
    float qc3;

    IMU(TwoWire& wire, uint8_t unit_id, unsigned sample_freq_hz = 100);

protected:
    TwoWire& wire_;
    Bmi088Accel accel_;
    Bmi088Gyro gyro_;
    QMC5883LCompass compass_;

    void setup() override;
    float getroll(float ay, float az);
    float getpitch(float ax, float ay, float az);
    float getyaw(float roll, float pitch, float mx, float my, float mz);

    void SendCommand(float yaw);


    class SampleTimer : public process::Timer{
    public:
        SampleTimer(IMU& imu_ref, uint8_t unit_id_ref, unsigned interval_ms);

    protected:
        void callback() override;

    private:
        IMU& imu_;
        Bmi088Accel accel_;
        Bmi088Gyro gyro_;
        QMC5883LCompass compass_;
        uint8_t unit_id_;
    } sample_timer_;
};

}
