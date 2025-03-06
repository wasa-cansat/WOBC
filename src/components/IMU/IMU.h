#include <library/wobc.h>
#include <Wire.h>
#include "lib/BMI088/BMI088.h"
#include "lib/QMC5883L/QMC5883LCompass.h"

namespace component{

class IMU : public process::Component{
public:
    static const uint8_t component_id = 20;
    static const uint8_t telemetry_id = 'M';

    IMU(TwoWire& wire, uint8_t unit_id, unsigned sample_freq_hz = 10);

protected:
    TwoWire& wire_;

    void setup() override;


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
