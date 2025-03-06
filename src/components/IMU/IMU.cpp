#include "IMU.h"

namespace component{

IMU::IMU(TwoWire& wire, uint8_t unit_id, unsigned sample_freq_hz)
    : process::Component("IMU", component_id),
    wire_(wire),
    accel_(wire, 0x19),
    gyro_(wire, 0x68),
    compass_(),
    sample_timer_(*this, unit_id, 1000 / sample_freq_hz) {
}

void IMU::setup() {
    start(sample_timer_);
    Wire.begin();
    while(!accel_.begin()){
        LOG("Could not initialize BMI088 Accel");
        delay(1000);
    }
    while(!gyro_.begin()){
        LOG("Could not initialize BMI088 Gyro");
        delay(1000);
    }
    compass_.init();
}

float IMU::getroll(float ay, float az) {
    return atan2(ay, az);
}

float IMU::getpitch(float ax, float ay, float az) {
    return atan2(-ax, sqrt(ay * ay + az * az));
}

float IMU::getyaw(float roll, float pitch, float mx, float my, float mz) {
    float xh = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch);
    float yh = my * cos(roll) - mz * sin(roll);
    return atan2(-yh, xh);
}

IMU::SampleTimer::SampleTimer(IMU& imu_ref, uint8_t unit_id_ref, unsigned interval_ms)
    : process::Timer("IMU", interval_ms),
    imu_(imu_ref),
    accel_(imu_.accel_),
    gyro_(imu_.gyro_),
    compass_(imu_.compass_),
    unit_id_(unit_id_ref) {

}   

void IMU::SendCommand(float yaw) {
    wcpp::Packet packet2 = newPacket(64);
    packet2.command(command_id, ctrl_component_id);
    packet2.append("YA").setFloat16(yaw);
    sendPacket(packet2);
}

void IMU::SampleTimer::callback() {
    accel_.readSensor();
    gyro_.readSensor();
    compass_.read();

    imu_.q1 = compass_.getX();
    imu_.q2 = compass_.getY();
    imu_.q3 = compass_.getZ();
    imu_.qc1 = imu_.p11*(imu_.p11*imu_.q1 + imu_.p21*imu_.q2 + imu_.p31*imu_.q3 + (imu_.b1*imu_.p11 + imu_.b2*imu_.p12 + imu_.b3*imu_.p13)/(2*imu_.t1) + imu_.p12*(imu_.p12*imu_.q1 + imu_.p22*imu_.q2 + imu_.p32*imu_.q3 + (imu_.b1*imu_.p21 + imu_.b2*imu_.p22 + imu_.b3*imu_.p23)/(2*imu_.t2) + imu_.p13*(imu_.p13*imu_.q1 + imu_.p23*imu_.q2 + imu_.p33*imu_.q3 + (imu_.b1*imu_.p31 + imu_.b2*imu_.p32 + imu_.b3*imu_.p33)/(2*imu_.t3))));
    imu_.qc2 = imu_.p21*(imu_.p11*imu_.q1 + imu_.p21*imu_.q2 + imu_.p31*imu_.q3 + (imu_.b1*imu_.p11 + imu_.b2*imu_.p12 + imu_.b3*imu_.p13)/(2*imu_.t1) + imu_.p12*(imu_.p12*imu_.q1 + imu_.p22*imu_.q2 + imu_.p32*imu_.q3 + (imu_.b1*imu_.p21 + imu_.b2*imu_.p22 + imu_.b3*imu_.p23)/(2*imu_.t2) + imu_.p13*(imu_.p13*imu_.q1 + imu_.p23*imu_.q2 + imu_.p33*imu_.q3 + (imu_.b1*imu_.p31 + imu_.b2*imu_.p32 + imu_.b3*imu_.p33)/(2*imu_.t3))));
    imu_.qc3 = imu_.p31*(imu_.p11*imu_.q1 + imu_.p21*imu_.q2 + imu_.p31*imu_.q3 + (imu_.b1*imu_.p11 + imu_.b2*imu_.p12 + imu_.b3*imu_.p13)/(2*imu_.t1) + imu_.p12*(imu_.p12*imu_.q1 + imu_.p22*imu_.q2 + imu_.p32*imu_.q3 + (imu_.b1*imu_.p21 + imu_.b2*imu_.p22 + imu_.b3*imu_.p23)/(2*imu_.t2) + imu_.p13*(imu_.p13*imu_.q1 + imu_.p23*imu_.q2 + imu_.p33*imu_.q3 + (imu_.b1*imu_.p31 + imu_.b2*imu_.p32 + imu_.b3*imu_.p33)/(2*imu_.t3))));
    // テレメトリを送信
    wcpp::Packet packet1 = newPacket(64);
    packet1.telemetry(telemetry_id, component_id());
    packet1.append("AX").setFloat16((float)accel_.getAccelX_mss());
    packet1.append("AY").setFloat16((float)accel_.getAccelY_mss());
    packet1.append("AZ").setFloat16((float)accel_.getAccelZ_mss());
    packet1.append("GX").setFloat16((float)gyro_.getGyroX_rads());
    packet1.append("GY").setFloat16((float)gyro_.getGyroY_rads());
    packet1.append("GZ").setFloat16((float)gyro_.getGyroZ_rads());
    packet1.append("MX").setFloat16((float)imu_.qc1);
    packet1.append("MY").setFloat16((float)imu_.qc2);
    packet1.append("MZ").setFloat16((float)imu_.qc3);
    packet1.append("RO").setFloat16(imu_.getroll(accel_.getAccelY_mss(), accel_.getAccelZ_mss()));
    packet1.append("PI").setFloat16(imu_.getpitch(accel_.getAccelX_mss(), accel_.getAccelY_mss(), accel_.getAccelZ_mss()));
    packet1.append("YA").setFloat16(imu_.getyaw(imu_.getroll(accel_.getAccelY_mss(), accel_.getAccelZ_mss()), imu_.getpitch(accel_.getAccelX_mss(), accel_.getAccelY_mss(), accel_.getAccelZ_mss()), imu_.qc1, imu_.qc2, imu_.qc3));
    // ... TODO
    sendPacket(packet1);

    if(imu_.status == 1){  // 缶サットの近距離制御用など
        imu_.SendCommand(imu_.getyaw(imu_.getroll(accel_.getAccelY_mss(), accel_.getAccelZ_mss()), imu_.getpitch(accel_.getAccelX_mss(), accel_.getAccelY_mss(), accel_.getAccelZ_mss()), imu_.qc1, imu_.qc2, imu_.qc3));
    }
}

}