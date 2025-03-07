#include "Separate.h"

namespace component {

Separate::Separate(): process::Component("Separate", component_id) {};


void Separate::setup() {
    //my_listener_.telemetry();
    my_listener_.command();
    // コンポーネントIDでフィルタリング
    my_listener_.component(Pressure::component_id);
    // リスナーを開始(キューサイズ4で)
    listen(my_listener_, linstener_queue_size);

    // ゲートの初期化
    pinMode(gate_pin, OUTPUT);
    digitalWrite(gate_pin, LOW);
};

void Separate::GateHIGH() {
    start_time = millis();
    while(millis() - start_time < 10000) {
        digitalWrite(gate_pin, HIGH);
    }
    digitalWrite(gate_pin, LOW); 
};

void Separate::loop() {
    // リスナーからの受信データを処理
    while (my_listener_) {
        wcpp::Packet packet = my_listener_.pop();
        if (packet) {
            // パケットの内容を解析
            auto e = packet.find("SM");
            LOG("Separate Mechanism: %d", (*e).getInt());
            // ... TODO
            start_time = millis();
            if (e && millis() - start_time < 10000) {
                pinMode(gate_pin, HIGH);
            }
        }
    }
};

}