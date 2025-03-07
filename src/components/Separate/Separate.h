#include <library/wobc.h>


namespace component {

class Separate: public process::Component {
public:
    static const uint8_t component_id; // TBD
    static const unsigned linstener_queue_size;
    static const uint8_t gate_pin;

    Separate();

protected:
    Listener my_listener_;
    int start_time;
    
    void setup();
    void loop();
    void GateHIGH();

};

}