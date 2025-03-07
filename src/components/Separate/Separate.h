#include <library/wobc.h>
#include "../Pressure/pressure.h"

namespace component {

class Separate: public process::Component {
public:
    static const uint8_t component_id = 0x28; // TBD
    static const unsigned linstener_queue_size = 4;
    static const uint8_t gate_pin;

    Separate(): process::Component("Separate", component_id) {};

protected:
    Listener my_listener_;
    int start_time;
    
    void setup();
    void loop();
    void GateHIGH();

};

}