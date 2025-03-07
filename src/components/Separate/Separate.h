#include <library/wobc.h>

namespace component {

class Separate: public process::Component {
public:
    static const uint8_t component_id = 0x28; // TBD
    static const unsigned linstener_queue_size = 4;

    Separate(): process::Component("Separate", component_id) {};

protected:
    Listener my_listener_;
    
    void setup() override {
        //my_listener_.telemetry();
        my_listener_.command();

        

    };

};

}