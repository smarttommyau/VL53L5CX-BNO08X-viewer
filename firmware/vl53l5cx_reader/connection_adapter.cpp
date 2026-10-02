#include "config.h"
#include "connection_adapter.h"
#include <HardwareSerial.h>

ConnectionAdapter* ConnectionAdapter::instance = nullptr;

ConnectionAdapter& ConnectionAdapter::getInstance() {
            if (!instance) {
                instance = new ConnectionAdapter();
            }
            return *instance;
}


#ifdef BLE_MODE


#else // USB Mode
void ConnectionAdapter::setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ; // Wait for serial port to connect. Needed for native USB
    }
}

void ConnectionAdapter::printf(const char* format, ...) {
  char buffer[128]; // Adjust this size if you expect longer log strings
    
    va_list args;
    va_start(args, format);
    // Format the text locally into our buffer safely
    vsnprintf(buffer, sizeof(buffer), format, args); 
    va_end(args);
    
    // Send the final, flat string directly downstream
    Serial.print(buffer); 
}
#endif // USB Mode

//TODO: wifi mode