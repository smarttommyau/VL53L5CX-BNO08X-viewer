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


#ifdef WIFI_MODE
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
WiFiServer server(2340); // telnet server
WiFiClient client;

void ConnectionAdapter::setup() {
    /// Initial serial for debugging and logging
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ; // Wait for serial port to connect. Needed for native USB
    }

    // connect to WiFi
    Serial.println("Connecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int retryCount = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println(WiFi.status());
        retryCount++;
        if (retryCount > 20) { // timeout after 10 seconds
            Serial.println("Failed to connect to WiFi");
            delay(1000);
            ESP.restart();
        }
    }
    Serial.print("Connected to WiFi. IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();
    server.setNoDelay(true);
    Serial.println("WiFi connected, telnet server started");
}


void ConnectionAdapter::printf(const char* format, ...) {


    if(!client.connected()){
        return; // No client connected, skip sending
    }

    char buffer[128]; // Adjust this size if you expect longer log strings
    
    va_list args;
    va_start(args, format);
    // Format the text locally into our buffer safely
    vsnprintf(buffer, sizeof(buffer), format, args); 
    va_end(args);

    client.write(buffer);
}

void ConnectionAdapter::loop() {
    // Check for new client connections
    if (server.hasClient()) {
        if(!client.connected() || !client.available()){
            client = server.accept();
            Serial.println("New client connected");
        }
    }

    // If a client is connected, read data from it
    if (client && client.connected()) {
        if (client.available()) {
            char c = client.read();
            Serial.write(c); // Echo received data to Serial
        }
    } else  {
        client.stop(); // Disconnect if the client is no longer connected
    }
}


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