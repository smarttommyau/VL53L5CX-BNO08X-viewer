#ifndef CONNECTION_ADAPTER_H
#define CONNECTION_ADAPTER_H

class ConnectionAdapter {

    public:
        void setup();
        void printf(const char* format, ...);
        static ConnectionAdapter& getInstance();

    #ifdef WIFI_MODE
            void loop();
    #endif    

    private:
    //make it singleton
        ConnectionAdapter() {}
        ConnectionAdapter(const ConnectionAdapter&) = delete;
        ConnectionAdapter& operator=(const ConnectionAdapter&) = delete;
        static ConnectionAdapter* instance;
    #ifdef WIFI_MODE
        char telnetBuffer[1024]; // Buffer for outgoing telnet data
        u8_t loopCounter = 0; // Counter to track loop iterations
    #endif
};



#endif // CONNECTION_ADAPTER_H