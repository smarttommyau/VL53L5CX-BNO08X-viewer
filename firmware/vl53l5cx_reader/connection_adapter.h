#ifndef CONNECTION_ADAPTER_H
#define CONNECTION_ADAPTER_H

class ConnectionAdapter {

    public:
        void setup();
        void printf(const char* format, ...);
        static ConnectionAdapter& getInstance();

    private:
    //make it singleton
        ConnectionAdapter() {}
        ConnectionAdapter(const ConnectionAdapter&) = delete;
        ConnectionAdapter& operator=(const ConnectionAdapter&) = delete;
        static ConnectionAdapter* instance;
};



#endif // CONNECTION_ADAPTER_H