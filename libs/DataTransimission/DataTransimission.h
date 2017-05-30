#ifndef _DATATRANSIMISSION_H_
#define _DATATRANSIMISSION_H_
#include "Arduino.h"
#include "LWiFi.h"

#define MAX_BUFFER_SIZE 1024

class DataTransimission
{
    public:
        DataTransimission(const char* ssid, const char* pass, IPAddress server, uint16_t port);
        void begin();
        void print_wifi_status();
        void connect();
        size_t send_data(const uint8_t* data, size_t size);
        
    private:
        IPAddress server;
        uint16_t port;
        WiFiClient client; 
        WiFiClass wifi;
        const char* ssid;
        const char* pass;
        int wifi_status;
};

#endif
