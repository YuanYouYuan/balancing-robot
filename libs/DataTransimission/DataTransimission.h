#ifndef _DATATRANSIMISSION_H_
#define _DATATRANSIMISSION_H_
#include "Arduino.h"
#include "LWiFi.h"

#define MAX_BUFFER_SIZE 1024

class DataTransimission
{
    public:
        DataTransimission(const char* ssid, const char* pass, IPAddress server, uint16_t port);
        void connect_wifi();
        void print_wifi_status();
        bool connect_server();
        size_t send_data(const uint8_t* data, size_t size);
        
    private:
        WiFiClass wifi;
        const char* ssid;
        const char* pass;
        IPAddress server;
        uint16_t port;
        WiFiClient client; 
        int wifi_status;
};

#endif
