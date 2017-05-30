#include "DataTransimission.h"
#include "LWiFi.h"
#include "Arduino.h"

DataTransimission::DataTransimission(const char* _ssid, const char* _pass, IPAddress _server, uint16_t _port)
{
    wifi_status = WL_IDLE_STATUS;
    ssid        = _ssid;
    pass        = _pass;
    server      = _server;
    port        = _port;
}

size_t DataTransimission::send_data(const uint8_t* data, size_t size)
{
    // if(!client.connected())
        // connect_server();
    connect_server();

    Serial.println("Client write data header");
    client.write("d", 1);
    Serial.println("Client write data size");
    client.write((const uint8_t*)&size, sizeof(size_t));

    const uint8_t* buf;
    const uint8_t* end = data + size - MAX_BUFFER_SIZE;
    size_t sent_size = 0;
    for(buf = data; buf < end; buf += MAX_BUFFER_SIZE)
        sent_size += client.write(buf, MAX_BUFFER_SIZE);
    sent_size += client.write(buf, data + size - buf);
    Serial.println("Client write done");
    return sent_size;
}

void DataTransimission::connect_wifi()
{
    while (wifi_status != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        wifi_status = wifi.begin(ssid, pass);
        delay(3000);
    }
    Serial.println("Connected to wifi");

}

bool DataTransimission::connect_server()
{
    Serial.println("Stop client first");
    client.stop();
    Serial.print("Connect to ");
    Serial.print(server);
    Serial.print(":");
    Serial.println(port);
    if(client.connect(server, port))
    {
        Serial.println("Connect succeed");
        delay(1000);
        return true;
    }
    else
    {
        Serial.println("Connect failed");
        return false;
    }
}


void DataTransimission::print_wifi_status()
{
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

