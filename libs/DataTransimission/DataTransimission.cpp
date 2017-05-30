#include "DataTransimission.h"
#include "LWiFi.h"
#include "Arduino.h"

DataTransimission::DataTransimission(const char* _ssid, const char* _pass, IPAddress _server, uint16_t _port)
{
    WiFiClient client;
    WiFiClass wifi;
    int wifi_status = WL_IDLE_STATUS;
    const char* ssid = _ssid;
    const char* pass = _pass;
    IPAddress server = _server;
    int port = _port;
}

size_t DataTransimission::send_data(const uint8_t* data, size_t size)
{
    client.stop();
    Serial.println("Stop client first");
    client.connect(server, port);
    Serial.println("Connecting to server");
    if(client.connected())
    {
        Serial.println("Client connected");
        client.write("d", 1);
        client.write((const uint8_t*)&size, sizeof(size_t));
        Serial.println("Client write data size");

        const uint8_t* buf;
        const uint8_t* end = data + size - MAX_BUFFER_SIZE;
        size_t sent_size = 0;
        for(buf = data; buf < end; buf += MAX_BUFFER_SIZE)
            sent_size += client.write(buf, MAX_BUFFER_SIZE);
        sent_size += client.write(buf, data + size - end);
        Serial.println("Client write done");
        return sent_size;
    }
    else
    {
        Serial.println("connect failed");
        return 0;
    }
}

void DataTransimission::begin()
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

void DataTransimission::connect()
{
    if(client.connect(server, port))
        Serial.println("Connected to server");
    else
        Serial.println("Disconnected from server");
}


void DataTransimission::print_wifi_status()
{
    Serial.print("SSID: ");
    Serial.println(wifi.SSID());

    IPAddress ip = wifi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    long rssi = wifi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

