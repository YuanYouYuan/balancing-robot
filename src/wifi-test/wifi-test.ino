#include <LWiFi.h>

char ssid[] = "BRL03_2.4G";
char pass[] = "biorola33669771";

int status = WL_IDLE_STATUS;
char server[] = "download.labs.meditek.com";

WiFiClient wifi_client;
WiFiClass wifi;

void setup()
{
    Serial.begin(115200);
    while(!Serial);
    if(wf.status() == WL_NO_SHIELD)
    {
        Serial.println("WiFi shield not present");
        while(true);
    }

    while(status != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to SSID ");
        Serial.println(ssid);
        status = wf.begin(ssid, pass);
        delay(10000);
    }
    Serial.println("Connected to wifi");
    print_wifi_status();
}

void loop()
{

}

void print_wifi_status()
{
    Serial.print("SSID: ");
    Serial.println(wf.SSID());
    IPAddress ip = wf.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
    long rssi = wf.RSSI();
    Serial.print("Signal strength (RSSI): ");
    Serial.print(rssi);
    Serial.println(" dBm");

}

