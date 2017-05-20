#include <LWiFi.h>

#define NUM_CHAN 4
#define NUM_LIST 1000
#define SEND_MAX_LEN 1024

char ssid[] = "54321";
char pass[] = "2222222222";

int status = WL_IDLE_STATUS;

WiFiClient client;
IPAddress server(192,168,0,134);
int port = 5000;

int data[NUM_LIST][NUM_CHAN];
const int max_list   = SEND_MAX_LEN / (NUM_CHAN * sizeof(int));
const int send_times = NUM_LIST / max_list;
const int res_list   = NUM_LIST % max_list;

void setup()
{
    Serial.begin(115200);
    while(!Serial);
    while(status != WL_CONNECTED)
    {
        Serial.print("Connecting to wifi ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        delay(1000);
    }
    Serial.println("Connected to wifi");

    for(int i = 0; i < NUM_LIST; i++)
        for(int j = 0; j < NUM_CHAN; j++)
            data[i][j] = i * NUM_CHAN + j;

    Serial.println("Press <a> to send data");
    Serial.print("The size of data is: ");
    Serial.println(sizeof(data));
}

void loop()
{
    if(Serial.available())
        if(Serial.read() == 'a')
            send_data();

}

void send_data()
{
    client.stop();
    Serial.println("Stop client first");
    client.connect(server, port);
    Serial.println("Connecting to server");
    if(client.connected())
    {
        Serial.println("Client connected");
        send_data_header();
        send_data_value();
    }
    else
        Serial.println("connect failed");
}

void send_data_header()
{
    Serial.println("send data header");
    char data_type = 'd';
    int  data_size = sizeof(data);
    client.write((const uint8_t*)&data_type, sizeof(char));
    client.write((const uint8_t*)&data_size, sizeof(int));
    Serial.println("finished sending data header");
}


void send_data_value()
{
    Serial.println("send data value");
    for(int t = 0; t < send_times; t++) 
    {
        Serial.print("=====");
        Serial.print(t+1);
        Serial.println("-th send=====");
        int buf[max_list * NUM_CHAN];
        for(int i = 0; i < max_list; i++)
        {
            Serial.print("Pack ");
            Serial.print(i+1);
            Serial.print("-th data: ");
            for(int j = 0; j < NUM_CHAN; j++)
            {
                buf[i*NUM_CHAN + j] = data[t*max_list + i][j];
                Serial.print(buf[i*NUM_CHAN + j]);
                Serial.print(" ");
            }
            Serial.println();
        }
        Serial.println("client write buffer ... ");
        client.write((const uint8_t*)buf, sizeof(buf));
        Serial.print("Send ");
        Serial.print(t+1);
        Serial.println("-th time buffer successfully");
        Serial.println();
    }
    if(res_list)
    {
        Serial.println("Send residue list");
        int buf[res_list * NUM_CHAN];
        for(int i = 0; i < res_list; i++)
        {
            Serial.print("Pack ");
            Serial.print(i+1);
            Serial.print("-th data: ");
            for(int j = 0; j < NUM_CHAN; j++)
            {
                buf[i*NUM_CHAN + j] = data[send_times*max_list + i][j];
                Serial.print(buf[i*NUM_CHAN + j]);
                Serial.print(" ");
            }
            Serial.println();
        }
        Serial.println("client write buffer ... ");
        client.write((const uint8_t*)buf, sizeof(buf));
        Serial.println("finished sending residue list");
    }
    Serial.println("Send data done.");
}
