#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <vector>
#include <HardwareSerial.h>

#define     WEB_SERVER_PORT     80  
#define     RX_pin              16
#define     TX_pin              17

const char* NETWORK_SSID = ""; 
const char* NETWORK_PASSWORD = "";

WebServer espWebServer(WEB_SERVER_PORT);
HardwareSerial SerialPort(1);

String Joint = "";
String Angle = "";
 
void requestHandler()
{   
    if(espWebServer.hasArg("Joint"))
    {
      Joint = espWebServer.arg("Joint");
      espWebServer.send(200, "text/plain", "Message received: " + Joint);
    }
    if (espWebServer.hasArg("Angle"))
    {
      Angle = espWebServer.arg("Angle");
      espWebServer.send(200, "text/plain", "Message received: " + Angle);
      Serial.println("Sucessful Comunication. ");
    }
    else 
    {
      espWebServer.send(400, "text/plain", "Bad Request");
      Serial.println("Failed Comunication. ");
    }

}

void setup() 
{
    Serial.begin(115200);
    Serial.print("Connecting to ");
    Serial.println(NETWORK_SSID);
    WiFi.begin(NETWORK_SSID, NETWORK_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    espWebServer.on("/", requestHandler);     
    espWebServer.begin();
    Serial.println("HTTP server started");          

   SerialPort.begin(15200, SERIAL_8N1, RX_pin, TX_pin);                                                       
}

void loop() 
{
    espWebServer.handleClient();
}

