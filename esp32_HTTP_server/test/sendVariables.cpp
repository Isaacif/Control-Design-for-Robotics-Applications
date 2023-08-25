/**
 * @file webserver.cpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief Source file for esp32 HTTP server
 * @version 0.1
 * @date 2023-07-02
 * 
 */

//*****************************************************************************
//
// Macro defines for functions usage.
//
//*****************************************************************************

#define     WEB_SERVER_PORT     80  
#define     RX_pin              16
#define     TX_pin              17

//*****************************************************************************
//
//  Libraries used in source file.
//  WiFi.h for wifi connection, WebServer for the HTTP server configs.
//  HardwareSerial for USART connection and std vector for the message queue.
//
//*****************************************************************************

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <vector>

//*****************************************************************************
//
// Global variables declaration.
//
//*****************************************************************************

const int numBytes = 4; // Number of bytes to read from STM32


const char* NETWORK_SSID = "MOB-ADRIANO";              // Network Login setup
const char* NETWORK_PASSWORD = "1801bianca";          // Network password setup

WebServer espWebServer(WEB_SERVER_PORT);    // HTTP server object 
HardwareSerial SerialPort(2);               // USART object

String Joint = "";                          // Joint value handler
String Angle = "";                          // Angle value handler


/**
 * @brief requestHandler()
 * Recevies Data from clients and sends it to the stm32.
 */

void requestHandler()
{   
    if(espWebServer.hasArg("Joint"))        // Checks if the message has the Joint key.
    {
      Joint = espWebServer.arg("Joint");
      espWebServer.send(200, "text/plain", "Message received: " + Joint);    // Gives a sucess indication.
      SerialPort.println(Joint);            // Sends the data to the stm32.
    }
    if (espWebServer.hasArg("Angle")) // Checks if the message has the Angle key.
    {
      Angle = espWebServer.arg("Angle");
      espWebServer.send(200, "text/plain", "Message received: " + Angle); // Gives a sucess indication.
      SerialPort.println(Angle);            // Sends the data to the stm32.
      Serial.println("Sucessful Comunication. ");
      SerialPort.println(Angle);
    }
    else 
    {
      espWebServer.send(400, "text/plain", "Bad Request");      // Gives a failure indication.
      Serial.println("Failed Comunication. ");
    }

}

/**
 * @brief setup()
 * configures wifi connection and creates the HTTP server
 * establishes USART connetion
 * prints out the IP address
 */

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

    SerialPort.begin(115200, SERIAL_8N1, RX_pin, TX_pin);                                                       
}

/**
 * @brief loop()
 * keeps checking for clients request
 * for both HTTP and USART
 */

void loop() {
    char buffer[20];

    // Wait for the string from STM32
    while (!SerialPort.available()) {
        // Wait until data is available
    }

    // Read the received string from STM32
    int bytesRead = SerialPort.readBytesUntil('\n', buffer, sizeof(buffer) - 1);

    if (bytesRead > 0) {
        buffer[bytesRead] = '\0'; // Null-terminate the string
        // Parse the received string back to an integer
        int receivedValue = atoi(buffer);

        // Do something with the received integer value (print it in this case)
        Serial.print("Received value from STM32: ");
        Serial.println(receivedValue);
    } else {
        // No data received or an error occurred
        Serial.println("Error reading data from STM32.");
    }

    delay(100);
}

