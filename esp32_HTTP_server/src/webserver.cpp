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
#define     BUFFER_SIZE         20

//*****************************************************************************
//
//  Libraries used in source file.
//  WiFi.h for wifi connection, WebServer for the HTTP server configs.
//  HardwareSerial for USART connection and std vector for the message queue.
//
//*****************************************************************************

#define Period 10

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


//*****************************************************************************
//
// Global variables declaration.
//
//*****************************************************************************

const char* NETWORK_SSID = "MOB-ADRIANO";              // Network Login setup
const char* NETWORK_PASSWORD = "1801bianca";          // Network password setup

WebServer espWebServer(WEB_SERVER_PORT);    // HTTP server object 
HardwareSerial SerialPort(2);               // USART object

String Joint = "";                          // Joint value handler
String Angle = "";                          // Angle value handler

char message_buffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;
uint8_t voltage;
uint8_t applied;
uint8_t ia;

uint8_t step;

/**
 * @brief requestHandler()
 * Recevies Data from clients and sends it to the stm32.
 */


int logic = true;
TaskHandle_t task1Handle;
TaskHandle_t task2Handle;

void task1Function(void* parameter) 
{
  (void)parameter;  // Unused

  for (;;) {
    if (SerialPort.available()) 
    {
        char received_data = SerialPort.read();
        if(received_data == '\n')
        {
            message_buffer[bufferIndex] = '\0';
            bufferIndex = 0;
            Serial.print("STM32: ");
            Serial.println(message_buffer);
            Serial.println(voltage);
        }
        else
        {
            if (bufferIndex < BUFFER_SIZE - 1) 
            {
                message_buffer[bufferIndex] = received_data;
                bufferIndex++;
            }
        }
    }
  }
}

// Function to toggle LED2
void task2Function(void* parameter) {
  (void)parameter;  // Unused

  for (;;) 
  {
        for(int i=0; i < 100; i++)
            {
                step = i;
                if(i < 10)
                {
                    voltage = 0;
                    analogWrite(15, 0);
                    analogWrite(18, 0);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 20)
                {
                    voltage = 12*(2.5*i)/100;
                    applied = 2.5*i;
                    analogWrite(15, 2.55*applied);
                    analogWrite(18, 0);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 40)
                {   
                    voltage = 12;
                    analogWrite(15, 255);
                    analogWrite(18, 0);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 50)
                {
                    ia = i - 40;
                    voltage = 6 - 12*(2.5*ia)/100;
                    applied = 50-2.5*ia;
                    analogWrite(15, 2.55*applied);
                    analogWrite(18, 0);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 60)
                {
                    voltage = 0;
                    analogWrite(15, 0);
                    analogWrite(18, 0);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 70)
                {
                    ia = i-50;
                    voltage = -12*(2.5*(i-50))/100;
                    applied = 2.5*ia;
                    analogWrite(15, 0);
                    analogWrite(18, 2.55*applied);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
                else if(i < 90)
                {
                    voltage = -12;
                    analogWrite(15, 0);
                    analogWrite(18, 255);
                    vTaskDelay(pdMS_TO_TICKS(Period));   
                }
                else if(i < 100)
                {
                    ia = i - 90;
                    voltage = -(6 - 12*(2.5*ia)/100);
                    applied = 50-2.5*ia;
                    analogWrite(15, 0);
                    analogWrite(18, 2.55*applied);
                    vTaskDelay(pdMS_TO_TICKS(Period));
                }
            }	            
        }
}


void requestHandler()
{   
    if(espWebServer.hasArg("Joint"))        // Checks if the message has the Joint key.
    {
      Joint = espWebServer.arg("Joint");
      //espWebServer.send(200, "text/plain", "Message received: " + Joint);    // Gives a sucess indication.
      //SerialPort.println(Joint);            // Sends the data to the stm32.
    }
    if(espWebServer.hasArg("Angle")) // Checks if the message has the Angle key.
    {
      Angle = espWebServer.arg("Angle");
      espWebServer.send(200, "text/plain", "Message received: " + Angle); // Gives a sucess indication.
      SerialPort.println(Angle);            // Sends the data to the stm32.
      Serial.println("Sucessful Comunication. ");
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
    pinMode(15, OUTPUT);
    pinMode(18, OUTPUT);
    pinMode(19, OUTPUT);
    pinMode(35, INPUT_PULLDOWN);
    Serial.begin(115200);
    /*
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
    */
    digitalWrite(19, HIGH);
    SerialPort.begin(115200, SERIAL_8N1, RX_pin, TX_pin);    
    while(true)
    {
        int value = digitalRead(35);
        if(value)
        {
            xTaskCreatePinnedToCore(task1Function, "Task1", 1000, NULL, tskIDLE_PRIORITY, &task1Handle, 0);

            // Create and start Task 2
            xTaskCreatePinnedToCore(task2Function, "Task2", 1000, NULL, 1, &task2Handle, 0);

            // Start the FreeRTOS scheduler
            //vTaskStartScheduler();
            break;
        }
        else
        {
            //Serial.println(value);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
                                                       
}

/**
 * @brief loop()
 * keeps checking for clients request
 * for both HTTP and USART
 */

void loop() 
{
    //espWebServer.handleClient();
}

