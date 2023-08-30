#include "controller.hpp"
#include "algorithms.hpp"
#include <SoftwareSerial.h>


SoftwareSerial espSerial(2, 3);

ADPI_Controller motor_controller_one(1.75, 0.25, 1000);
controller joint_one(0, 5, 4, &motor_controller_one, 0.01);
unsigned long time = 0;

void setup() 
{
    Serial.begin(9600);
    espSerial.begin(9600);
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);
}

void loop() 
{
    if(espSerial.available()) 
    {
        String receivedData = espSerial.readStringUntil('\n');
        Serial.print("Received from ESP32: ");
        Serial.println(receivedData);
        joint_one.Update(receivedData.toFloat());
        espSerial.println(receivedData);
    }   

    joint_one.loop();
    delay(10);
    if(millis() - time > 1000)
    {
        Serial.print("Position x[k]: ");
        Serial.println(joint_one.sensor_k);
        Serial.print("Setpoint[k]: ");
        Serial.println(motor_controller_one.log_setpoint());
        Serial.print("Error[k]: ");
        Serial.println(motor_controller_one.log_error());
        Serial.print("Prop ControlAction[k]: ");
        Serial.println(motor_controller_one.log_P_control_action());
        Serial.print("Int ControlAction[k]: ");
        Serial.println(motor_controller_one.log_I_control_action());
        Serial.println("###############");
        time = millis();
    }
}