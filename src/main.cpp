#include <Arduino.h>

#include <epos4.h>

void setup() 
{
    Serial.begin(115200);
    EPOS4 my_epos(Serial1);

    Serial.println("Starting epos4 example");
}

void loop() 
{

}