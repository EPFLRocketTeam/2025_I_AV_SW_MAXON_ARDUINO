#include <Arduino.h>

#include <epos4.h>

EPOS4 my_epos(Serial1);

void setup() 
{
    Serial.begin(115200);
    Serial.println("Starting epos4 example");

    my_epos.go_to_position(0);
}

void loop() 
{
    my_epos.go_to_position(0);
    delay(500);
    my_epos.go_to_position(10000);
    delay(500);
}