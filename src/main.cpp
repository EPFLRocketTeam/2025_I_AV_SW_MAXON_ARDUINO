/*
    File: main.cpp
    Author: Axel Juaneda
    Organization: EPFL Rocket Team
    Version : 1.0
*/

#include <Arduino.h>

#include <epos4.h>

EPOS4 my_epos(Serial4);

long targetPositions[] = { -50000, 50000 };
int currentTargetIndex = 0;
unsigned long lastMoveTime = 0;
const unsigned long moveInterval = 2000; // ms between moves

void setup() 
{
    Serial.begin(115200);
    Serial.println("Starting epos4 example in 2s");
    delay(2000);
    Serial.println("Starting epos4 example");

    my_epos.set_homing_offset_distance(50000);
}

void loop() 
{   
    unsigned long start = micros();
    
    /*
    if (millis() - lastMoveTime >= moveInterval) 
    {
        Serial.println("------- NEW SETPOINT -------\n");
        currentTargetIndex = 1 - currentTargetIndex;
        my_epos.go_to_position(targetPositions[currentTargetIndex]);
        lastMoveTime = millis();
    }
    */
    my_epos.current_threshold_homing();
    my_epos.tick();

    unsigned long stop = micros();

    Serial.print("Elapsed time: ");
    Serial.print(stop - start);
    Serial.println("us");
    delay(10);
}