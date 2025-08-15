#include <Arduino.h>

#include <epos4.h>

EPOS4 my_epos(Serial4);

long targetPositions[] = { -500000, 500000 };
int currentTargetIndex = 0;
unsigned long lastMoveTime = 0;
const unsigned long moveInterval = 10000; // ms between moves

void setup() 
{
    Serial.begin(115200);
    Serial.println("Starting epos4 example in 2s");
    delay(2000);
    Serial.println("Starting epos4 example");
}

void loop() 
{
    /*
    constexpr BYTE NODE_ID = 0x01;
    constexpr WORD CONTROL_WORD_INDEX = 0x6040;
    constexpr BYTE CONTROL_WORD_SUBINDEX = 0x00;
    constexpr DWORD CONTROL_WORD_SHUTDOWN = 0x0006;
    constexpr DWORD CONTROL_WORD_SWITCH_ON = 0x0007;

    unsigned long start = micros();
    if (!my_epos.get_isReading())
    {
        // my_epos.startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_SHUTDOWN);
        my_epos.startReadObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX);
    }
    else
    {
        DWORD error_code = 0x0001;
        DWORD read_value = 0x0000;
        //if(my_epos.pollWriteObject(error_code)) Serial.println(error_code);
        if (my_epos.pollReadObject(read_value, error_code)) Serial.println(read_value);
    }
    unsigned long stop = micros();

    Serial.print("Elapsed time: ");
    Serial.print(stop - start);
    Serial.println("us");*/
    
    unsigned long start = micros();

    if (millis() - lastMoveTime >= moveInterval) 
    {
        Serial.println("------- NEW SETPOINT -------\n");
        currentTargetIndex = 1 - currentTargetIndex;
        my_epos.go_to_position(targetPositions[currentTargetIndex]);
        lastMoveTime = millis();
    }
    my_epos.tick();

    unsigned long stop = micros();

    Serial.print("Elapsed time: ");
    Serial.print(stop - start);
    Serial.println("us");
    delay(10);
}