#include <Arduino.h>

#include <epos4.h>

EPOS4 my_epos(Serial4);

void setup() 
{
    Serial.begin(115200);
    Serial.println("Starting epos4 example");
}

void loop() 
{
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
    Serial.println("us");

    delay(10);
}