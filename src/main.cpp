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

    constexpr BYTE NODE_ID = 0x01;
    constexpr WORD TARGET_POSITION_INDEX = 0x607A;
    constexpr BYTE TARGET_POSITION_SUBINDEX = 0x00;
    constexpr WORD CONTROL_WORD_INDEX = 0x6040;
    constexpr BYTE CONTROL_WORD_SUBINDEX = 0x00;
    constexpr WORD STATUS_WORD_INDEX = 0x6041;
    constexpr BYTE STATUS_WORD_SUBINDEX = 0x00;
    //constexpr WORD OPERATION_MODE_INDEX = 0x6060;
    //constexpr BYTE OPERATION_MODE_SUBINDEX = 0x00;
    constexpr DWORD CONTROL_WORD_DISABLE = 0x0006;
    constexpr DWORD CONTROL_WORD_ENABLE = 0x000F;
    constexpr DWORD CONTROL_WORD_TRIGGER = 0x003F;
    DWORD errorCode = 0x0000;
    int position = 50000;


    Serial.println("-------  Read Status Word  -------");
    DWORD status2 = my_epos.readObject(NODE_ID, 0x30B0, STATUS_WORD_SUBINDEX, errorCode);
    printf("Status Word: 0x%08X\n", int(status2));
    delay(2000);

    return;



    Serial.println("-------  Send Control Word Disable  -------");
    my_epos.writeObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_DISABLE, errorCode);
    delay(2000);

    Serial.println("-------  Send Control Word Enable  -------");
    my_epos.writeObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_ENABLE, errorCode);
    delay(2000);

    Serial.println("-------  Send Target Position  -------");
    my_epos.writeObject(NODE_ID, TARGET_POSITION_INDEX, TARGET_POSITION_SUBINDEX, position, errorCode);
    delay(2000);

    Serial.println("-------  Send Control Word Trigger  -------");
    my_epos.writeObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_TRIGGER, errorCode);
    delay(2000);

    Serial.println("-------  Read Status Word  -------");
    DWORD status = my_epos.readObject(NODE_ID, STATUS_WORD_INDEX, STATUS_WORD_SUBINDEX, errorCode);
    printf("Status Word: 0x%08X\n", int(status));
    delay(2000);

    return;

    my_epos.go_to_position(-50000);
    Serial.println("pos -50000");
    delay(1000);
    my_epos.go_to_position(50000);
    Serial.println("pos 50000");
    delay(1000);
}