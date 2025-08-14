#include <Arduino.h>

#include <epos4.h>

EPOS4 my_epos(Serial4);

PpmCmd ppm;

void setup() 
{
    Serial.begin(115200);
    Serial.println("Starting epos4 example");

    delay(3000);

    ppm.changeImmediately = true;
    ppm.relative = true;
    ppm.targetPos = 50000;
    
    EPOS4 my_epos(Serial4);
    my_epos.requestPpmMove(ppm);
    //my_epos.go_to_position(0);


    // epos.requestBringup();
}

void loop() 
{
    my_epos.tick();

    /*
    PpmCmd cmd;
    cmd.targetPos = 100000; // counts
    cmd.relative  = false;
    cmd.changeImmediately = true;
    epos.requestPpmMove(cmd, 15000);
    */
    /*
    Serial.println("-------  Reading word  -------");
    DWORD status2 = my_epos.readObject(NODE_ID, 0x30B2, 0x00, errorCode);
    printf("Word: 0x%08X\n", int(status2));
    delay(2000);*/

    /*my_epos.go_to_position(-50000);
    Serial.println("pos -50000");
    delay(1000);
    my_epos.go_to_position(50000);
    Serial.println("pos 50000");
    delay(1000);*/
}