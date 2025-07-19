#ifndef EPOS4_H
#define EPOS4_H

#include <Arduino.h>
#include <vector>

class EPOS4 
{
public:
    EPOS4(HardwareSerial &eposSerial, unsigned long baudrate = 115200);

private:
    HardwareSerial &eposSerial;
    unsigned long baudrate;

    uint16_t calcCRC(uint16_t* dataArray, uint8_t numWords);
    void addStuffedByte(std::vector<uint8_t> &frame, uint8_t byte);
    std::vector<uint8_t> buildFrame(uint8_t opcode, const std::vector<uint8_t> &data);
    void sendFrame(const std::vector<uint8_t> &frame);
};

#endif