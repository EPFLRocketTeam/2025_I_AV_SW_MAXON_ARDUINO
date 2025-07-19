#ifndef EPOS4_H
#define EPOS4_H

#include <Arduino.h>
#include <vector>

typedef int8_t BYTE;
typedef int16_t WORD;
typedef int32_t DWORD;

class EPOS4 
{
public:
    EPOS4(HardwareSerial &eposSerial, unsigned long baudrate = 115200);

    void writeObject(BYTE nodeID, WORD index, BYTE sub_index, const DWORD& value, DWORD& errorCode);
    //void readObject();

    void go_to_position(const DWORD& position);
private:
    HardwareSerial &eposSerial;
    unsigned long baudrate;

    uint16_t calcCRC(uint16_t* dataArray, uint8_t numWords);
    void addStuffedByte(std::vector<uint8_t> &frame, uint8_t byte);
    std::vector<uint8_t> buildFrame(uint8_t opcode, const std::vector<uint8_t> &data);
    void sendFrame(const std::vector<uint8_t> &frame);
};

#endif