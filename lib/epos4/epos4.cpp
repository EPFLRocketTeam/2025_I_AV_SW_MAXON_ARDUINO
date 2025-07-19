#include "epos4.h"

EPOS4::EPOS4(HardwareSerial &eposSerial, unsigned long baudrate) : eposSerial(eposSerial), baudrate(baudrate)
{
    eposSerial.begin(baudrate);
}

uint16_t EPOS4::calcCRC(uint16_t* dataArray, uint8_t numWords) 
{
    uint16_t CRC = 0;
    while (numWords--) 
    {
        uint16_t c = *dataArray++;
        uint16_t shifter = 0x8000;
        for (uint8_t i = 0; i < 16; i++) 
        {
            uint16_t carry = CRC & 0x8000;
            CRC <<= 1;
            if (c & shifter) CRC++;
            if (carry) CRC ^= 0x1021;
            shifter >>= 1;
        }
    }
    return CRC;
}

void EPOS4::addStuffedByte(std::vector<uint8_t> &frame, uint8_t byte) 
{
    frame.push_back(byte);
    if (byte == 0x90) frame.push_back(0x90);  // Character stuffing
}

std::vector<uint8_t> EPOS4::buildFrame(uint8_t opcode, const std::vector<uint8_t> &data) 
{
    std::vector<uint8_t> frame;
    uint8_t len = (data.size() + 1) >> 1;

    // Prepare CRC array: [Len<<8 | OpCode, DataWords..., 0x0000]
    std::vector<uint16_t> crcWords;
    crcWords.push_back(((uint16_t)len << 8) | opcode);

    for (size_t i = 0; i < data.size(); i += 2) 
    {
        uint16_t word = data[i];
        if (i + 1 < data.size()) word |= ((uint16_t)data[i + 1]) << 8;
        crcWords.push_back(word);
    }
    
    crcWords.push_back(0x0000);

    uint16_t crc = calcCRC(crcWords.data(), crcWords.size());

    // Start of frame
    frame.push_back(0x90); // DLE
    frame.push_back(0x02); // STX

    // Payload
    addStuffedByte(frame, opcode);
    addStuffedByte(frame, len);

    for (uint8_t byte : data) 
        addStuffedByte(frame, byte);

    addStuffedByte(frame, crc & 0xFF);         // CRC low byte
    addStuffedByte(frame, (crc >> 8) & 0xFF);  // CRC high byte

    return frame;
}

void EPOS4::sendFrame(const std::vector<uint8_t> &frame) 
{
    for (uint8_t b : frame) 
        eposSerial.write(b);
}