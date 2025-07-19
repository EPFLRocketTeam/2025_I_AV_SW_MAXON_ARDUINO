#include "epos4.h"

// Object Dictionary entries for EPOS4
namespace
{
    constexpr BYTE NODE_ID = 0x01;
    constexpr WORD TARGET_POSITION_INDEX = 0x607A;
    constexpr BYTE TARGET_POSITION_SUBINDEX = 0x00;

    constexpr WORD CONTROL_WORD_INDEX = 0x6040;
    constexpr BYTE CONTROL_WORD_SUBINDEX = 0x00;

    constexpr WORD STATUS_WORD_INDEX = 0x6041;
    constexpr BYTE STATUS_WORD_SUBINDEX = 0x00;

    constexpr WORD OPERATION_MODE_INDEX = 0x6060;
    constexpr BYTE OPERATION_MODE_SUBINDEX = 0x00;
}

namespace
{
    constexpr DWORD CONTROL_WORD_DISABLE = 0x0006;
    constexpr DWORD CONTROL_WORD_ENABLE = 0x000F;
    constexpr DWORD CONTROL_WORD_TRIGGER = 0x003F;
}

EPOS4::EPOS4(HardwareSerial &eposSerial, unsigned long baudrate) : eposSerial(eposSerial), baudrate(baudrate), read_timeout(500)
{
    eposSerial.begin(baudrate);
}

void EPOS4::writeObject(BYTE nodeID, WORD index, BYTE sub_index, const DWORD& value, DWORD& errorCode)
{
    std::vector<uint8_t> data;
    std::vector<uint8_t> response;

    // nodeID
    data.push_back(nodeID);
    // index in little-endian order
    data.push_back(static_cast<uint8_t>(index & 0xFF));        // Low byte
    data.push_back(static_cast<uint8_t>((index >> 8) & 0xFF)); // High byte
    // Sub-index
    data.push_back(sub_index); 
    // Value in little-endian order
    data.push_back(static_cast<uint8_t>(value & 0xFF));         // Byte 0
    data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));  // Byte 1
    data.push_back(static_cast<uint8_t>((value >> 16) & 0xFF)); // Byte 2
    data.push_back(static_cast<uint8_t>((value >> 24) & 0xFF)); // Byte 3

    std::vector<uint8_t> frame = buildFrame(0x68, data);
    sendFrame(frame);

    /*
    * Response
    */
    unsigned long startTime = millis();
    while (!eposSerial.available()) 
    {
        if (millis() - startTime > read_timeout) 
        {
            errorCode = 0x0001;
            return;
        }
    }

    while (eposSerial.available()) 
    {
        uint8_t b = eposSerial.read();
        response.push_back(b);
    }

    /*
    ** TODO: get error code
    */
}

void EPOS4::go_to_position(const DWORD& position)
{
    DWORD errorCode = 0x0000;
    writeObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_ENABLE, errorCode);
    writeObject(NODE_ID, TARGET_POSITION_INDEX, TARGET_POSITION_SUBINDEX, position, errorCode);
    writeObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_TRIGGER, errorCode);
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