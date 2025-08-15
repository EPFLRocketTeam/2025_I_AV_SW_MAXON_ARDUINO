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

    constexpr WORD HOMING_METHOD_INDEX = 0x6098;
    constexpr BYTE HOMING_METHOD_SUBINDEX = 0x00;

    constexpr WORD HOME_POSITION_INDEX = 0x30B0;
    constexpr BYTE HOME_POSITION_SUBINDEX = 0x00;
    
    constexpr WORD HOME_OFFSET_MOVE_DISTANCE_INDEX = 0x30B1;
    constexpr BYTE HOME_OFFSET_MOVE_DISTANCE_SUBINDEX = 0x00;
    
    constexpr WORD CURRENT_THRESHOLD_INDEX = 0x30B2;
    constexpr BYTE CURRENT_THRESHOLD_SUBINDEX = 0x00;

    constexpr WORD SPEED_FOR_SWITCH_SEARCH_INDEX = 0x6099;
    constexpr BYTE SPEED_FOR_SWITCH_SEARCH_SUBINDEX = 0x02;

    constexpr WORD HOMING_ACCELERATION_INDEX = 0x609A;
    constexpr BYTE HOMING_ACCELERATION_SUBINDEX = 0x00;
    
    constexpr WORD MODE_OF_OPERATION_INDEX = 0x6060;
    constexpr BYTE MODE_OF_OPERATION_SUBINDEX = 0x00;
    
    constexpr WORD MODE_OF_OPERATION_DISPLAY_INDEX = 0x6061;
    constexpr BYTE MODE_OF_OPERATION_DISPLAY_SUBINDEX = 0x00;
}

namespace
{
    constexpr DWORD CONTROL_WORD_DISABLE_VOLTAGE = 0x0000;
    constexpr DWORD CONTROL_WORD_SHUTDOWN = 0x0006;
    constexpr DWORD CONTROL_WORD_SWITCH_ON = 0x0007;
    constexpr DWORD CONTROL_WORD_ENABLE_OPERATION = 0x000F;
    constexpr DWORD CONTROL_WORD_QUICK_STOP = 0x000B;
    constexpr DWORD CONTROL_WORD_FAULT_RESET = 0x0080;
    constexpr DWORD CONTROL_WORD_TRIGGER = 0x003F;

    // Extras in OP-enabled
    constexpr DWORD HALT             = 0x0100; // bit 8
    constexpr DWORD NEW_SETPOINT     = 0x0010; // bit 4 (PPM: pulse)
    constexpr DWORD CHANGE_IMMED     = 0x0020; // bit 5
    constexpr DWORD RELATIVE         = 0x0040; // bit 6 (0=absolute,1=relative)
}

namespace status_word_constants
{
    constexpr DWORD STATUS_WORD_HOMING = 0x6041;
}

namespace
{
    constexpr DWORD OPERATION_MODE_PROFILE_POSITION = 1; // PPM
    constexpr DWORD OPERATION_MODE_PROFILE_VELOCITY = 3; // PVM
    constexpr DWORD OPERATION_MODE_HOMING = 6; // HMM
    constexpr DWORD OPERATION_MODE_CYCLIC_SYNCHRONOUS_POSITION = 8; // CSP
    constexpr DWORD OPERATION_MODE_CYCLIC_SYNCHRONOUS_VELOCITY = 9; // CSV
    constexpr DWORD OPERATION_MODE_CYCLIC_SYNCHRONOUS_TORQUE = 10; // CST
}

namespace homing_constants
{
    constexpr DWORD HOMING_METHOD_CURRENT_THRESHOLD = -3;
    constexpr DWORD HOME_POSITION = 0;
    constexpr DWORD CURRENT_THRESHOLD = 500; // [mA]
    constexpr DWORD SPEED_FOR_SWITCH_SEARCH = 500; // [rpm]
    constexpr DWORD HOMING_ACCELERATION = 5000; // [rpm/s]
}

EPOS4::EPOS4(HardwareSerial &eposSerial, unsigned long baudrate) : 
    eposSerial(eposSerial), baudrate(baudrate), read_timeout(500), homing_timeout(10000)
{
    eposSerial.begin(baudrate);
}

void EPOS4::writeObject(BYTE nodeID, WORD index, BYTE sub_index, const DWORD& value, DWORD& errorCode)
{
    std::vector<uint8_t> data;
    std::vector<uint8_t> response;

    // nodeID
    data.push_back(nodeID);
    // index/sub-index in little-endian order
    data.push_back(static_cast<uint8_t>(index & 0xFF));        // Low byte
    data.push_back(static_cast<uint8_t>((index >> 8) & 0xFF)); // High byte
    // Sub-index
    data.push_back(sub_index); 
    // Value in little-endian order
    data.push_back(static_cast<uint8_t>(value & 0xFF));         // Byte 0
    data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));  // Byte 1
    data.push_back(static_cast<uint8_t>((value >> 16) & 0xFF)); // Byte 2
    data.push_back(static_cast<uint8_t>((value >> 24) & 0xFF)); // Byte 3
    // Send frame
    std::vector<uint8_t> frame = buildFrame(0x68, data); // writeObject op code -> 0x68
    sendFrame(frame);
    
    // Wait for response
    unsigned long startTime = millis();
    while (eposSerial.available() < 10) // wait for expected response size
    {
        if (millis() - startTime > read_timeout) 
        {
            Serial.println("[readObject] Timeout waiting for response");
            errorCode = 0x0001; // homemade error code
            return;
        }
    }
    // Read response
    while (eposSerial.available()) 
    {
        uint8_t b = eposSerial.read();
        Serial.print(" 0x");
        Serial.print(b, HEX);
        response.push_back(b);
    }
    Serial.println();
    // Check if response is valid (size = 6, op code = 0x00, len = 2)
    if (response.size() < 10 || response[2] != 0x00 || response[3] != 0x02) 
    {
        Serial.println("[writeObject] Invalid response");
        errorCode = 0x0002; // homemade error code
        return;
    }
    // Extract error code from response
    errorCode = (static_cast<uint32_t>(response[4]) << 0 ) |
                (static_cast<uint32_t>(response[5]) << 8 ) |
                (static_cast<uint32_t>(response[6]) << 16) |
                (static_cast<uint32_t>(response[7]) << 24);
}

void EPOS4::startWriteObject(BYTE nodeID, WORD index, BYTE sub_index, const DWORD& value)
{
    std::vector<uint8_t> data;

    // nodeID
    data.push_back(nodeID);
    // index/sub-index in little-endian order
    data.push_back(static_cast<uint8_t>(index & 0xFF));        // Low byte
    data.push_back(static_cast<uint8_t>((index >> 8) & 0xFF)); // High byte
    // Sub-index
    data.push_back(sub_index); 
    // Value in little-endian order
    data.push_back(static_cast<uint8_t>(value & 0xFF));         // Byte 0
    data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));  // Byte 1
    data.push_back(static_cast<uint8_t>((value >> 16) & 0xFF)); // Byte 2
    data.push_back(static_cast<uint8_t>((value >> 24) & 0xFF)); // Byte 3
    // Send frame
    std::vector<uint8_t> frame = buildFrame(0x68, data); // writeObject op code -> 0x68
    sendFrame(frame);
}

bool EPOS4::pollWriteObject(DWORD& errorCode)
{
    constexpr unsigned response_length = 10;
    std::vector<uint8_t> response;

    if (eposSerial.available() < response_length) // check for expected response size
        return false;

    // Read response
    while (eposSerial.available()) 
    {
        uint8_t b = eposSerial.read();
        Serial.print(" 0x");
        Serial.print(b, HEX);
        response.push_back(b);
    }
    Serial.println();
    // Check if response is valid (size = 6, op code = 0x00, len = 2)
    if (response.size() != response_length || response[2] != 0x00 || response[3] != 0x02) 
    {
        Serial.println("[writeObject] Invalid response");
        errorCode = 0x0002; // homemade error code
        return false;
    }
    // Extract error code from response
    errorCode = (static_cast<uint32_t>(response[4]) << 0 ) |
                (static_cast<uint32_t>(response[5]) << 8 ) |
                (static_cast<uint32_t>(response[6]) << 16) |
                (static_cast<uint32_t>(response[7]) << 24);

    return true;
}

DWORD EPOS4::readObject(BYTE nodeID, WORD index, BYTE sub_index, DWORD& errorCode)
{
    std::vector<uint8_t> data;
    std::vector<uint8_t> response;

    // nodeID
    data.push_back(nodeID);
    // index/sub-index in little-endian order
    data.push_back(static_cast<uint8_t>(index & 0xFF));        // Low byte
    data.push_back(static_cast<uint8_t>((index >> 8) & 0xFF)); // High byte
    // Sub-index
    data.push_back(sub_index); 
    // Send frame
    std::vector<uint8_t> frame = buildFrame(0x60, data); // readObject op code -> 0x60
    sendFrame(frame);

    // Wait for response
    unsigned long startTime = millis();
    while (eposSerial.available() < 14) // wait for expected response size
    {
        if (millis() - startTime > read_timeout) 
        {
            Serial.println("[readObject] Timeout waiting for response");
            errorCode = 0x0001; // homemade error code
            return 0;
        }
    }
    // Read response
    while (eposSerial.available()) 
    {
        uint8_t b = eposSerial.read();
#ifdef DEBUG
        Serial.print(" 0x");
        Serial.print(b, HEX);
#endif
        response.push_back(b);
    }
    // Serial.println();
    // Check if response is valid (size = 6, op code = 0x00, len = 4)
    if (response.size() < 14 || response[2] != 0x00 || response[3] != 0x04) 
    {
        Serial.println("[readObject] Invalid response");
        errorCode = 0x0002; // homemade error code
        return 0; // Return 0 on error
    }
    // Extract error code from response
    errorCode = (static_cast<uint32_t>(response[4]) << 0 ) |
                (static_cast<uint32_t>(response[5]) << 8 ) |
                (static_cast<uint32_t>(response[6]) << 16) |
                (static_cast<uint32_t>(response[7]) << 24);
    // Extract value from response
    DWORD value =   (static_cast<uint32_t>(response[8]) << 0 ) |
                    (static_cast<uint32_t>(response[9]) << 8 ) |
                    (static_cast<uint32_t>(response[10]) << 16) |
                    (static_cast<uint32_t>(response[11]) << 24);
    return value;
}

void EPOS4::startReadObject(BYTE nodeID, WORD index, BYTE sub_index)
{
    std::vector<uint8_t> data;

    // nodeID
    data.push_back(nodeID);
    // index/sub-index in little-endian order
    data.push_back(static_cast<uint8_t>(index & 0xFF));        // Low byte
    data.push_back(static_cast<uint8_t>((index >> 8) & 0xFF)); // High byte
    // Sub-index
    data.push_back(sub_index); 
    // Send frame
    std::vector<uint8_t> frame = buildFrame(0x60, data); // readObject op code -> 0x60
    sendFrame(frame);
}

bool EPOS4::pollReadObject(DWORD& value, DWORD& errorCode)
{
    constexpr unsigned response_length = 14;
    std::vector<uint8_t> response;

    if (eposSerial.available() < response_length) // check for expected response size
        return false;

    // Read response
    while (eposSerial.available()) 
    {
        uint8_t b = eposSerial.read();
#ifdef DEBUG
        Serial.print(" 0x");
        Serial.print(b, HEX);
#endif
        response.push_back(b);
    }
    // Serial.println();
    // Check if response is valid (size = 6, op code = 0x00, len = 4)
    if (response.size() < response_length || response[2] != 0x00 || response[3] != 0x04) 
    {
        Serial.println("[readObject] Invalid response");
        errorCode = 0x0002; // homemade error code
        return false; // Return 0 on error
    }
    // Extract error code from response
    errorCode = (static_cast<uint32_t>(response[4]) << 0 ) |
                (static_cast<uint32_t>(response[5]) << 8 ) |
                (static_cast<uint32_t>(response[6]) << 16) |
                (static_cast<uint32_t>(response[7]) << 24);
    // Extract value from response
    value   =   (static_cast<uint32_t>(response[8]) << 0 ) |
                (static_cast<uint32_t>(response[9]) << 8 ) |
                (static_cast<uint32_t>(response[10]) << 16) |
                (static_cast<uint32_t>(response[11]) << 24);
    return true;
}

void EPOS4::go_to_position(const DWORD& position)
{
    /*
    DWORD errorCode = 0x0000;
    writeObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_ENABLE, errorCode);
    delay(20); // TODO check status word instead of waiting
    writeObject(NODE_ID, TARGET_POSITION_INDEX, TARGET_POSITION_SUBINDEX, position, errorCode);
    delay(20);
    writeObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_TRIGGER, errorCode);
    delay(20);*/
}

void EPOS4::current_threshold_homing(DWORD home_offset_move_distance)
{
    /*
    DWORD errorCode = 0x0000;
    unsigned long homingStart = millis();

    writeObject(NODE_ID, MODE_OF_OPERATION_DISPLAY_INDEX, MODE_OF_OPERATION_DISPLAY_SUBINDEX, mode_of_operation_constants::MODE_OF_OPERATION_HOMING, errorCode);
    delay(40);
    writeObject(NODE_ID, HOMING_METHOD_INDEX, HOMING_METHOD_SUBINDEX, homing_constants::HOMING_METHOD_CURRENT_THRESHOLD, errorCode);
    delay(40);
    writeObject(NODE_ID, HOME_POSITION_INDEX, HOME_POSITION_SUBINDEX, homing_constants::HOME_POSITION, errorCode);
    delay(40);
    writeObject(NODE_ID, HOME_OFFSET_MOVE_DISTANCE_INDEX, HOME_OFFSET_MOVE_DISTANCE_SUBINDEX, home_offset_move_distance, errorCode);
    delay(40);
    writeObject(NODE_ID, CURRENT_THRESHOLD_INDEX, CURRENT_THRESHOLD_SUBINDEX, homing_constants::CURRENT_THRESHOLD, errorCode);
    delay(40);
    writeObject(NODE_ID, SPEED_FOR_SWITCH_SEARCH_INDEX, SPEED_FOR_SWITCH_SEARCH_SUBINDEX, homing_constants::SPEED_FOR_SWITCH_SEARCH, errorCode);
    delay(40);
    writeObject(NODE_ID, HOMING_ACCELERATION_INDEX, HOMING_ACCELERATION_SUBINDEX, homing_constants::HOMING_ACCELERATION, errorCode);
    delay(40);
    writeObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_ENABLE, errorCode);

    STATUS status(readObject(NODE_ID, STATUS_WORD_INDEX, STATUS_WORD_SUBINDEX, errorCode));
    while(not status.homingAttained())
    {
        status = STATUS(readObject(NODE_ID, STATUS_WORD_INDEX, STATUS_WORD_SUBINDEX, errorCode));
        if(errorCode != 0x0000)
        {
            Serial.println("[current_threshold_homing] Error reading status word, error code: " + String(errorCode, HEX));
            errorCode = 0x0001; // homemade error code            
            return; // Handle error accordingly
        }
        if(status.HomingError())
        {
            Serial.println("[current_threshold_homing] Homing error detected");
            errorCode = 0x0002; // homemade error code
            return; // Handle error accordingly
        }
        // Timeout for homing (e.g., 10 seconds)
        if (millis() - homingStart > homing_timeout)
        {
            Serial.println("[current_threshold_homing] Homing timeout reached");
            errorCode = 0x0003; // homemade error code
            return;
        }
    }
    */
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
    {
        eposSerial.write(b);
#ifdef DEBUG
        Serial.print(" 0x");
        Serial.print(b, HEX);
#endif
    }
#ifdef DEBUG
    Serial.println();
#endif
}