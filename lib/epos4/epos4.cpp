/*
    File: epos.cpp
    Author: Axel Juaneda
    Organization: EPFL Rocket Team
    Version : 1.0
*/

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
    
    constexpr WORD HOMING_CURRENT_INDEX = 0x30B2;
    constexpr BYTE HOMING_CURRENT_SUBINDEX = 0x00;

    constexpr WORD SPEED_FOR_SWITCH_SEARCH_INDEX = 0x6099;
    constexpr BYTE SPEED_FOR_SWITCH_SEARCH_SUBINDEX = 0x01;

    constexpr WORD SPEED_FOR_ZERO_SEARCH_INDEX = 0x6099;
    constexpr BYTE SPEED_FOR_ZERO_SEARCH_SUBINDEX = 0x02;

    constexpr WORD HOMING_ACCELERATION_INDEX = 0x609A;
    constexpr BYTE HOMING_ACCELERATION_SUBINDEX = 0x00;
    
    constexpr WORD MODE_OF_OPERATION_INDEX = 0x6060;
    constexpr BYTE MODE_OF_OPERATION_SUBINDEX = 0x00;
    
    constexpr WORD MODE_OF_OPERATION_DISPLAY_INDEX = 0x6061;
    constexpr BYTE MODE_OF_OPERATION_DISPLAY_SUBINDEX = 0x00;

    constexpr WORD PROFILE_VELOCITY_INDEX = 0x6081;
    constexpr BYTE PROFILE_VELOCITY_SUBINDEX = 0x00;
}

namespace
{
    constexpr DWORD CONTROL_WORD_DISABLE_VOLTAGE = 0x0000;
    constexpr DWORD CONTROL_WORD_SHUTDOWN = 0x0006;
    constexpr DWORD CONTROL_WORD_SWITCH_ON = 0x0007;
    constexpr DWORD CONTROL_WORD_ENABLE_OPERATION = 0x000F;
    constexpr DWORD CONTROL_WORD_QUICK_STOP = 0x000B;
    constexpr DWORD CONTROL_WORD_FAULT_RESET = 0x0080;
    constexpr DWORD CONTROL_WORD_TOGGLE = 0x003F;
    constexpr DWORD CONTROL_WORD_START_HOMING = 0x003F;
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

namespace
{
    constexpr DWORD HOMING_METHOD_CURRENT_THRESHOLD = -3;
}

EPOS4::EPOS4(HardwareSerial &eposSerial, unsigned long baudrate) : 
    eposSerial(eposSerial), baudrate(baudrate), read_timeout(500), isReading(false), isWriting(false), isReadingStatus(false), timeout(false),
    driver_state(DriverState::IDLE), ppm_state(PPMState::SET_OPERATION_MODE), homing_state(HomingState::SET_OPERATION_MODE)
{
    eposSerial.begin(baudrate);
}

void EPOS4::tick()
{
    DWORD errorCode = 0x0000;
    DWORD statusWord = 0x0000;

    if(isReadingStatus && pollReadObject(statusWord, errorCode))
    {
        epos_status = STATUS(statusWord);
        isReadingStatus = false;
    }
    else if (!get_isWriting() && !get_isReading())
    {
        startReadObject(NODE_ID, STATUS_WORD_INDEX, STATUS_WORD_SUBINDEX);
        isReadingStatus = true;
    }

    if (!isReadingStatus)
    {
        switch (driver_state) {

        case DriverState::IDLE:
            // Wait for a start command or event
            break;

        case DriverState::PPM:
            runPPM();
            break;

        case DriverState::HOMING:
            runHoming();
            break;
        case DriverState::FAULT:
            fault();
            break;
        }
    }
    
    if (epos_status.fault() || timeout || errorCode)
    {
        driver_state = DriverState::FAULT;
        isReadingStatus = false;
    }
}

void EPOS4::runPPM()
{
    DWORD errorCode = 0x0000;
    switch (ppm_state)
    {
    case PPMState::SET_OPERATION_MODE:
        Serial.println("PPM_SET_OPERATION_MODE");
        if (!get_isWriting())
            startWriteObject(NODE_ID, OPERATION_MODE_INDEX, OPERATION_MODE_SUBINDEX, OPERATION_MODE_PROFILE_POSITION);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::SET_PROFILE_VELOCITY;
        break;
    
    case PPMState::SET_PROFILE_VELOCITY:
        Serial.println("PPM_SET_PROFILE_VELOCITY");
        if (!get_isWriting())
            startWriteObject(NODE_ID, PROFILE_VELOCITY_INDEX, PROFILE_VELOCITY_SUBINDEX, ppm_cfg.profile_velocity);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::SHUTDOWN;
        break;
    
    case PPMState::SHUTDOWN:
        Serial.println("PPM_SHUTDOWN");
        if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_SHUTDOWN);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::ENABLE;
        break;

    case PPMState::ENABLE:
        Serial.println("PPM_ENABLE");
        if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_ENABLE_OPERATION);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::SET_TARGET_POSITION;
        break;
    
    case PPMState::SET_TARGET_POSITION:
        Serial.println("PPM_SET_TARGET_POSITION");
        if (!get_isWriting())
            startWriteObject(NODE_ID, TARGET_POSITION_INDEX, TARGET_POSITION_SUBINDEX, ppm_cfg.target_position);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::TOGGLE;
        break;
    
    case PPMState::TOGGLE:
        Serial.println("PPM_TOGGLE");
        if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_TOGGLE);
        else if (pollWriteObject(errorCode))
        {
            ppm_state = PPMState::ENABLE;
            driver_state = DriverState::IDLE;
        }
        break;
    }
}

void EPOS4::runHoming()
{
    DWORD errorCode = 0x0000;
    switch (homing_state)
    {
    case HomingState::SET_OPERATION_MODE:
        if (!get_isWriting())
            startWriteObject(NODE_ID, OPERATION_MODE_INDEX, OPERATION_MODE_SUBINDEX, OPERATION_MODE_HOMING);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_SPEED_FOR_SWITCH_SEARCH;
        break;

    case HomingState::SET_SPEED_FOR_SWITCH_SEARCH:
        if (!get_isWriting())
            startWriteObject(NODE_ID, SPEED_FOR_SWITCH_SEARCH_INDEX, SPEED_FOR_SWITCH_SEARCH_SUBINDEX, homing_cfg.speed_for_switch_search);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_SPEED_FOR_ZERO_SEARCH;
        break;
    
    case HomingState::SET_SPEED_FOR_ZERO_SEARCH:
        if (!get_isWriting())
            startWriteObject(NODE_ID, SPEED_FOR_ZERO_SEARCH_INDEX,  SPEED_FOR_ZERO_SEARCH_SUBINDEX, homing_cfg.speed_for_zero_search);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_HOMING_ACCELERATION;
        break;

    case HomingState::SET_HOMING_ACCELERATION:
        if (!get_isWriting())
            startWriteObject(NODE_ID, HOMING_ACCELERATION_INDEX, HOMING_ACCELERATION_SUBINDEX, homing_cfg.homing_acceleration);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_HOMING_CURRENT;
        break;

    case HomingState::SET_HOMING_CURRENT:
        if (!get_isWriting())
            startWriteObject(NODE_ID, HOMING_CURRENT_INDEX, HOMING_CURRENT_SUBINDEX, homing_cfg.homing_current);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_OFFSET_MOVE_DISTANCE;
        break;

    case HomingState::SET_OFFSET_MOVE_DISTANCE:
        if (!get_isWriting())
            startWriteObject(NODE_ID, HOME_OFFSET_MOVE_DISTANCE_INDEX, HOME_OFFSET_MOVE_DISTANCE_SUBINDEX, homing_cfg.homing_offset_distance);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_HOME_POSITION;
        break;

    case HomingState::SET_HOME_POSITION:
        if (!get_isWriting())
            startWriteObject(NODE_ID, HOME_POSITION_INDEX, HOME_POSITION_SUBINDEX, homing_cfg.home_position);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_HOMING_METHOD;
        break;

    case HomingState::SET_HOMING_METHOD:
        if (!get_isWriting())
            startWriteObject(NODE_ID, HOMING_METHOD_INDEX, HOMING_METHOD_SUBINDEX, HOMING_METHOD_CURRENT_THRESHOLD);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SHUTDOWN;
        break;
    case HomingState::SHUTDOWN:
        if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_SHUTDOWN);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::ENABLE;
        break;
    case HomingState::ENABLE:
        if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_ENABLE_OPERATION);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::START_HOMING;
        break;
    case HomingState::START_HOMING:
        if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_START_HOMING);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::DONE;
        break;
    case HomingState::DONE:
        /* code */
        break;
    
    }
}

void EPOS4::fault()
{
    Serial.println("Fault");
    timeout = false;
    ppm_state = PPMState::SET_OPERATION_MODE;
    homing_state = HomingState::SET_OPERATION_MODE;

    while (eposSerial.available()) // flush buffer
        eposSerial.read();
    
    DWORD errorCode = 0x0000;

    if (epos_status.fault())
    {
        if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_FAULT_RESET);
        else if (pollWriteObject(errorCode))
            driver_state = DriverState::IDLE;
    }
    else
        driver_state = DriverState::IDLE;
}

void EPOS4::go_to_position(const DWORD position)
{
    ppm_cfg.target_position = position;
    driver_state = DriverState::PPM;
}

void EPOS4::current_threshold_homing()
{
    driver_state = DriverState::HOMING;
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
    isWriting = true;
    startTime = millis();
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

    if (!isWriting)
        return false;

    if (millis() - startTime > read_timeout) 
    {
        Serial.println("[writeObject] Timeout waiting for response");
        isWriting = false;
        timeout = true;
        errorCode = 0x0001; // homemade timeout error code
        return false;
    }

    if (eposSerial.available() < response_length) // check for expected response size
        return false;

    isWriting = false;
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
    isReading = true;
    startTime = millis();
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

    if (!isReading)
        return false;

    if (millis() - startTime > read_timeout) 
    {
        Serial.println("[readObject] Timeout waiting for response");
        isReading = false;
        timeout = true;
        errorCode = 0x0001; // homemade timeout error code
        return false;
    }
    
    if (eposSerial.available() < response_length) // check for expected response size
        return false;
    
    isReading = false;
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