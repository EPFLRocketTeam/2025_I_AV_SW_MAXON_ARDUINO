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

    constexpr WORD OPERATION_MODE_DISPLAY_INDEX = 0x6061;
    constexpr BYTE OPERATION_MODE_DISPLAY_SUBINDEX = 0x00;

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

    constexpr WORD PROFILE_ACCELERATION_INDEX = 0x6083;
    constexpr BYTE PROFILE_ACCELERATION_SUBINDEX = 0x00;

    constexpr WORD PROFILE_DECELERATION_INDEX = 0x6084;
    constexpr BYTE PROFILE_DECELERATION_SUBINDEX = 0x00;

    constexpr WORD MOTOR_DATA_INDEX = 0x3001;
    constexpr BYTE NOMINAL_CURRENT_SUBINDEX = 0x01; 
    constexpr BYTE OUTPUT_CURRENT_LIMIT_SUBINDEX = 0x02;
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
    constexpr DWORD HOMING_METHOD_CURRENT_THRESHOLD_NEGATIVE = -4;
    
}

EPOS4::EPOS4(HardwareSerial &eposSerial, unsigned long baudrate) : 
    eposSerial(eposSerial), baudrate(baudrate), read_timeout(500), min_tick_time(1), startTime(0), last_tick_time(0)
{
    eposSerial.begin(baudrate);
    reset();
}

void EPOS4::reset()
{
    //Serial.println("Reset");

    while (eposSerial.available()) // flush buffer
            eposSerial.read();

    isReading = false;
    isWriting = false;
    timeout = false;
    homing_done = false;
    observed_mode = 0x0000;
    driver_state = DriverState::IDLE;
    working_state = DriverState::IDLE;
    ppm_state = PPMState::SET_OPERATION_MODE;
    homing_state = HomingState::SET_OPERATION_MODE;
    epos_status = 0x0000;
}

void EPOS4::tick()
{
    DWORD errorCode = 0x0000;
    DWORD status_word = 0x0000;

    if (millis() - last_tick_time < min_tick_time)
        return;
    
    switch (driver_state) 
    {
    case DriverState::IDLE:
        //Serial.println("IDLE");
        if (!get_isWriting() || !get_isReading())
            driver_state = DriverState::READ_STATUS;
        break;

    case DriverState::READ_STATUS:
        //Serial.println("READ_STATUS");
        if (!get_isReading())
            startReadObject(NODE_ID, STATUS_WORD_INDEX, STATUS_WORD_SUBINDEX);
        else if (pollReadObject(status_word, errorCode))
        {
            epos_status = status_word;
            driver_state = working_state;
        }
        break;

    case DriverState::PPM:
        //Serial.println("PPM");
        runPPM();
        if (!get_isWriting() && !get_isReading())
            driver_state = DriverState::READ_STATUS;
        break;

    case DriverState::HOMING:
        //Serial.println("HOMING");
        runHoming();
        if (!get_isWriting() && !get_isReading())
            driver_state = DriverState::READ_STATUS;
        break;
    case DriverState::FAULT:
        //Serial.println("FAULT");
        fault();
        if (!get_isWriting() && !get_isReading())
            driver_state = DriverState::READ_STATUS;
        break;
    }
    
    if (timeout || errorCode)
        reset();
    
    if (epos_status.fault())
    {
        reset();
        driver_state = DriverState::FAULT;
    }
    last_tick_time = millis();
}

void EPOS4::runPPM()
{
    DWORD errorCode = 0x0000;
    switch (ppm_state)
    {
    case PPMState::SET_OPERATION_MODE:
        //Serial.println("PPM_SET_OPERATION_MODE");
        if (!get_isWriting())
            startWriteObject(NODE_ID, OPERATION_MODE_INDEX, OPERATION_MODE_SUBINDEX, OPERATION_MODE_PROFILE_POSITION);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::CHECK_OPERATION_MODE;
        break;
    
    case PPMState::CHECK_OPERATION_MODE:
        //Serial.println("PPM_CHECK_OPERATION_MODE");
        if (observed_mode != OPERATION_MODE_PROFILE_POSITION)
        {
            DWORD response = 0x0000;
            if (!get_isReading())
                startReadObject(NODE_ID, OPERATION_MODE_DISPLAY_INDEX, OPERATION_MODE_DISPLAY_SUBINDEX);
            else 
            {
                pollReadObject(response, errorCode);
                observed_mode = response;
            }
        }
        else
            ppm_state = PPMState::SET_PROFILE_VELOCITY;
        break;

    case PPMState::SET_PROFILE_VELOCITY:
        //Serial.println("PPM_SET_PROFILE_VELOCITY");
        if (!get_isWriting())
            startWriteObject(NODE_ID, PROFILE_VELOCITY_INDEX, PROFILE_VELOCITY_SUBINDEX, ppm_cfg.profile_velocity);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::SET_PROFILE_ACCELERATION;
        
        break;
    case PPMState::SET_PROFILE_ACCELERATION:
        Serial.println("setting acceleration");
        if (!get_isWriting())
            startWriteObject(NODE_ID, PROFILE_ACCELERATION_INDEX, PROFILE_ACCELERATION_SUBINDEX, ppm_cfg.profile_acceleration);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::SET_PROFILE_DECELERATION;
        break;
    case PPMState::SET_PROFILE_DECELERATION:
        Serial.println("setting decel");
        if (!get_isWriting())
            startWriteObject(NODE_ID, PROFILE_DECELERATION_INDEX, PROFILE_DECELERATION_SUBINDEX, ppm_cfg.profile_deceleration);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::SET_NOMINAL_CURRENT;
        break;
    case PPMState::SET_NOMINAL_CURRENT:
        Serial.println("setting nominal current");
        if (!get_isWriting())
            startWriteObject(NODE_ID, MOTOR_DATA_INDEX, NOMINAL_CURRENT_SUBINDEX, ppm_cfg.nominal_current);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::SET_OUTPUT_CURRENT_LIMIT;
        break;
    case PPMState::SET_OUTPUT_CURRENT_LIMIT:
        Serial.println("setting current limit");
        if (!get_isWriting())
            startWriteObject(NODE_ID, MOTOR_DATA_INDEX, OUTPUT_CURRENT_LIMIT_SUBINDEX, ppm_cfg.output_current_limit);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::SHUTDOWN;
        break;
    case PPMState::SHUTDOWN:
        //Serial.println("PPM_SHUTDOWN");
        if (epos_status.readyToSwitchOn())
            ppm_state = PPMState::ENABLE;
        else if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_SHUTDOWN);
        else 
            pollWriteObject(errorCode);
        break;

    case PPMState::ENABLE:
        //Serial.println("PPM_ENABLE");
            
        if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_ENABLE_OPERATION);
        else if (pollWriteObject(errorCode))
            ppm_state = PPMState::SET_TARGET_POSITION;
        break;
    
    case PPMState::SET_TARGET_POSITION:
        //Serial.println("PPM_SET_TARGET_POSITION");
        if (epos_status.operationEnabled())
        {
            if (!get_isWriting())
                startWriteObject(NODE_ID, TARGET_POSITION_INDEX, TARGET_POSITION_SUBINDEX, ppm_cfg.target_position);
            else if (pollWriteObject(errorCode))
                ppm_state = PPMState::TOGGLE;
        }
        break;
    
    case PPMState::TOGGLE:
        //Serial.println("PPM_TOGGLE");
        if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_TOGGLE);
        else if (pollWriteObject(errorCode))
        {
            ppm_state = PPMState::ENABLE;
            working_state = DriverState::IDLE;
        }
        break;
    }
}

void EPOS4::runHoming(bool direction)
{
    DWORD errorCode = 0x0000;
    switch (homing_state)
    {
    case HomingState::SET_OPERATION_MODE:
        //Serial.println("SET_OPERATION_MODE");
        if (!get_isWriting())
            startWriteObject(NODE_ID, OPERATION_MODE_INDEX, OPERATION_MODE_SUBINDEX, OPERATION_MODE_HOMING);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_SPEED_FOR_SWITCH_SEARCH;
        break;

    case HomingState::SET_SPEED_FOR_SWITCH_SEARCH:
        //Serial.println("SET_SPEED_FOR_SWITCH_SEARCH");
        if (!get_isWriting())
            startWriteObject(NODE_ID, SPEED_FOR_SWITCH_SEARCH_INDEX, SPEED_FOR_SWITCH_SEARCH_SUBINDEX, homing_cfg.speed_for_switch_search);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_SPEED_FOR_ZERO_SEARCH;
        break;
    
    case HomingState::SET_SPEED_FOR_ZERO_SEARCH:
        //Serial.println("SET_SPEED_FOR_ZERO_SEARCH");
        if (!get_isWriting())
            startWriteObject(NODE_ID, SPEED_FOR_ZERO_SEARCH_INDEX,  SPEED_FOR_ZERO_SEARCH_SUBINDEX, homing_cfg.speed_for_zero_search);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_HOMING_ACCELERATION;
        break;

    case HomingState::SET_HOMING_ACCELERATION:
        //Serial.println("SET_HOMING_ACCELERATION");
        if (!get_isWriting())
            startWriteObject(NODE_ID, HOMING_ACCELERATION_INDEX, HOMING_ACCELERATION_SUBINDEX, homing_cfg.homing_acceleration);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_HOMING_CURRENT;
        break;

    case HomingState::SET_HOMING_CURRENT:
        //Serial.println("SET_HOMING_CURRENT");
        if (!get_isWriting())
            startWriteObject(NODE_ID, HOMING_CURRENT_INDEX, HOMING_CURRENT_SUBINDEX, homing_cfg.homing_current);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_OFFSET_MOVE_DISTANCE;
        break;

    case HomingState::SET_OFFSET_MOVE_DISTANCE:
        //Serial.println("SET_OFFSET_MOVE_DISTANCE");
        if (!get_isWriting())
            startWriteObject(NODE_ID, HOME_OFFSET_MOVE_DISTANCE_INDEX, HOME_OFFSET_MOVE_DISTANCE_SUBINDEX, homing_cfg.homing_offset_distance);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_HOME_POSITION;
        break;

    case HomingState::SET_HOME_POSITION:
        //Serial.println("SET_HOME_POSITION");
        if (!get_isWriting())
            startWriteObject(NODE_ID, HOME_POSITION_INDEX, HOME_POSITION_SUBINDEX, homing_cfg.home_position);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SET_HOMING_METHOD;
        break;

    case HomingState::SET_HOMING_METHOD:
        //Serial.println("SET_HOMING_METHOD");
        if (!get_isWriting())
            if(direction){startWriteObject(NODE_ID, HOMING_METHOD_INDEX, HOMING_METHOD_SUBINDEX, HOMING_METHOD_CURRENT_THRESHOLD);}
            else {startWriteObject(NODE_ID, HOMING_METHOD_INDEX, HOMING_METHOD_SUBINDEX, HOMING_METHOD_CURRENT_THRESHOLD_NEGATIVE);}
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::SHUTDOWN;
        break;

    case HomingState::SHUTDOWN:
        //Serial.println("SHUTDOWN");
        if (epos_status.readyToSwitchOn())
            homing_state = HomingState::ENABLE;
        else if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_SHUTDOWN);
        else 
            pollWriteObject(errorCode);
        break;

    case HomingState::ENABLE:
        //Serial.println("ENABLE");
        if (!get_isWriting())
            startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_ENABLE_OPERATION);
        else if (pollWriteObject(errorCode))
            homing_state = HomingState::START_HOMING;
        break;

    case HomingState::START_HOMING:
        if (epos_status.operationEnabled())
        {
            //Serial.println("START_HOMING");
            if (!get_isWriting())
                startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_START_HOMING);
            else if (pollWriteObject(errorCode))
                homing_state = HomingState::IN_PROGRESS;
        }
        break;

    case HomingState::IN_PROGRESS:
        if (epos_status.homingAttained() && epos_status.targetReached())
        {
            homing_done = true;
            working_state = DriverState::IDLE;
            homing_state = HomingState::SET_OPERATION_MODE;
        }
        break;
    }
}

void EPOS4::fault()
{   
    DWORD errorCode = 0x0000;

    //Serial.println("fault reset");
    if (!get_isWriting())
        startWriteObject(NODE_ID, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_FAULT_RESET);
    else if (pollWriteObject(errorCode))
        reset();
}

void EPOS4::go_to_position(const DWORD position)
{
    if (driver_state == DriverState::FAULT)
        return;
    else
    {
        ppm_cfg.target_position = position;
        working_state = DriverState::PPM;
    }
}

void EPOS4::current_threshold_homing()
{
    if (driver_state == DriverState::FAULT)
        return;
    else
    {
        homing_done = false;
        working_state = DriverState::HOMING;
        homing_state = HomingState::SET_OPERATION_MODE;
    }
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
            //Serial.println("[readObject] Timeout waiting for response");
            errorCode = 0x0001; // homemade error code
            return;
        }
    }
    // Read response
    while (eposSerial.available()) 
    {
        uint8_t b = eposSerial.read();
        //Serial.print(" 0x");
        //Serial.print(b, HEX);
        response.push_back(b);
    }
    //Serial.println();
    // Check if response is valid (size = 6, op code = 0x00, len = 2)
    if (response.size() < 10 || response[2] != 0x00 || response[3] != 0x02) 
    {
        //Serial.println("[writeObject] Invalid response");
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
    //Serial.println("[writeObject] Start writing");
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
        //Serial.println("[writeObject] Timeout waiting for response");
        isWriting = false;
        timeout = true;
        errorCode = 0x0001; // homemade timeout error code
        return false;
    }

    //Serial.println("[writeObject] Polling");

    if (eposSerial.available() < response_length) // check for expected response size
        return false;

    isWriting = false;
    // Read response
    while (eposSerial.available()) 
    {
        uint8_t b = eposSerial.read();
        //Serial.print(" 0x");
        //Serial.print(b, HEX);
        response.push_back(b);
    }
    //Serial.println();
    // Check if response is valid (size = 6, op code = 0x00, len = 2)
    if (response.size() != response_length || response[2] != 0x00 || response[3] != 0x02) 
    {
        //Serial.println("[writeObject] Invalid response");
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
            //Serial.println("[readObject] Timeout waiting for response");
            errorCode = 0x0001; // homemade error code
            return 0;
        }
    }
    // Read response
    while (eposSerial.available()) 
    {
        uint8_t b = eposSerial.read();
#ifdef DEBUG
        //Serial.print(" 0x");
        //Serial.print(b, HEX);
#endif
        response.push_back(b);
    }
    // //Serial.println();
    // Check if response is valid (size = 6, op code = 0x00, len = 4)
    if (response.size() < 14 || response[2] != 0x00 || response[3] != 0x04) 
    {
        //Serial.println("[readObject] Invalid response");
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
    //Serial.println("[readObject] Start reading");
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
        //Serial.println("[readObject] Timeout waiting for response");
        isReading = false;
        timeout = true;
        errorCode = 0x0001; // homemade timeout error code
        return false;
    }

    //Serial.println("[readObject] Polling");
    
    if (eposSerial.available() < response_length) // check for expected response size
        return false;
    
    //Serial.println("[readObject] Polled");
    isReading = false;
    // Read response
    while (eposSerial.available()) 
    {
        uint8_t b = eposSerial.read();
#ifdef DEBUG
        //Serial.print(" 0x");
        //Serial.print(b, HEX);
#endif
        response.push_back(b);
    }
    // //Serial.println();
    // Check if response is valid (size = 6, op code = 0x00, len = 4)
    if (response.size() < response_length || response[2] != 0x00 || response[3] != 0x04) 
    {
        //Serial.println("[readObject] Invalid response");
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
        //Serial.print(" 0x");
        //Serial.print(b, HEX);
#endif
    }
#ifdef DEBUG
    //Serial.println();
#endif
}