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
    
    constexpr WORD POSITION_ACTUAL_VALUE_WORD_INDEX = 0x6064;
    constexpr BYTE POSITION_ACTUAL_VALUE_WORD_SUBINDEX = 0x00;

    constexpr WORD CURRENT_ACTUAL_VALUE_WORD_INDEX = 0x30D1;
    constexpr BYTE CURRENT_ACTUAL_VALUE_WORD_SUBINDEX = 0x02;

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
    // Control Word:
    // 0: Switched on
    // 1: Enable Voltage
    // 2: Quick Stop
    // 3: Enable Operation
    // 4: New Set Point (PPM) / Homing Oparation Start (HMM)
    // 5: Change Set imediately (PPM) / reserved (HMM)
    // 6: Abs or Relative (PPM) / reserved (HMM)
    // 7: Fault reset
    // 8: Halt
    // 9-14: Reserved
    // 15: Endless movement (PPM) / reserved (HMM)


    // Command ( Doc: Firmware specification - 2.2.3)
    constexpr DWORD CONTROL_WORD_SHUTDOWN = 0x0006; // ... 0xxx x110    id:2,6,8
    constexpr DWORD CONTROL_WORD_SWITCH_ON = 0x0007; // ... 0xxx x111    id:3
    constexpr DWORD CONTROL_WORD_SWITCH_ON_AND_ENABLE_OPERATION = 0x000F; // ... 0xxx 1111    id:3,4,(*1)
    constexpr DWORD CONTROL_WORD_DISABLE_VOLTAGE = 0x0000; // ... 0xxx xx0x    id:7,9,10,12
    constexpr DWORD CONTROL_WORD_QUICK_STOP = 0x0002; // ...  0xxx x01x    id:11
    constexpr DWORD CONTROL_WORD_DISABLE_OPERATION = 0x0007; // ... 0xxx 0111    id:5
    constexpr DWORD CONTROL_WORD_ENABLE_OPERATION = 0x000F; // ... 0xxx 1111    id:4,16
    constexpr DWORD CONTROL_WORD_FAULT_RESET = 0x0080; // ... 0xxx xxxx -> 1xxx xxxx    id:14,15

    // Homing specific command ( that keep operation enable )
    constexpr DWORD CONTROL_WORD_START_HOMING = 0x001F; // ... 0001 1111

    // PPM specific command ( that keep operation enable )
    constexpr DWORD CONTROL_WORD_TOGGLE_LOW = 0x02F; // ... 0010 1111
    constexpr DWORD CONTROL_WORD_TOGGLE_HIGH = 0x03F; // ... 0011 1111
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
    epos_position_actual_value = 0;
    epos_current_actual_value = 0;
    read_position_actual_value_queued = false;
    read_current_actual_value_queued = false;
    rx_index = 0;
    memset(rx_buffer, 0, sizeof(rx_buffer));
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
    homing_error = false;
    ppm_error = false;
    homingWriteRetriesCount = 0;
    homingWaitRetriesCount = 0;
    ppmWriteRetriesCount = 0;
    ppmWaitRetriesCount = 0;
    observed_mode = 0x0000;
    driver_state = DriverState::IDLE;
    working_state = DriverState::IDLE;
    ppm_state = PPMState::SET_OPERATION_MODE;
    homing_state = HomingState::SET_OPERATION_MODE;
    epos_status = 0x0000;
}

DriverState EPOS4::determineRead()
{
    switch (driver_state)
    {
        // order: position -> current - > status
        if( !PPMsetupDone ){ return DriverState::READ_STATUS; }
        case DriverState::READ_POSITION_ACTUAL_VALUE:
            if (read_current_actual_value_queued){ return DriverState::READ_CURRENT_ACTUAL_VALUE; }
            // add new read here
            else { return DriverState::READ_STATUS; }

        case DriverState::READ_CURRENT_ACTUAL_VALUE:
            // add new read here
            return DriverState::READ_STATUS;

        // add case new read here

        default:
            if( read_position_actual_value_queued ){ return DriverState::READ_POSITION_ACTUAL_VALUE; }
            else if( read_current_actual_value_queued ){ return DriverState::READ_CURRENT_ACTUAL_VALUE; }
            // add new read here
            else { return DriverState::READ_STATUS; }
    }
}

void EPOS4::readRegisterStep(DriverState nextState, WORD readIndex, BYTE readSubIndex, DWORD& readValue, const char* debugName)
{
    DWORD errorCode = 0x0000;
    DWORD read_word = 0x0000;
    const uint8_t maxRetries = 5;
    long maxReadTime = 200; // 2 seconds

    if (!get_isReading())
    {
        // Serial.print("    - ");
        // Serial.print(debugName);
        // Serial.println(": start read");
        startReadObject(NODE_ID, readIndex, readSubIndex);
        readStartTime = millis();
    }
    else if (pollReadObject(read_word, errorCode)) // si probleme verifier que la fonction a acces a readValue
    {
        if(errorCode == 0x0000)
        {
            readValue = read_word;
            driver_state = nextState;
            readRetriesCount = 0;
            // Serial.print("    - ");
            // Serial.print(debugName);
            // Serial.println(": successfull");
        }
        else
        {
            readRetriesCount ++;

            Serial.print("    - ");
            Serial.print(debugName);
            Serial.print(" unsuccessfull -> errorCode: ");
            Serial.print(errorCode, HEX);
            Serial.print("  ( nb of retries: ");
            Serial.print(readRetriesCount);
            Serial.println(" )");

            if ( readRetriesCount >= maxRetries )
            {
                driver_state = working_state;
                readRetriesCount = 0;
                Serial.print("    - ");
                Serial.print(debugName);
                Serial.println(": max retries reached, aborting read status...");
            }
        }
    }
    else if ( millis() - readStartTime > maxReadTime )
    {
        isReading = false; // reset reading state to allow retry
        ++readRetriesCount;

        Serial.print("    - ");
        Serial.print(debugName);
        Serial.print(": timeout, retrying...");
        Serial.print(" ( nb of retries: ");
        Serial.print(readRetriesCount);
        Serial.println(" )");
        if ( readRetriesCount >= maxRetries )
        {
            driver_state = nextState;
            readRetriesCount = 0;
            Serial.print("    - ");
            Serial.print(debugName);
            Serial.println(": max retries reached, aborting homing...");
        }
    }
}

void EPOS4::tick()
{
    DWORD errorCode = 0x0000;
    DWORD read_word = 0x0000;
    DWORD readStatusValue = epos_status.value();

    if (millis() - last_tick_time < min_tick_time){ return; }
    if (working_state != DriverState::PPM){PPMsetupDone = false;}

    switch(driver_state) 
    {   
    case DriverState::IDLE:
        //Serial.println("> IDLE");
        if (!get_isWriting() and !get_isReading())
        {
            driver_state = DriverState::READ_STATUS;
        }
        else // should not happen
        {
            pollReadObject(read_word, errorCode);
            pollWriteObject(errorCode);
        }
        break;

    case DriverState::READ_STATUS:
        //Serial.print(">RS_");

        readRegisterStep(working_state, STATUS_WORD_INDEX, STATUS_WORD_SUBINDEX, readStatusValue, "Read Status");
        epos_status = STATUS(readStatusValue);
   
        Serial.print("Status Word: 0x");Serial.println(epos_status.value(), HEX);

        break;

    case DriverState::PPM:
        Serial.print("PPM_");
        runPPM();
        if (!get_isWriting() && !get_isReading())
        {
            driver_state = determineRead();
        }
        break;

    case DriverState::HOMING:
        Serial.println("> HOMING");
        runHoming();
        if (!get_isWriting() && !get_isReading())
        {
            Serial.println("going to read status");
            driver_state = DriverState::READ_STATUS;
        }
        if(homing_done and !homing_error)
        {
            working_state = DriverState::IDLE;
        }
        break;

    case DriverState::FAULT:
        Serial.println("> FAULT");
        //fault();
        if (!get_isWriting() && !get_isReading())
        {
            driver_state = DriverState::READ_STATUS;
        }
        else
        {
            pollReadObject(read_word, errorCode);
            pollWriteObject(errorCode);
        }
        break;

    case DriverState::READ_POSITION_ACTUAL_VALUE:
        // Serial.println("> READ_POSITION_ACTUAL_VALUE");
        readRegisterStep(determineRead(), POSITION_ACTUAL_VALUE_WORD_INDEX, POSITION_ACTUAL_VALUE_WORD_SUBINDEX, epos_position_actual_value, "Read Position Actual Value");
        // if (!get_isReading())
        // {
        //     startReadObject(NODE_ID, POSITION_ACTUAL_VALUE_WORD_INDEX, POSITION_ACTUAL_VALUE_WORD_SUBINDEX);
        // }
        // else if (pollReadObject(read_word, errorCode))
        // {
        //     epos_position_actual_value = read_word;
        //     read_position_actual_value_queued = false;
        //     driver_state = determineRead();
        // }
        break;

    case DriverState::READ_CURRENT_ACTUAL_VALUE:
        // Serial.println("> READ_CURRENT_ACTUAL_VALUE");
        readRegisterStep(determineRead(), CURRENT_ACTUAL_VALUE_WORD_INDEX, CURRENT_ACTUAL_VALUE_WORD_SUBINDEX, epos_current_actual_value, "Read Current Actual Value");
        // if (!get_isReading())
        // {
        //     startReadObject(NODE_ID, CURRENT_ACTUAL_VALUE_WORD_INDEX, CURRENT_ACTUAL_VALUE_WORD_SUBINDEX);
        // }
        // else if (pollReadObject(read_word, errorCode))
        // {
        //     epos_current_actual_value = read_word;
        //     read_current_actual_value_queued = false;
        //     driver_state = determineRead();
        // }
        break;
    }
    
    if (timeout)
    {
        Serial.println("[Tick] Timeout");
        reset();
    }

    if (errorCode)
    {
        Serial.print("[Tick] Error: 0x");
        Serial.println(errorCode, HEX);
        reset();
    }
    
    if (epos_status.fault())
    {
        Serial.print("Status Word Fault: 0x");
        Serial.println(epos_status.value(), HEX); // read status word to clear fault condition
        reset();
        driver_state = DriverState::FAULT;
    }

    last_tick_time = millis();
}

void EPOS4::executePPMStep(PPMState nextState, WORD writeIndex, BYTE writeSubIndex, DWORD writeValue, const char* debugName)
{
    DWORD errorCode = 0xFFFF;
    const uint8_t maxRetries = 5;
    long maxWriteTime = 2000; // 2 seconds

    if (!get_isWriting())
    {
        startWriteObject(NODE_ID, writeIndex, writeSubIndex, writeValue);
        ppmWriteRetriesCount = 0;
        ppmWriteStartTime = millis();

        Serial.print("    - ");
        Serial.print(debugName);
        Serial.println(": start write");
    }
    else if (pollWriteObject(errorCode))
    {
        if(errorCode == 0x0000)
        {
            ppm_state = nextState;
            ppmWriteRetriesCount = 0;
            // Serial.print("    - ");
            // Serial.print(debugName);
            //Serial.println(" sent successfully");
        }
        else
        {
            Serial.print("    - ");
            Serial.print(debugName);
            Serial.print(" unsuccessfull -> errorCode: ");
            Serial.print(errorCode, HEX);
            ppmWriteRetriesCount ++;
            Serial.print("  ( nb of retries: ");
            Serial.print(ppmWriteRetriesCount);
            Serial.println(" )");

            if ( ppmWriteRetriesCount >= maxRetries )
            {
                Serial.print("    - ");
                Serial.print(debugName);
                Serial.println(": max retries reached, aborting homing...");
                ppm_error = true;
                working_state = DriverState::IDLE;
                homing_state = HomingState::SET_OPERATION_MODE;
                ppm_state = PPMState::SET_OPERATION_MODE;
            }
        }
    }
    else if ( millis() - ppmWriteStartTime > maxWriteTime )
    {
        Serial.print("    - ");
        Serial.print(debugName);
        Serial.print(": write timeout, retrying...");
        isWriting = false; // reset writing state to allow retry
        ppmWriteRetriesCount ++;
        Serial.print(" ( nb of retries: ");
        Serial.print(ppmWriteRetriesCount);
        Serial.println(" )");
        if ( ppmWriteRetriesCount >= maxRetries )
        {
            Serial.print("    - ");
            Serial.print(debugName);
            Serial.println(": max retries reached, aborting homing...");
            ppm_error = true;
            working_state = DriverState::IDLE;
            homing_state = HomingState::SET_OPERATION_MODE;
            ppm_state = PPMState::SET_OPERATION_MODE;
        }
    }
    ppmWaitStartTime = millis();
    ppmWaitRetriesCount = 0;
}

void EPOS4::runPPM()
{
    const unsigned long waitTimeForStatus = 400; // [ms]

    switch (ppm_state)
    {
    // [1] Set variables
    case PPMState::SET_OPERATION_MODE:
        Serial.println("> PPM_SET_OPERATION_MODE");
        executePPMStep(PPMState::SET_PROFILE_VELOCITY, OPERATION_MODE_INDEX, OPERATION_MODE_SUBINDEX, OPERATION_MODE_PROFILE_POSITION, "Set operation mode");
        break;
    case PPMState::SET_PROFILE_VELOCITY:
        Serial.println("> PPM_SET_PROFILE_VELOCITY");
        executePPMStep(PPMState::SET_PROFILE_ACCELERATION,PROFILE_VELOCITY_SUBINDEX,TARGET_POSITION_SUBINDEX,ppm_cfg.profile_velocity, "Set profile velocity");
        break;
    case PPMState::SET_PROFILE_ACCELERATION:
        Serial.println("> PPM_SET_PROFILE_ACCELERATION");
        executePPMStep(PPMState::SET_PROFILE_DECELERATION, PROFILE_ACCELERATION_INDEX, PROFILE_ACCELERATION_SUBINDEX, ppm_cfg.profile_acceleration, "Set profile acceleration");
        break;
    case PPMState::SET_PROFILE_DECELERATION:
      Serial.println("> PPM_SET_PROFILE_DECELERATION");
        executePPMStep(PPMState::SET_NOMINAL_CURRENT, PROFILE_DECELERATION_INDEX, PROFILE_DECELERATION_SUBINDEX, ppm_cfg.profile_deceleration, "Set profile deceleration");
        break;
    //case PPMState::SET_QUICK_STOP_DECELERATION -> not used
    case PPMState::SET_NOMINAL_CURRENT:
        Serial.println("> PPM_SET_NOMINAL_CURRENT");
        executePPMStep(PPMState::SET_OUTPUT_CURRENT_LIMIT, MOTOR_DATA_INDEX, NOMINAL_CURRENT_SUBINDEX, ppm_cfg.nominal_current, "Set nominal current");
        break;
    case PPMState::SET_OUTPUT_CURRENT_LIMIT:
        Serial.println("> PPM_SET_OUTPUT_CURRENT_LIMIT");
        executePPMStep(PPMState::SHUTDOWN, MOTOR_DATA_INDEX, OUTPUT_CURRENT_LIMIT_SUBINDEX, ppm_cfg.output_current_limit, "Set output current limit");
        break;
        
    // [2] Set operation mode to PPM (command: SHUTDOWN -> SWITCH_ON -> ENABLE_OPERATION )
    case PPMState::SHUTDOWN:
        Serial.println("> SHUTDOWN");
        executePPMStep(PPMState::WAIT_READY_TO_SWITCH_ON, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_SHUTDOWN, "Shutdown");
        break;
    case PPMState::WAIT_READY_TO_SWITCH_ON:
        Serial.println("> WAIT_READY_TO_SWITCH_ON");
        if (epos_status.readyToSwitchOn())
        {
            ppm_state = PPMState::ENABLE;
        }

        else if (millis() - ppmWaitStartTime > waitTimeForStatus)
        {
            Serial.println("    - Wait for readyToSwitchOn timeout, retrying shutdown...");
            ppmWaitRetriesCount++;
            ppm_state = PPMState::SHUTDOWN; // retry shutdown
        }
        else if ( ppmWaitRetriesCount >= 5 )
        {
            Serial.println("    - Ready to switch on: max retries reached, aborting ppm...");
            working_state = DriverState::IDLE;
            ppm_state = PPMState::SET_OPERATION_MODE;
        }
        break;
    case PPMState::ENABLE:
        Serial.println("> PPM_ENABLE");
        executePPMStep(PPMState::WAIT_ENABLE, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_SWITCH_ON_AND_ENABLE_OPERATION, "Enable operation");
        break;
    case PPMState::WAIT_ENABLE:
        Serial.println("> WAIT_ENABLE");
        if (epos_status.operationEnabled())
        {
            ppm_state = PPMState::SET_TARGET_POSITION_IF_UPDATED;
            PPMsetupDone = true;
            ppm_error = false;
        }
        else if (millis() - ppmWaitStartTime > waitTimeForStatus)
        {
            Serial.println("    - Wait for operationEnabled timeout, retrying enable...");
            ppmWaitRetriesCount++;
            ppm_state = PPMState::ENABLE; // retry enable
        }
        else if ( ppmWaitRetriesCount >= 5 )
        {
            Serial.println("    - Ready to switch on: max retries reached, aborting ppm...");
            working_state = DriverState::IDLE;
            ppm_state = PPMState::SET_OPERATION_MODE;
        }
        break;
    
    // [3] Set target and trig
    case PPMState::SET_TARGET_POSITION_IF_UPDATED: // to increase response time for a position change
        Serial.println("> SET_TARGET_POSITION_IF_UPDATED");
        if(ppmLastSentPosition != ppm_cfg.target_position or millis() - lastSentPositionTime > 100)
        {
            ppmLastSentPosition = ppm_cfg.target_position;
            ppm_state = PPMState::SET_TARGET_POSITION;
            lastSentPositionTime = millis();
        }
        break;
    case PPMState::SET_TARGET_POSITION:
        Serial.println("> SET_TARGET_POSITION");
        executePPMStep(PPMState::TOGGLE_LOW,TARGET_POSITION_INDEX,TARGET_POSITION_SUBINDEX, ppm_cfg.target_position, "Set target position");
        break;
    case PPMState::TOGGLE_LOW:
        Serial.println("> PPM_TOGGLE_LOW");
        executePPMStep(PPMState::SET_OPERATION_MODE, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_TOGGLE_LOW, "toggle operation");
        break;
    case PPMState::TOGGLE_HIGH:
        Serial.println("> PPM_TOGGLE_HIGH");
        executePPMStep(PPMState::SET_TARGET_POSITION_IF_UPDATED, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_TOGGLE_HIGH, "toggle operation");
        break;
    }
}

void EPOS4::executeHomingStep(HomingState nextState, WORD writeIndex, BYTE writeSubIndex, DWORD writeValue, const char* debugName)
{
    DWORD errorCode = 0xFFFF;
    const uint8_t maxRetries = 5;
    long maxWriteTime = 10000; // 2 seconds

    if (!get_isWriting())
    {
        Serial.print("    - ");
        Serial.print(debugName);
        Serial.println(": start write");
        startWriteObject(NODE_ID, writeIndex, writeSubIndex, writeValue);
        homingWriteStartTime = millis();
        homingWriteRetriesCount = 0;
    }
    else if (pollWriteObject(errorCode))
    {
        if(errorCode == 0x0000)
        {
            homing_state = nextState;
            homingWriteRetriesCount = 0;
            Serial.print("    - ");
            Serial.print(debugName);
            Serial.println(" sent successfully");
        }
        else
        {
            Serial.print("    - ");
            Serial.print(debugName);
            Serial.print(" unsuccessfull -> errorCode: ");
            Serial.print(errorCode, HEX);
            homingWriteRetriesCount ++;
            Serial.print("  ( nb of retries: ");
            Serial.print(homingWriteRetriesCount);
            Serial.println(" )");

            if ( homingWriteRetriesCount >= maxRetries )
            {
                Serial.print("    - ");
                Serial.print(debugName);
                Serial.println(": max retries reached, aborting homing...");
                homing_done = true;
                homing_error = true;
                working_state = DriverState::IDLE;
                homing_state = HomingState::SET_OPERATION_MODE;
            }
        }
    }
    else if ( millis() - homingWriteStartTime > maxWriteTime )
    {
        Serial.print("    - ");
        Serial.print(debugName);
        Serial.print(": write timeout, retrying...");
        isWriting = false; // reset writing state to allow retry
        homingWriteRetriesCount ++;
        Serial.print(" ( nb of retries: ");
        Serial.print(homingWriteRetriesCount);
        Serial.println(" )");
        if ( homingWriteRetriesCount >= maxRetries )
        {
            Serial.print("    - ");
            Serial.print(debugName);
            Serial.println(": max retries reached, aborting homing...");
            homing_done = true;
            homing_error = true;
            working_state = DriverState::IDLE;
            homing_state = HomingState::SET_OPERATION_MODE;
        }
    }
    homingWaitStartTime = millis();
    homingWaitRetriesCount = 0;
}

void EPOS4::runHoming(bool direction)
{
    const unsigned long waitTimeForStatus = 400; // [ms]
    const unsigned long waitTimeWhileMoving = 10000; // [ms]

    switch (homing_state)
    {
    case HomingState::SET_OPERATION_MODE:
        Serial.println("> SET_OPERATION_MODE");
        executeHomingStep(HomingState::SET_SPEED_FOR_SWITCH_SEARCH, OPERATION_MODE_INDEX, OPERATION_MODE_SUBINDEX, OPERATION_MODE_HOMING, "Set operation mode");
        break;

    case HomingState::SET_SPEED_FOR_SWITCH_SEARCH:
        Serial.println("> SET_SPEED_FOR_SWITCH_SEARCH");
        executeHomingStep(HomingState::SET_SPEED_FOR_ZERO_SEARCH, SPEED_FOR_SWITCH_SEARCH_INDEX, SPEED_FOR_SWITCH_SEARCH_SUBINDEX, homing_cfg.speed_for_switch_search, "Set speed for switch search");
        break;
    
    case HomingState::SET_SPEED_FOR_ZERO_SEARCH:
        Serial.println("> SET_SPEED_FOR_ZERO_SEARCH");
        executeHomingStep(HomingState::SET_HOMING_ACCELERATION, SPEED_FOR_ZERO_SEARCH_INDEX, SPEED_FOR_ZERO_SEARCH_SUBINDEX, homing_cfg.speed_for_zero_search, "Set speed for zero search");
        break;

    case HomingState::SET_HOMING_ACCELERATION:
        Serial.println("> SET_HOMING_ACCELERATION");
        executeHomingStep(HomingState::SET_HOMING_CURRENT, HOMING_ACCELERATION_INDEX, HOMING_ACCELERATION_SUBINDEX, homing_cfg.homing_acceleration, "Set homing acceleration");
        break;

    case HomingState::SET_HOMING_CURRENT:
        Serial.println("> SET_HOMING_CURRENT");
        executeHomingStep(HomingState::SET_OFFSET_MOVE_DISTANCE, HOMING_CURRENT_INDEX, HOMING_CURRENT_SUBINDEX, homing_cfg.homing_current, "Set homing current");
        break;

    case HomingState::SET_OFFSET_MOVE_DISTANCE:
        Serial.println("> SET_OFFSET_MOVE_DISTANCE");
        executeHomingStep(HomingState::SET_HOME_POSITION, HOME_OFFSET_MOVE_DISTANCE_INDEX, HOME_OFFSET_MOVE_DISTANCE_SUBINDEX, homing_cfg.homing_offset_distance, "Set offset move distance");
        break;

    case HomingState::SET_HOME_POSITION:
        Serial.println("> SET_HOME_POSITION");
        executeHomingStep(HomingState::SET_HOMING_METHOD, HOME_POSITION_INDEX, HOME_POSITION_SUBINDEX, homing_cfg.home_position, "Set home position");
        break;

    case HomingState::SET_HOMING_METHOD:
        Serial.println("> SET_HOMING_METHOD");
        executeHomingStep(HomingState::SHUTDOWN, HOMING_METHOD_INDEX, HOMING_METHOD_SUBINDEX, direction ? HOMING_METHOD_CURRENT_THRESHOLD : HOMING_METHOD_CURRENT_THRESHOLD_NEGATIVE, "Set homing method");
        break;

    case HomingState::SHUTDOWN:
        Serial.println("> SHUTDOWN");
        executeHomingStep(HomingState::WAIT_READY_TO_SWITCH_ON, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_SHUTDOWN, "Shutdown");
        break;

    case HomingState::WAIT_READY_TO_SWITCH_ON:
        Serial.println("> WAIT_READY_TO_SWITCH_ON");
        if (epos_status.readyToSwitchOn())
        {
            Serial.print("   - Ready to switch on detected");
            homing_state = HomingState::ENABLE;
        }
        else if (millis() - homingWaitStartTime > waitTimeForStatus)
        {
            Serial.println("    - Wait for readyToSwitchOn timeout, retrying shutdown...");
            homingWaitRetriesCount++;
            homing_state = HomingState::SHUTDOWN; // retry shutdown
        }
        else if ( homingWaitRetriesCount >= 5 )
        {
            Serial.println("    - Ready to switch on: max retries reached, aborting homing...");
            homing_done = true;
            homing_error = true;
            working_state = DriverState::IDLE;
            homing_state = HomingState::SET_OPERATION_MODE;
        }
        break;

    case HomingState::ENABLE:
        Serial.println("> ENABLE");
        executeHomingStep(HomingState::OPERATION_ENABLED, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_SWITCH_ON_AND_ENABLE_OPERATION, "Enable operation");
        break;


    case HomingState::OPERATION_ENABLED:
        Serial.println("> OPERATION_ENABLED");
        if (epos_status.operationEnabled())
        {
            homing_state = HomingState::START_HOMING;
        }
        else if (millis() - homingWaitStartTime > waitTimeForStatus)
        {
            Serial.println("    - Wait for operationEnabled timeout, retrying enable...");
            homingWaitRetriesCount++;
            homing_state = HomingState::ENABLE; // retry enable
        }
        else if ( homingWaitRetriesCount >= 5 )
        {
            Serial.println("    - Operation enabled: max retries reached, aborting homing...");
            homing_done = true;
            homing_error = true;
            working_state = DriverState::IDLE;
            homing_state = HomingState::SET_OPERATION_MODE;
        }
        break;
        
    
    case HomingState::START_HOMING:
        Serial.println("> START_HOMING");
        ppm_state = PPMState::SET_OPERATION_MODE;
        executeHomingStep(HomingState::IN_PROGRESS, CONTROL_WORD_INDEX, CONTROL_WORD_SUBINDEX, CONTROL_WORD_START_HOMING, "Start homing");
        break;

    case HomingState::IN_PROGRESS:
        Serial.println("> HOMING IN PROGRESS");
        Serial.print("    - Waiting for homing to complete... ( time: ");
        Serial.print(millis() - homingWaitStartTime);
        Serial.println(" ms )");

        if (epos_status.homingAttained())
        {
            Serial.println("    - homing attained successfully !");
            homing_done = true;
            homing_error = false;
            working_state = DriverState::IDLE;
            homing_state = HomingState::SET_OPERATION_MODE;
        }
        else if (millis() - homingWaitStartTime > waitTimeWhileMoving)
        {
            Serial.println("    - Wait for homingAttained, retrying enable...");
            homingWaitRetriesCount++;
            homing_state = HomingState::SHUTDOWN; // retry from shutdown
        }
        else if ( homingWaitRetriesCount >= 5 )
        {
            Serial.println("    - In progress: max retries reached, aborting homing...");
            homing_done = true;
            homing_error = true;
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
        Serial.println("Fault reset ended (write error code: 0x" + String(errorCode, HEX) + ")");
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

DWORD EPOS4::read_position()
{
    if (driver_state != DriverState::FAULT)
    {
        read_position_actual_value_queued = true; // will trigger a read position in the main tick loop once
    }
    return epos_position_actual_value;
}

DWORD EPOS4::read_current()
{
    if (driver_state != DriverState::FAULT)
    {
        read_current_actual_value_queued = true; // will trigger a read current in the main tick loop once
    }
    return epos_current_actual_value;
}

void EPOS4::current_threshold_homing()
{
    if (driver_state == DriverState::FAULT)
    {
        return;
    }
    else
    {
        homing_done = false;
        homing_error = false;
        ppm_error  = false;
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
            Serial.println("[writeObject] Timeout waiting for response");
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
    if (isReading or isWriting)
    {
        return;
    }

    Serial.println("[writeObject] Start writing");
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
    // return true when it's done even it's with error (errorCode != 0), return false if still waiting for response
    
    if (!isWriting)
        return false;

    if (millis() - startTime > read_timeout) 
    {
        isWriting = false;
        timeout = true;
        errorCode = 0x0001; // homemade timeout error code
        return true;
    }

    do  
    {
        // shift buffer (to the left)
        if (eposSerial.available())
        {
            for(size_t i = 0; i < sizeof(rx_buffer) - 1; i++)
            {
                rx_buffer[i] = rx_buffer[i+1];
            }
            rx_buffer[sizeof(rx_buffer)-1] = eposSerial.read();
        }

        // find DLE-STX
        uint8_t header_index = 0;
        for(size_t i = 0; i < sizeof(rx_buffer) - 1; i++)
        {
            header_index = i;
            if(rx_buffer[i] == 0x90 && rx_buffer[i+1] == 0x02)
            {
                break;
            }
            else if(i==sizeof(rx_buffer))
            {
                // DLE-STX not found, wait for more bytes
                continue;
            }
        }

        // try to get OP CODE and LEN
        uint8_t op_code;
        uint8_t len;
        uint8_t header_len = 4; // DLE + STX + op code + len
        uint8_t data_len;
        uint8_t error_code_len = 4; 
        uint8_t crc_len = 2;
        if ( header_index + header_len < sizeof(rx_buffer) )
        {
            op_code = rx_buffer[header_index+2];
            len = rx_buffer[header_index+3];
            data_len = len*2-error_code_len;
        }
        else{ continue; }

        // try to unstuff until end of message (len + error code + crc)
        uint8_t error_code[error_code_len];
        uint8_t data[data_len];
        uint8_t crc[crc_len];
        uint8_t index = 0;
        bool not_enough_bytes = false;
        for (size_t i = 0; i < error_code_len + data_len + crc_len; i++)
        {
            if(rx_buffer[header_index + header_len + index] == 0x90)
            {
                if( header_index + header_len + index + 1 >= sizeof(rx_buffer) ){ not_enough_bytes = true; break; }
                else if(rx_buffer[header_index + header_len + index + 1] == 0x90) // stuffed byte (double 0x90)
                {
                    index++;
                }
                else if(rx_buffer[header_index + header_len + index + 1] == 0x02) // new start comm => erase previous
                {
                    for(size_t i = 0; i < header_index + header_len + index - 1; i++)
                    {
                        rx_buffer[i] = 0x00;
                    }
                    not_enough_bytes = true;
                    break;
                }
                else // corrupted message => erase until corruption
                {
                    for(size_t i = 0; i < header_index + header_len + index + 1; i++)
                    {
                        rx_buffer[i] = 0x00;
                    }
                    not_enough_bytes = true;
                    break;
                }
            } 
            if( header_index + header_len + index >= sizeof(rx_buffer) ){ not_enough_bytes = true; break; }
            if( i < error_code_len )
            {
                error_code[i] = rx_buffer[header_index + header_len + index];
            }
            else if ( i < error_code_len + data_len )
            {
                data[i - error_code_len] = rx_buffer[header_index + header_len + index];
            }
            else
            {
                crc[i - error_code_len - data_len] = rx_buffer[header_index + header_len + index];
            }
            index++;
        }
        if(not_enough_bytes){ continue; }

        // FROM THIS POINT THE FUNCTION WILL RETURN TRUE (MESSAGE IS COMPLETE)

        // Erase frame <= message is complete after "try to unstuff"
        uint8_t total_len = 4 + data_len + error_code_len + crc_len;
        for(size_t i = 0; i < total_len; i++)
        {
            rx_buffer[i] = 0x00;
        }

        // Compute crc
        size_t messageWordLen = 1 + (data_len + error_code_len) / 2 + 1;
        uint16_t messageWord[messageWordLen];
        messageWord[0] = static_cast<uint16_t>(len << 8) | static_cast<uint16_t>(op_code);
        messageWord[1] = static_cast<uint16_t>(error_code[1]<< 8) | (static_cast<uint16_t>(error_code[0]));
        messageWord[2] = static_cast<uint16_t>(error_code[3]<< 8) | (static_cast<uint16_t>(error_code[2]));
        for(size_t i = 0; i < data_len; i+=2)
        {
            messageWord[3 + i/2] = static_cast<uint16_t>(data[i+1]<< 8) | static_cast<uint16_t>(data[i]);
        }
        messageWord[3 + data_len/2] = 0x0000; // crc = 0 for crc calculation
        uint16_t expected_crc = calcCRC(messageWord, messageWordLen);
        uint16_t crc_word = crc[0] | (crc[1] << 8);

        // OLD Compute crc
        // size_t messageWordLen = 1 + (data_len + error_code_len) / 2 + 1;
        // uint16_t messageWord[messageWordLen];
        // messageWord[0] = static_cast<uint16_t>(len << 8) | static_cast<uint16_t>(op_code);
        // for(size_t i = 0; i < data_len; i+=2)
        // {
        //     messageWord[1+i/2] = static_cast<uint16_t>(data[i+1] << 8) | static_cast<uint16_t>(data[i]);
        // }
        // messageWord[1 + data_len/2] = static_cast<uint16_t>(error_code[0]) | (static_cast<uint16_t>(error_code[1]) << 8);
        // messageWord[1 + data_len/2 + 1] = static_cast<uint16_t>(error_code[2]) | (static_cast<uint16_t>(error_code[3]) << 8);
        // messageWord[1 + data_len/2 + 2] = 0x0000; // crc = 0 for crc calculation
        // uint16_t expected_crc = calcCRC(messageWord, messageWordLen);
        // uint16_t crc_word = crc[0] | (crc[1] << 8);

        // check message integrity with crc
        if( crc_word == expected_crc )
        {
            // Extract error code from response
            errorCode = (static_cast<uint32_t>(error_code[0]) << 0 ) |
                        (static_cast<uint32_t>(error_code[1]) << 8 ) |
                        (static_cast<uint32_t>(error_code[2]) << 16) |
                        (static_cast<uint32_t>(error_code[3]) << 24);
        }
        else
        {
            errorCode = 0x0504; // reuse errorCode for CRC error

            //Serial.println("[pollWriteObject] CRC error");
            //Serial.println(" - Received CRC: 0x" + String(crc[1], HEX) + String(crc[0], HEX));
            //Serial.println(" - Expected CRC: 0x" + String((expected_crc >> 8) & 0xFF, HEX) + String(expected_crc & 0xFF, HEX));

        }

        // print error code if not zero
        if (errorCode != 0x0000)
        {
            Serial.println("[pollWriteObject] Error code: 0x" + String(errorCode, HEX));
        }

        isWriting = false;
        return true;

    }while(eposSerial.available());

    return false; // still waiting for response
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
    if (isReading or isWriting)
    {
        return;
    }

    //Serial.println("   - [readObject] Start reading");
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
    // return true when it's done even it's with error (errorCode != 0)
    // return false if still waiting for response


    if (!isReading)
        return false;

    if (millis() - startTime > read_timeout) 
    {
        isReading = false;
        timeout = true;
        errorCode = 0x0001; // homemade timeout error code
        return true;
    }

    do  
    {
        // shift buffer (to the left)
        if (eposSerial.available())
        {
            for(size_t i = 0; i < sizeof(rx_buffer) - 1; i++)
            {
                rx_buffer[i] = rx_buffer[i+1];
            }
            rx_buffer[sizeof(rx_buffer)-1] = eposSerial.read();
        }

        // find DLE-STX
        uint8_t header_index = 0;
        for(size_t i = 0; i < sizeof(rx_buffer) - 1; i++)
        {
            header_index = i;
            if(rx_buffer[i] == 0x90 && rx_buffer[i+1] == 0x02)
            {
                break;
            }
            else if(i==sizeof(rx_buffer))
            {
                // DLE-STX not found, wait for more bytes
                continue;
            }
        }

        // try to get OP CODE and LEN
        uint8_t op_code;
        uint8_t len;
        uint8_t data_len;
        uint8_t header_len = 4; // DLE + STX + op code + len
        uint8_t error_code_len = 4;
        uint8_t crc_len = 2;
        if ( header_index + header_len < sizeof(rx_buffer) )
        {
            op_code = rx_buffer[header_index+2];
            len = rx_buffer[header_index+3];
            data_len = len*2-error_code_len;
        }
        else{ continue; }

        // try to unstuff until end of message (len + error code + crc)
        uint8_t error_code[error_code_len];
        uint8_t data[data_len];
        uint8_t crc[crc_len];
        uint8_t index = 0;
        bool not_enough_bytes = false;
        for (size_t i = 0; i < data_len + error_code_len + crc_len; i++)
        {
            if(rx_buffer[header_index + header_len + index] == 0x90)
            {
                if( header_index + header_len + index + 1 >= sizeof(rx_buffer) ){ not_enough_bytes = true; break; }
                else if(rx_buffer[header_index + header_len + index + 1] == 0x90) // stuffed byte (double 0x90)
                {
                    index++;
                }
                else if(rx_buffer[header_index + header_len + index + 1] == 0x02) // new start comm => erase previous
                {
                    for(size_t i = 0; i < header_index + header_len + index - 1; i++)
                    {
                        rx_buffer[i] = 0x00;
                    }
                    not_enough_bytes = true;
                    break;
                }
                else // corrupted message => erase until corruption
                {
                    for(size_t i = 0; i < header_index + header_len + index + 1; i++)
                    {
                        rx_buffer[i] = 0x00;
                    }
                    not_enough_bytes = true;
                    break;
                }
            }
            if( header_index + header_len + index >= sizeof(rx_buffer) ){ not_enough_bytes = true; break; }
            if( i < error_code_len )
            {
                error_code[i] = rx_buffer[header_index + header_len + index];
            }
            else if ( i <  error_code_len + data_len)
            {
                data[i - error_code_len] = rx_buffer[header_index + header_len + index];
            }
            else
            {
                crc[i - error_code_len - data_len] = rx_buffer[header_index + header_len + index];
            }
            index++;
        }
        if(not_enough_bytes){ continue; }

        // FROM THIS POINT THE FUNCTION WILL RETURN TRUE (MESSAGE IS COMPLETE)

        // Erase frame <= message is complete after "try to unstuff"
        uint8_t total_len = 4 + data_len + error_code_len + crc_len;
        for(size_t i = 0; i < total_len; i++)
        {
            rx_buffer[i] = 0x00;
        }

        // Compute crc
        size_t messageWordLen = 1 + (data_len + error_code_len) / 2 + 1;
        uint16_t messageWord[messageWordLen];
        messageWord[0] = static_cast<uint16_t>(len << 8) | static_cast<uint16_t>(op_code);
        messageWord[1] = static_cast<uint16_t>(error_code[1]<< 8) | (static_cast<uint16_t>(error_code[0]));
        messageWord[2] = static_cast<uint16_t>(error_code[3]<< 8) | (static_cast<uint16_t>(error_code[2]));
        for(size_t i = 0; i < data_len; i+=2)
        {
            messageWord[3 + i/2] = static_cast<uint16_t>(data[i+1]<< 8) | static_cast<uint16_t>(data[i]);
        }
        messageWord[3 + data_len/2] = 0x0000; // crc = 0 for crc calculation
        uint16_t expected_crc = calcCRC(messageWord, messageWordLen);
        uint16_t crc_word = crc[0] | (crc[1] << 8);

        // Check message integrity with crc
        if( crc_word == expected_crc )
        {
            // Extract error code from response
            errorCode = (static_cast<uint32_t>(error_code[0]) << 0 ) |
                        (static_cast<uint32_t>(error_code[1]) << 8 ) |
                        (static_cast<uint32_t>(error_code[2]) << 16) |
                        (static_cast<uint32_t>(error_code[3]) << 24);

            // Extract value from response
            value   =   (static_cast<uint32_t>(data[0]) << 0 ) |
                        (static_cast<uint32_t>(data[1]) << 8 ) |
                        (static_cast<uint32_t>(data[2]) << 16) |
                        (static_cast<uint32_t>(data[3]) << 24);
        }
        else
        {
            errorCode = 0x0504; // reused errorCode (CRC error)
            value = 0;
            isReading = false;

            //prints
                Serial.println("> [pollReadObject]");
                Serial.print("   - CRC error");
                Serial.println("   - Received CRC: 0x" + String(crc[1], HEX) + String(crc[0], HEX));
                Serial.println("   - Expected CRC: 0x" + String((expected_crc >> 8) & 0xFF, HEX) + String(expected_crc & 0xFF, HEX));
                Serial.print("   - Message words for CRC calculation: ");
                for(size_t i = 0; i < messageWordLen; i++)        {
                    Serial.print("0x" + String(messageWord[i], HEX) + " ");
                }
                Serial.println();
                Serial.print("   - Received response: ");
                for(size_t i = 0; i < sizeof(rx_buffer); i++)
                {
                    Serial.print("0x" + String(rx_buffer[i], HEX) + " ");
                }
                Serial.println();
                Serial.println("   - Header index" + String(header_index));
                Serial.println("   - OP Code: 0x" + String(op_code, HEX));
                Serial.println("   - Len: " + String(len));
                Serial.print("   - Value: ");
                for(size_t i = 0; i < data_len; i++)
                {
                    Serial.print("0x" + String(data[i], HEX) + " ");
                }
                Serial.println();
                Serial.print(" -   Error code: ");
                for(size_t i = 0; i < error_code_len; i++)
                {
                    Serial.print("0x" + String(error_code[i], HEX) + " ");
                }
                Serial.println();
        }

        // print error code if not zero
        if (errorCode != 0x0000)
        {
            Serial.println("[pollReadObject] Error code: 0x" + String(errorCode, HEX));
        }

        isReading= false;
        return true;

    }while(eposSerial.available());

    return false; // still waiting for response

    // OLD FUNCTION
    // constexpr unsigned response_length = 14;
    // uint8_t response[response_length];
    // if (!isReading)
    //     return false;
    // if (millis() - startTime > read_timeout) 
    // {
    //     //Serial.println("[readObject] Timeout waiting for response");
    //     isReading = false;
    //     timeout = true;
    //     errorCode = 0x0001; // homemade timeout error code
    //     return false;
    // }
    // if (eposSerial.available() >= response_length) // check for expected response size
    // {
    //     for (size_t i = 0; i < response_length; i++)
    //     {
    //         response[i] = eposSerial.read();
    //     }
    //     while (eposSerial.available())
    //     {
    //         eposSerial.read(); // flush any extra bytes
    //     }
    // }
    // else
    // {
    //     return false;
    // }
    // isReading = false; // end of reading anyway
    // // Check if response is valid (size = 6, op code = 0x00, len = 4)
    // if (response[2] != 0x00 || response[3] != 0x04) 
    // {
    //     errorCode = 0x0002; // homemade error code
    //     return false; // Return 0 on error
    // }
    // // Extract error code from response
    // errorCode = (static_cast<uint32_t>(response[4]) << 0 ) |
    //             (static_cast<uint32_t>(response[5]) << 8 ) |
    //             (static_cast<uint32_t>(response[6]) << 16) |
    //             (static_cast<uint32_t>(response[7]) << 24);
    // // Extract value from response
    // value   =   (static_cast<uint32_t>(response[8]) << 0 ) |
    //             (static_cast<uint32_t>(response[9]) << 8 ) |
    //             (static_cast<uint32_t>(response[10]) << 16) |
    //             (static_cast<uint32_t>(response[11]) << 24);
    // return true;
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
    }

    // Serial.print("Sent frame: ");
    // for (uint8_t b : frame)
    // {
    //     Serial.print(" 0x");
    //     Serial.print(b, HEX);
    // }
    // Serial.println("");
}