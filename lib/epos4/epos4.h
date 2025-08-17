/*
    File: epos4.h
    Author: Axel Juaneda
    Organization: EPFL Rocket Team
    Version : 1.0
*/

#ifndef EPOS4_H
#define EPOS4_H

#include <Arduino.h>
#include <vector>

typedef int8_t BYTE;
typedef int16_t WORD;
typedef int32_t DWORD;


class STATUS
{
public:
    STATUS() {}
    STATUS(DWORD value) : raw(value) {}
    // explicit STATUS(DWORD value) : raw(value) {}

    // Accès direct aux bits
    bool readyToSwitchOn()      const { return raw & (1 << 0); }
    bool switchedOn()           const { return raw & (1 << 1); }
    bool operationEnabled()     const { return raw & (1 << 2); }
    bool fault()                const { return raw & (1 << 3); }
    bool voltageEnabled()       const { return raw & (1 << 4); }
    bool quickStop()            const { return raw & (1 << 5); }
    bool switchOnDisabled()     const { return raw & (1 << 6); }
    bool warning()              const { return raw & (1 << 7); }
    bool remote()               const { return raw & (1 << 9); }
    bool targetReached()        const { return raw & (1 << 10); }
    bool internalLimitActive()  const { return raw & (1 << 11); }
    bool setpointAcknowledge()  const { return raw & (1 << 12); }
    bool homingAttained()       const { return raw & (1 << 12); }
    bool followingError()       const { return raw & (1 << 13); }
    bool HomingError()          const { return raw & (1 << 13); }
    bool positionReferenced()   const { return raw & (1 << 15); }

    // Accès brut au mot de statut
    DWORD value() const { return raw; }

private:
    DWORD raw;
};

enum class DriverState {
    IDLE,
    PPM,
    HOMING,
    FAULT
};

enum class PPMState {
    SET_OPERATION_MODE,
    SET_PARAMETER,
    SHUTDOWN,
    ENABLE,
    SET_TARGET_POSITION,
    TOGGLE
};

enum class HomingState {
    SET_OPERATION_MODE,
    SET_SPEED_FOR_SWITCH_SEARCH,
    SET_SPEED_FOR_ZERO_SEARCH,
    SET_HOMING_ACCELERATION,
    SET_HOMING_CURRENT,
    SET_OFFSET_MOVE_DISTANCE,
    SET_HOME_POSITION,
    SET_HOMING_METHOD,
    SHUTDOWN,
    ENABLE,
    START_HOMING,
    DONE
};

struct HomingConfig
{
    DWORD homing_offset_distance = 0;
    DWORD home_position = 0;
    DWORD speed_for_switch_search = 500;
    DWORD speed_for_zero_search = 500;
    DWORD homing_acceleration = 2000;
    DWORD homing_current = 500; // in mA
};

class EPOS4 
{
public:
    EPOS4(HardwareSerial &eposSerial, unsigned long baudrate = 115200);

    void tick();

    void writeObject(BYTE nodeID, WORD index, BYTE sub_index, const DWORD& value, DWORD& errorCode);
    DWORD readObject(BYTE nodeID, WORD index, BYTE sub_index, DWORD& errorCode);

    void startWriteObject(BYTE nodeID, WORD index, BYTE sub_index, const DWORD& value);
    bool pollWriteObject(DWORD& errorCode);

    void startReadObject(BYTE nodeID, WORD index, BYTE sub_index);
    bool pollReadObject(DWORD& value, DWORD& errorCode);

    void go_to_position(const DWORD position);
    void current_threshold_homing();

    bool get_isReading() { return isReading; }
    bool get_isWriting() { return isWriting; }

    void set_homing_offset_distance(const DWORD value) { homing_cfg.homing_offset_distance = value; }
    void set_home_position(const DWORD value) { homing_cfg.home_position = value; }
    void set_homing_speed_for_switch_search(const DWORD value) { homing_cfg.speed_for_switch_search = value; }
    void set_homing_speed_for_zero_search(const DWORD value) { homing_cfg.speed_for_zero_search = value; }
    void set_homing_acceleration(const DWORD value) { homing_cfg.homing_acceleration = value; }
    void set_homing_current(const DWORD value) { homing_cfg.homing_current = value; }

private:
    HardwareSerial &eposSerial;
    unsigned long baudrate;
    unsigned long read_timeout;
    unsigned long homing_timeout;
    bool isReading, isWriting;

    DriverState driver_state;
    PPMState ppm_state;
    HomingState homing_state;
    STATUS epos_status;

    HomingConfig homing_cfg;
    DWORD target_position;

    void runPPM();
    void runHoming();

    uint16_t calcCRC(uint16_t* dataArray, uint8_t numWords);
    void addStuffedByte(std::vector<uint8_t> &frame, uint8_t byte);
    std::vector<uint8_t> buildFrame(uint8_t opcode, const std::vector<uint8_t> &data);
    void sendFrame(const std::vector<uint8_t> &frame);
};

#endif