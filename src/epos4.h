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
    READ_STATUS,
    PPM,
    HOMING,
    FAULT,
    READ_POSITION_ACTUAL_VALUE,
    READ_CURRENT_ACTUAL_VALUE,
};

enum class PPMState {
    SET_OPERATION_MODE,
    CHECK_OPERATION_MODE,
    SET_PROFILE_VELOCITY,
    SET_PROFILE_ACCELERATION,
    SET_PROFILE_DECELERATION,
    SET_NOMINAL_CURRENT,
    SET_OUTPUT_CURRENT_LIMIT,
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
    READY_TO_SWITCH_ON,
    ENABLE,
    OPERATION_ENABLED,
    START_HOMING,
    IN_PROGRESS,
};

struct PPMConfig
{
    DWORD target_position = 0;
    DWORD profile_velocity = 7000;
    DWORD profile_acceleration= 10000;
    DWORD profile_deceleration = 20000;
    DWORD nominal_current = 4870;
    DWORD output_current_limit = 4500;
};

struct HomingConfig
{
    DWORD homing_offset_distance = 0;
    DWORD home_position = 0;
    DWORD speed_for_switch_search = 600;
    DWORD speed_for_zero_search = 600;
    DWORD homing_acceleration = 3000;
    DWORD homing_current = 600; // in mA
};

/**
 * @class EPOS4
 * @brief Asynchronous, non-blocking interface for the Maxon EPOS4 motor controller.
 *
 * Communication is handled via UART and relies on frequent calls to tick() 
 * in the main loop to process incoming/outgoing frames.
 */

class EPOS4 
{
public:
    /**
     * @brief Construct a new EPOS4 object.
     * 
     * @param eposSerial Reference to the hardware serial port connected to the EPOS4.
     * @param baudrate UART baudrate (default: 115200).
     */
    EPOS4(HardwareSerial &eposSerial, unsigned long baudrate = 115200);

    /**
     * @brief Main update function for the asynchronous driver.
     * 
     * Must be called as frequently as possible in the main loop.
     * Handles communication with the EPOS4, processes responses,
     * and advances internal state machines.
     */
    void tick();

    /**
     * @brief Write a value to the EPOS4 Object Dictionary (blocking).
     * 
     * @param nodeID Node ID of the device.
     * @param index Object index.
     * @param sub_index Object sub-index.
     * @param value Value to write.
     * @param errorCode Reference to store the error code.
     */
    void writeObject(BYTE nodeID, WORD index, BYTE sub_index, const DWORD& value, DWORD& errorCode);
    
    /**
     * @brief Read a value from the EPOS4 Object Dictionary (blocking).
     * 
     * @param nodeID Node ID of the device.
     * @param index Object index.
     * @param sub_index Object sub-index.
     * @param errorCode Reference to store the error code.
     * @return DWORD Value read from the object.
     */
    DWORD readObject(BYTE nodeID, WORD index, BYTE sub_index, DWORD& errorCode);

    /**
     * @brief Start a non-blocking write to the Object Dictionary.
     * 
     * Must be followed by calls to pollWriteObject() until it returns true.
     */
    void startWriteObject(BYTE nodeID, WORD index, BYTE sub_index, const DWORD& value);
    
    /**
     * @brief Poll the result of a non-blocking write.
     * 
     * @param errorCode Reference to store the error code.
     * @return true if the write is complete, false if still in progress.
     */
    bool pollWriteObject(DWORD& errorCode);

    /**
     * @brief Start a non-blocking read from the Object Dictionary.
     * 
     * Must be followed by calls to pollReadObject() until it returns true.
     */
    void startReadObject(BYTE nodeID, WORD index, BYTE sub_index);

    /**
     * @brief Poll the result of a non-blocking read.
     * 
     * @param value Reference to store the read value.
     * @param errorCode Reference to store the error code.
     * @return true if the read is complete, false if still in progress.
     */
    bool pollReadObject(DWORD& value, DWORD& errorCode);

    /**
     * @brief Command a move in Profile Position Mode (PPM).
     * 
     * @param position Target position in counts.
     */
    void go_to_position(const DWORD position);

    /**
     * @brief Start a homing sequence using current threshold detection.
     * 
     * Must be followed by periodic calls to tick() until get_homing_done() returns true.
     */
    void current_threshold_homing();

    /// @return true if the driver is currently performing a read operation.
    bool get_isReading() { return isReading; }

    /// @return true if the driver is currently performing a write operation.
    bool get_isWriting() { return isWriting; }

    /// @brief Set the target position for Profile Position Mode.
    void set_target_position(const DWORD value) { ppm_cfg.target_position = value; }
    
    /// @brief Set the profile velocity for Profile Position Mode.
    void set_profile_velocity(const DWORD value) { ppm_cfg.profile_velocity = value; }

    /// @brief Set the profile acceleration for Profile Position Mode.
    void set_profile_acceleration(const DWORD value) { ppm_cfg.profile_acceleration = value; }

    /// @brief Set the profile deceleration for Profile Position Mode.
    void set_profile_deceleration(const DWORD value) { ppm_cfg.profile_deceleration= value; }

    /// @brief Set nominal current for Profile Position Mode.
    void set_nominal_current(const DWORD value) { ppm_cfg.nominal_current= value; }

    /// @brief Set max output current for Profile Position Mode.
    void set_output_current_limit(const DWORD value) { ppm_cfg.output_current_limit= value; }

    /// @brief Set the homing offset distance.
    void set_homing_offset_distance(const DWORD value) { homing_cfg.homing_offset_distance = value; }
    
    /// @brief Set the home position after homing.
    void set_home_position(const DWORD value) { homing_cfg.home_position = value; }
    
    /// @brief Set the speed used for the switch search phase of homing.
    void set_homing_speed_for_switch_search(const DWORD value) { homing_cfg.speed_for_switch_search = value; }
    
    /// @brief Set the speed used for the zero search phase of homing.
    void set_homing_speed_for_zero_search(const DWORD value) { homing_cfg.speed_for_zero_search = value; }
    
    /// @brief Read position actual value from the EPOS4.
    DWORD read_position();

    /// @brief Read current actual value from the EPOS4.
    DWORD read_current();

    void set_homing_acceleration(const DWORD value) { homing_cfg.homing_acceleration = value; }
    
    void set_homing_current(const DWORD value) { homing_cfg.homing_current = value; }

    bool get_homing_done() { return homing_done; }
    bool homingError() { return epos_status.HomingError(); }

    void reset();

private:
    HardwareSerial &eposSerial;
    unsigned long baudrate;
    unsigned long read_timeout;
    unsigned long min_tick_time;
    unsigned long startTime;
    unsigned long last_tick_time;
    bool timeout;
    bool homing_done;
    bool homming_error;
    bool isReading, isWriting;
    bool read_position_actual_value_queued, read_current_actual_value_queued;

    DriverState driver_state;
    DriverState working_state;
    PPMState ppm_state;
    HomingState homing_state;
    STATUS epos_status;
    DWORD observed_mode;
    DWORD epos_position_actual_value;
    DWORD epos_current_actual_value;
    
    PPMConfig       ppm_cfg;
    HomingConfig    homing_cfg;

    DriverState determineRead();
    void runPPM();
    void runHoming(bool direction = true);
    void fault();

    uint16_t calcCRC(uint16_t* dataArray, uint8_t numWords);
    void addStuffedByte(std::vector<uint8_t> &frame, uint8_t byte);
    std::vector<uint8_t> buildFrame(uint8_t opcode, const std::vector<uint8_t> &data);
    void sendFrame(const std::vector<uint8_t> &frame);

    uint8_t rx_buffer[24];
    uint8_t rx_index;
    uint8_t rx_len;

    void executeHomingStep(HomingState nextState, WORD writeIndex, BYTE writeSubIndex, DWORD writeValue, const char* debugName);
    uint8_t hommingWriteRetriesCount = 0;
    unsigned long hommingWriteStartTime = 0;
    uint8_t hommingWaitRetriesCount = 0;
    unsigned long hommingWaitStartTime = 0;
    unsigned long readStatusStartTime = 0;
    uint8_t readStatusRetriesCount = 0;
    void readStatus();
};

#endif