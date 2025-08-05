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
    explicit STATUS(DWORD value) : raw(value) {}

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


class EPOS4 
{
public:
    EPOS4(HardwareSerial &eposSerial, unsigned long baudrate = 115200);

    void writeObject(BYTE nodeID, WORD index, BYTE sub_index, const DWORD& value, DWORD& errorCode);
    DWORD readObject(BYTE nodeID, WORD index, BYTE sub_index, DWORD& errorCode);

    void go_to_position(const DWORD& position);
    void current_threshold_homing(DWORD home_offset_move_distance = 0);

private:
    HardwareSerial &eposSerial;
    unsigned long baudrate;
    unsigned long read_timeout;
    unsigned long homing_timeout;

    uint16_t calcCRC(uint16_t* dataArray, uint8_t numWords);
    void addStuffedByte(std::vector<uint8_t> &frame, uint8_t byte);
    std::vector<uint8_t> buildFrame(uint8_t opcode, const std::vector<uint8_t> &data);
    void sendFrame(const std::vector<uint8_t> &frame);
};

#endif