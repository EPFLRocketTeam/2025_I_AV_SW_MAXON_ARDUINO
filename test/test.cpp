#include <Arduino.h>
#include <vector>

// ========== CONFIG ==========
#define EPOS_SERIAL Serial1   // UART to EPOS4 (change if needed)
#define DEBUG_SERIAL Serial   // Debug output (Serial Monitor)
#define BAUDRATE 115200
// ============================

// Calculate CRC (EPOS4 style, CCITT, x^16 + x^12 + x^5 + 1)
uint16_t calcCRC(uint16_t* dataArray, uint8_t numWords) 
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

void addStuffedByte(std::vector<uint8_t> &frame, uint8_t byte) 
{
  frame.push_back(byte);
  if (byte == 0x90) frame.push_back(0x90);  // Character stuffing
}

std::vector<uint8_t> buildFrame(uint8_t opcode, std::vector<uint8_t> data) 
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
  for (uint8_t byte : data) addStuffedByte(frame, byte);
  addStuffedByte(frame, crc & 0xFF);         // CRC low byte
  addStuffedByte(frame, (crc >> 8) & 0xFF);  // CRC high byte

  return frame;
}

void sendFrame(std::vector<uint8_t> &frame) 
{
  for (uint8_t b : frame) 
    EPOS_SERIAL.write(b);
}

void printReply() 
{
  unsigned long startTime = millis();
  while (!EPOS_SERIAL.available()) 
  {
    if (millis() - startTime > 1000) 
    {
      DEBUG_SERIAL.println("Timeout: no response");
      return;
    }
  }

  DEBUG_SERIAL.print("Reply: ");
  while (EPOS_SERIAL.available()) 
  {
    uint8_t b = EPOS_SERIAL.read();
    DEBUG_SERIAL.print("0x");
    if (b < 0x10) DEBUG_SERIAL.print("0");
    DEBUG_SERIAL.print(b, HEX);
    DEBUG_SERIAL.print(" ");
  }
  DEBUG_SERIAL.println();
}

/////////////////////////////////////////////////////////////////////////////////////////

void printHexFrame(const std::vector<uint8_t>& frame, const char* label = "Frame") 
{
  Serial.print(label);
  Serial.print(": ");
  for (uint8_t byte : frame) {
    Serial.print("0x");
    if (byte < 0x10) Serial.print("0"); // Leading zero for 1-digit hex
    Serial.print(byte, HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setControlWord(uint8_t byte) 
{
  std::vector<uint8_t> data =  {0x01, 0x40, 0x60, 0x00,   // NodeID | Index of Object | Subindex of Object
                                byte, 0x00, 0x00, 0x00};  // Data Bytes to write
  auto frame = buildFrame(0x68, data);
  printHexFrame(frame);
  sendFrame(frame);
  DEBUG_SERIAL.println("set control word.");
  delay(20);
  printReply();
}

void setTargetPosition1()
{
  std::vector<uint8_t> data =  {0x01, 0x7A, 0x60, 0x00,   // NodeID | Index of Object | Subindex of Object
                                0x00, 0x00, 0x00, 0x00};  // Data Bytes to write
  auto frame = buildFrame(0x68, data);
  printHexFrame(frame);
  sendFrame(frame);
  DEBUG_SERIAL.println("Sent POS command.");
  delay(20);
  printReply();
}

void setTargetPosition2()
{
  std::vector<uint8_t> data =  {0x01, 0x7A, 0x60, 0x00,   // NodeID | Index of Object | Subindex of Object
                                0x50, 0xC3, 0x00, 0x00};  // Data Bytes to write
  auto frame = buildFrame(0x68, data);
  printHexFrame(frame);
  sendFrame(frame);
  DEBUG_SERIAL.println("Sent POS command.");
  delay(20);
  printReply();
}

///////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  DEBUG_SERIAL.begin(115200);
  EPOS_SERIAL.begin(115200);  // Match EPOS4 baudrate (check 0x2002 object in EPOS Studio)

  DEBUG_SERIAL.println("Starting EPOS4 UART test...");
  delay(500);

  setControlWord(0x0F);  // You can repeat or extend with more commands
}

void loop() 
{
  setControlWord(0x0F);
  setTargetPosition1();
  setControlWord(0x3F);

  delay(500);

  setControlWord(0x0F);
  setTargetPosition2();
  setControlWord(0x3F);

  delay(500);
}
