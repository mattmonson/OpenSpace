#pragma once

#include <Arduino.h>

class JonahRX
{
public:
    JonahRX();

    bool onReceive(uint8_t byte);
    uint8_t getDataSize() const { return m_Size; }
    const uint8_t* getData() const { return m_Data; }

private:
    struct EState
    {
        enum Enum
        {
            Size,
            Data,
            CRC,
        };
    };

    EState::Enum m_State;

    uint8_t m_Size;
    uint8_t m_CurrentByte;
    uint32_t m_CRC;

    static const size_t c_MaxPacketSize = 32;
    static const size_t c_BufferSize = c_MaxPacketSize + sizeof(uint32_t); // leave space for the CRC
    uint8_t m_Data[c_BufferSize];
};
