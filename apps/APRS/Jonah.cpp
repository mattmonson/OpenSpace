#include "Jonah.h"
#include <CRC.h>

JonahRX::JonahRX() :
    m_State(EState::Size),
    m_Size(0)
{
}

bool JonahRX::onReceive(uint8_t byte)
{
    if (m_State == EState::Size && byte != 0 && byte <= c_MaxPacketSize)
    {
        // we were waiting for the size, and we have a seemingly valid one, so save it and move on
        m_Size = byte;
        m_CurrentByte = 0;
        m_CRC = crc32_init();
        m_CRC = crc32_update(m_CRC, byte);
        m_State = EState::Data;

        return false;
    }

    if (m_State == EState::Data)
    {
        m_Data[m_CurrentByte++] = byte;
        m_CRC = crc32_update(m_CRC, byte);

        if (m_CurrentByte >= m_Size)
        {
            m_State = EState::CRC;
        }

        return false;
    }

    if (m_State == EState::CRC)
    {
        m_Data[m_CurrentByte++] = byte;

        if (m_CurrentByte >= m_Size + sizeof(m_CRC))
        {
            m_State = EState::Size; // get our state machine back to the start

            m_CRC = crc32_finish(m_CRC);
            const uint32_t crc = *reinterpret_cast<const uint32_t*>(&m_Data[m_CurrentByte - sizeof(m_CRC)]);

            // see if the CRCs match
            return m_CRC == crc;
        }

        return false;
    }
}
