#include "Jonah.h"

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
		m_Checksum = byte;
		m_State = EState::Data;

		return false;
	}

	if (m_State == EState::Data)
	{
		m_Data[m_CurrentByte++] = byte;
		m_Checksum ^= byte;

		if (m_CurrentByte >= m_Size)
		{
			m_State = EState::Checksum;
		}

		return false;
	}

	if (m_State == EState::Checksum)
	{
		m_Checksum ^= byte;
		m_State = EState::Size; // get our state machine back to the start

		if (m_Checksum != 0)
		{
			//printf("Failed checksum test on packet with size %hu\n", m_Size);
		}

		return m_Checksum == 0; // if the checksum is 0, then it's a valid packet
	}
}
