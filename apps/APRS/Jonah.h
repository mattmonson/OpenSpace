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
			Checksum,
		};
	};

	EState::Enum m_State;

	uint8_t m_Size;
	uint8_t m_CurrentByte;
	uint8_t m_Checksum;

	static const size_t c_MaxPacketSize = 32;
	uint8_t m_Data[c_MaxPacketSize];
};
