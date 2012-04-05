#ifndef _AX25_H
#define _AX25_H

#include <Core.h>

class Sinewave;

struct AX25Address
{
	char m_CallSign[7];
	char m_SSID;
};

class AX25Packet
{
	static const uint32_t c_PrefixZeroesCount = 100;
	static const uint32_t c_PrefixFlagsCount = 3;
	static const uint32_t c_SuffixFlagsCount = 50;
	static const uint32_t c_BufferSize = (c_PrefixFlagsCount + 7 + 7 + 56 + 1 + 1 + 256 + 2 + c_SuffixFlagsCount) * 10 / 8; // the * at the end is to account for bit stuffing
	
public:
	AX25Packet();
	
	void MicECompress(AX25Address* dest, char* info, float lat, float lon, int32_t altMeters, float speedMetersPerSecond, uint32_t courseDeg, char symbol, char table) const;
	void build(const AX25Address& src, const AX25Address& dest, const AX25Address* path, const uint8_t pathCount, const char* message);

	typedef BitStream<c_BufferSize> AX25BitStream;
	const AX25BitStream& getBitStream() const;
	
private:
	void addFlagByte(uint8_t byte);
	void addByte(uint8_t byte);
	void addAddress(const AX25Address& address, bool isLast);
	void crcBit(uint8_t bit);
	
	AX25BitStream m_BitStream;
	uint16_t m_CRC;
	uint8_t m_ConsecutiveOnes;
	

public:
	void setPTTPin(int16_t PTTPin);
	void transmit(Sinewave* pSinewave);
	uint32_t getTransmissionTime() const; 	// in ms
	bool transmitting() const;
	
private:
	static void staticBaudCallback(void* pContext, Sinewave* pSinewave);
	void baudCallback(Sinewave* pSinewave);

private:
	int16_t m_PTTPin;
	uint32_t m_TransmitBitIndex;
	volatile bool m_Transmitting;
};

#endif
