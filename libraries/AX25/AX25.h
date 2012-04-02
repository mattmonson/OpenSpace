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
	static const u32 c_PrefixZeroesCount = 100;
	static const u32 c_PrefixFlagsCount = 3;
	static const u32 c_SuffixFlagsCount = 50;
	static const u32 c_BufferSize = (c_PrefixFlagsCount + 7 + 7 + 56 + 1 + 1 + 256 + 2 + c_SuffixFlagsCount) * 10 / 8; // the * at the end is to account for bit stuffing
	
public:
	AX25Packet();
	
	void MicECompress(AX25Address* dest, char* info, f32 lat, f32 lon, s32 altMeters, f32 speedMetersPerSecond, u32 courseDeg, char symbol, char table) const;
	void build(const AX25Address& src, const AX25Address& dest, const AX25Address* path, const u8 pathCount, const char* message);

	typedef BitStream<c_BufferSize> AX25BitStream;
	const AX25BitStream& getBitStream() const;
	
private:
	void addFlagByte(u8 byte);
	void addByte(u8 byte);
	void addAddress(const AX25Address& address, bool isLast);
	void crcBit(u8 bit);
	
	AX25BitStream m_BitStream;
	u16 m_CRC;
	u8 m_ConsecutiveOnes;
	

public:
	void setPTTPin(s16 PTTPin);
	void transmit(Sinewave* pSinewave);
	u32 getTransmissionTime() const; 	// in ms
	bool transmitting() const;
	
private:
	static void staticBaudCallback(void* pContext, Sinewave* pSinewave);
	void baudCallback(Sinewave* pSinewave);

private:
	s16 m_PTTPin;
	u32 m_TransmitBitIndex;
	volatile bool m_Transmitting;
};

#endif
