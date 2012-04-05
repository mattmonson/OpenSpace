#ifndef _XBEEAPI_H
#define _XBEEAPI_H

#include <Core.h>

class HardwareSerial;

class XBeeAPI
{
public:
	static const uint8_t APIIdentifier_Transmit = 0x10;
	static const uint8_t APIIdentifier_Receive  = 0x90;

	struct Address
	{
		uint32_t m_Low, m_High;
	};
	
	typedef uint16_t AddressShort;

	struct Frame
	{
		uint16_t m_Length;
		uint8_t m_APIIdentifier;
		Address m_SrcAddress;
		AddressShort m_SrcAddressShort;
		uint8_t m_Options;
		uint16_t m_PayloadLength;
		uint8_t m_Payload[72];
		uint8_t m_Checksum;
	};

public:
	XBeeAPI(HardwareSerial* pSerial);

	void setup(uint32_t baud);

	void SendTo(const Address& dest, const uint8_t* data, uint16_t size);
	bool Receive(const XBeeAPI::Frame** ppFrame);
	
protected:
	struct ENextSection
	{
		enum Enum
		{
			PacketStart,
            PacketLength,
            APIIdentifier,
            SrcAddressAndOptions,
            FrameData,
            Checksum,
			
			Failure
		};
	};
	
protected:
	uint8_t TransmitRaw(uint8_t data);
	uint8_t TransmitRaw(const uint8_t* data, uint16_t size);
	uint8_t TransmitRawSwapped(const uint8_t* data, uint16_t size);
	
	uint8_t ReceiveRaw(uint8_t& data);
	uint8_t ReceiveRaw(uint8_t* data, uint16_t size);
	uint8_t ReceiveRawSwapped(uint8_t* data, uint16_t size);

protected:
	HardwareSerial* m_pSerial;
	
	ENextSection::Enum m_NextSection;
	uint16_t m_LengthRemaining;
	Frame m_Frame;
};

#endif

