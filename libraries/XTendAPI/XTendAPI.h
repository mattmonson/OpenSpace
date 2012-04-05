#ifndef _XTENDAPI_H
#define _XTENDAPI_H

#include <Core.h>
#include <Stream.h>

class XTendAPI
{
public:
	typedef uint16_t Address;

	static const uint8_t c_APIIdentifier_Transmit 	= 0x01;
	static const uint8_t c_APIIdentifier_Receive  	= 0x81;

	struct Frame
	{
		uint16_t m_Length;
		uint8_t m_APIIdentifier;
		Address m_SrcAddress;
		uint8_t m_RSSI;
		uint8_t m_Options;
		uint16_t m_PayloadLength;
		uint8_t m_Payload[128];
		uint8_t m_Checksum;
	};

public:
	XTendAPI(Stream* pStream);

	void SendTo(const Address& dest, const uint8_t* data, uint16_t size);
	bool Receive(const XTendAPI::Frame** ppFrame);
	
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
	Stream* m_pStream;
	
	ENextSection::Enum m_NextSection;
	uint16_t m_LengthRemaining;
	Frame m_Frame;
};

#endif

