#ifndef _XTENDAPI_H
#define _XTENDAPI_H

#include <Core.h>
#include <Stream.h>

class XTendAPI
{
public:
	typedef u16 Address;

	static const u8 c_APIIdentifier_Transmit 	= 0x01;
	static const u8 c_APIIdentifier_Receive  	= 0x81;

	struct Frame
	{
		u16 m_Length;
		u8 m_APIIdentifier;
		Address m_SrcAddress;
		u8 m_RSSI;
		u8 m_Options;
		u16 m_PayloadLength;
		u8 m_Payload[128];
		u8 m_Checksum;
	};

public:
	XTendAPI(Stream* pStream);

	void SendTo(const Address& dest, const u8* data, u16 size);
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
	u8 TransmitRaw(u8 data);
	u8 TransmitRaw(const u8* data, u16 size);
	u8 TransmitRawSwapped(const u8* data, u16 size);
	
	u8 ReceiveRaw(u8& data);
	u8 ReceiveRaw(u8* data, u16 size);
	u8 ReceiveRawSwapped(u8* data, u16 size);

protected:
	Stream* m_pStream;
	
	ENextSection::Enum m_NextSection;
	u16 m_LengthRemaining;
	Frame m_Frame;
};

#endif

