#ifndef _XBEEAPI_H
#define _XBEEAPI_H

#include <Core.h>

class HardwareSerial;

class XBeeAPI
{
public:
	static const u8 APIIdentifier_Transmit = 0x10;
	static const u8 APIIdentifier_Receive  = 0x90;

	struct Address
	{
		u32 m_Low, m_High;
	};
	
	typedef u16 AddressShort;

	struct Frame
	{
		u16 m_Length;
		u8 m_APIIdentifier;
		Address m_SrcAddress;
		AddressShort m_SrcAddressShort;
		u8 m_Options;
		u16 m_PayloadLength;
		u8 m_Payload[72];
		u8 m_Checksum;
	};

public:
	XBeeAPI(HardwareSerial* pSerial);

	void setup(u32 baud);

	void SendTo(const Address& dest, const u8* data, u16 size);
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
	u8 TransmitRaw(u8 data);
	u8 TransmitRaw(const u8* data, u16 size);
	u8 TransmitRawSwapped(const u8* data, u16 size);
	
	u8 ReceiveRaw(u8& data);
	u8 ReceiveRaw(u8* data, u16 size);
	u8 ReceiveRawSwapped(u8* data, u16 size);

protected:
	HardwareSerial* m_pSerial;
	
	ENextSection::Enum m_NextSection;
	u16 m_LengthRemaining;
	Frame m_Frame;
};

#endif

