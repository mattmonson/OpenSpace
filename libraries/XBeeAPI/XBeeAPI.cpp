#include "XBeeAPI.h"
#include <HardwareSerial.h>

#define PACKET_DEBUGGING

XBeeAPI::XBeeAPI(HardwareSerial* pSerial) :
	m_pSerial(pSerial),
	m_NextSection(ENextSection::PacketStart)
{
}

void XBeeAPI::setup(u32 baud)
{
	m_pSerial->begin(baud);
}

void XBeeAPI::SendTo(const Address& dest, const u8* data, u16 size)
{
	TransmitRaw(0x7E);                                       // packet start byte

	const u16 totalSize = 1 + 1 + 8 + 2 + 1 + 1 + size;
	TransmitRawSwapped((const u8*)&totalSize, 2);            // total packet size

	u8 cksum = 0;
	cksum += TransmitRaw(APIIdentifier_Transmit);            // transmit request
	cksum += TransmitRaw(0x00);                              // frame ID (zero = no ack)
	cksum += TransmitRawSwapped((const u8*)&dest, 8); 		 // dest 64bit address
	cksum += TransmitRaw(0xFF);                              // dest 16bit address
	cksum += TransmitRaw(0xFE);                              // -- 0xFFFE means 'dont know'
	cksum += TransmitRaw(0x00);                              // broadcast radius, 0x00 means max
	cksum += TransmitRaw(0x00);                              // options
	cksum += TransmitRaw(data, size);                        // payload

	TransmitRaw(0xFF - cksum);                               // checksum (0xFF - sum of everything after size)
}

bool XBeeAPI::Receive(const XBeeAPI::Frame** ppFrame)
{
	(*ppFrame) = NULL;
	
	while (m_NextSection == ENextSection::PacketStart)
	{
		if (m_pSerial->available() < 1)
			return false;

		if (m_pSerial->read() == 0x7E)
		{
			m_Frame.m_Checksum = 0;
			m_NextSection = ENextSection::PacketLength;
		}
	}
			
	if (m_NextSection == ENextSection::PacketLength)
	{
		if (m_pSerial->available() < 2)
			return false;

		ReceiveRawSwapped((u8*)&m_Frame.m_Length, sizeof(m_Frame.m_Length));
		m_LengthRemaining = m_Frame.m_Length;
		
		if (m_LengthRemaining < 1)
		{
#ifdef PACKET_DEBUGGING
			serprintf(Serial, "%lu: XBeeAPI: Going to ENextSection::Failure because there's not enough data to read an API Identifier!\n", millis());
#endif
			m_NextSection = ENextSection::Failure;
		}
		else
			m_NextSection = ENextSection::APIIdentifier;
	}
	
	if (m_NextSection == ENextSection::APIIdentifier)
	{
		if (m_pSerial->available() < 1)
			return false;

		m_Frame.m_Checksum += ReceiveRaw(m_Frame.m_APIIdentifier);
		m_LengthRemaining--;

		if (m_Frame.m_APIIdentifier != APIIdentifier_Receive)
		{
#ifdef PACKET_DEBUGGING
			serprintf(Serial, "%lu: XBeeAPI: Going to ENextSection::Failure because of unhandled API Identifier %hu!\n", millis(), m_Frame.m_APIIdentifier);
#endif
			m_NextSection = ENextSection::Failure;
		}
		else if (m_LengthRemaining < 11)
		{
#ifdef PACKET_DEBUGGING
			serprintf(Serial, "%lu: XBeeAPI: Going to ENextSection::Failure because there's not enough data (only %u bytes) to read the packet header bits\n", millis(), m_LengthRemaining);
#endif
			m_NextSection = ENextSection::Failure;
		}
		else
			m_NextSection = ENextSection::SrcAddressAndOptions;
	}

	if (m_NextSection == ENextSection::SrcAddressAndOptions)
	{
		if (m_pSerial->available() < 11)
			return false;

		m_Frame.m_Checksum += ReceiveRawSwapped((u8*)&m_Frame.m_SrcAddress, sizeof(m_Frame.m_SrcAddress));
		m_Frame.m_Checksum += ReceiveRawSwapped((u8*)&m_Frame.m_SrcAddressShort, sizeof(m_Frame.m_SrcAddressShort));
		m_Frame.m_Checksum += ReceiveRaw(m_Frame.m_Options);
		m_LengthRemaining -= 11;

		m_NextSection = ENextSection::FrameData;
	}

	if (m_NextSection == ENextSection::FrameData)
	{
		if (m_LengthRemaining > sizeof(m_Frame.m_Payload))
		{
#ifdef PACKET_DEBUGGING
			serprintf(Serial, "%lu: XBeeAPI: Going to ENextSection::Failure because there's too much data (%u bytes) in the packet\n", millis(), m_LengthRemaining);
#endif
			m_NextSection = ENextSection::Failure;
		}
		else if (m_pSerial->available() < m_LengthRemaining)
		{
			return false;
		}
		else
		{
			m_Frame.m_PayloadLength = m_LengthRemaining;
			m_Frame.m_Checksum += ReceiveRaw(m_Frame.m_Payload, m_LengthRemaining);
			m_LengthRemaining = 0;

			m_NextSection = ENextSection::Checksum;
		}
	}
	
	if (m_NextSection == ENextSection::Checksum)
	{
		if (m_pSerial->available() < 1)
			return false;
			
		u8 computedChecksum = m_Frame.m_Checksum;
		m_Frame.m_Checksum = m_pSerial->read();

		bool checksumPassed = (m_Frame.m_Checksum == 0xFF - computedChecksum);
		m_NextSection = ENextSection::PacketStart;

		if (checksumPassed)
			(*ppFrame) = &m_Frame;
#ifdef PACKET_DEBUGGING
		else
		{
			serprintf(Serial, "%lu: XBeeAPI: Failed checksum\n", millis());
		}
#endif

		return true;
	}
	
	if (m_NextSection == ENextSection::Failure)
	{
#ifdef PACKET_DEBUGGING
		serprintf(Serial, "%lu: XBeeAPI: In failure state with %u bytes remaining\n", millis(), m_LengthRemaining);
#endif
		
		m_NextSection = ENextSection::PacketStart;
		return true;
	}
	
	return false;
}

u8 XBeeAPI::TransmitRaw(u8 data)
{
	m_pSerial->print(data, BYTE);
	return data;
}

u8 XBeeAPI::TransmitRaw(const u8* data, u16 size)
{
	u8 sum = 0;
	for (u16 i=0; i<size; ++i)
		sum += TransmitRaw(data[i]);
	return sum;
}

u8 XBeeAPI::TransmitRawSwapped(const u8* data, u16 size)
{
	u8 sum = 0;
	for (u16 i=0; i<size; ++i)
		sum += TransmitRaw(data[size - 1 - i]);
	return sum;
}

u8 XBeeAPI::ReceiveRaw(u8& data)
{
	data = m_pSerial->read();
	return data;
}

u8 XBeeAPI::ReceiveRaw(u8* data, u16 size)
{
	u8 sum = 0;
	for (u16 i=0; i<size; ++i)
		sum += ReceiveRaw(data[i]);
	return sum;
}

u8 XBeeAPI::ReceiveRawSwapped(u8* data, u16 size)
{
	u8 sum = 0;
	for (u16 i=0; i<size; ++i)
		sum += ReceiveRaw(data[size - 1 - i]);
	return sum;
}

