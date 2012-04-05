#include "XTendAPI.h"

//#define PACKET_DEBUGGING

const uint8_t c_StartDelimeter 			= 0x7E;
const uint8_t c_Option_Standard			= 0x00;
const uint8_t c_Option_DisableACK                    = 0x01;

XTendAPI::XTendAPI(Stream* pStream) :
	m_pStream(pStream),
	m_NextSection(ENextSection::PacketStart)
{
}

void XTendAPI::SendTo(const Address& dest, const uint8_t* data, uint16_t size)
{
	TransmitRaw(c_StartDelimeter);                           // packet start byte

	const uint16_t totalSize = 1 + 1 + 2 + 1 + size;
	TransmitRawSwapped((const uint8_t*)&totalSize, 2);            // total packet size

	uint8_t cksum = 0;
	cksum += TransmitRaw(c_APIIdentifier_Transmit);          // transmit request
	cksum += TransmitRaw(0x00);                              // frame ID (zero = no ack)
	cksum += TransmitRawSwapped((const uint8_t*)&dest, 2);        // dest 16bit address
	cksum += TransmitRaw(c_Option_DisableACK);               // option
	cksum += TransmitRaw(data, size);                        // payload

	TransmitRaw(0xFF - cksum);                               // checksum (0xFF - sum of everything after size)
}

bool XTendAPI::Receive(const XTendAPI::Frame** ppFrame)
{
	(*ppFrame) = NULL;
	
	while (m_NextSection == ENextSection::PacketStart)
	{
		if (m_pStream->available() < 1)
			return false;

		if ((uint8_t)m_pStream->read() == c_StartDelimeter)
		{
			m_Frame.m_Checksum = 0;
			m_NextSection = ENextSection::PacketLength;
		}
	}
			
	if (m_NextSection == ENextSection::PacketLength)
	{
		if (m_pStream->available() < 2)
			return false;

		ReceiveRawSwapped((uint8_t*)&m_Frame.m_Length, sizeof(m_Frame.m_Length));
		m_LengthRemaining = m_Frame.m_Length;
		
		if (m_LengthRemaining < 1)
		{
#ifdef PACKET_DEBUGGING
			serprintf(Serial, "%lu: XTendAPI: Going to ENextSection::Failure because there's not enough data to read an API Identifier!\n", millis());
#endif
			m_NextSection = ENextSection::Failure;
		}
		else
			m_NextSection = ENextSection::APIIdentifier;
	}
	
	if (m_NextSection == ENextSection::APIIdentifier)
	{
		if (m_pStream->available() < 1)
			return false;

		m_Frame.m_Checksum += ReceiveRaw(m_Frame.m_APIIdentifier);
		m_LengthRemaining--;

		if (m_Frame.m_APIIdentifier != c_APIIdentifier_Receive)
		{
#ifdef PACKET_DEBUGGING
			serprintf(Serial, "%lu: XTendAPI: Going to ENextSection::Failure because of unhandled API Identifier %hu!\n", millis(), m_Frame.m_APIIdentifier);
#endif
			m_NextSection = ENextSection::Failure;
		}
		else if (m_LengthRemaining < 2 + 1 + 1)
		{
#ifdef PACKET_DEBUGGING
			serprintf(Serial, "%lu: XTendAPI: Going to ENextSection::Failure because there's not enough data (only %u bytes) to read the packet header bits\n", millis(), m_LengthRemaining);
#endif
			m_NextSection = ENextSection::Failure;
		}
		else
			m_NextSection = ENextSection::SrcAddressAndOptions;
	}

	if (m_NextSection == ENextSection::SrcAddressAndOptions)
	{
		if (m_pStream->available() < 2 + 1 + 1)
			return false;

		m_Frame.m_Checksum += ReceiveRawSwapped((uint8_t*)&m_Frame.m_SrcAddress, sizeof(m_Frame.m_SrcAddress));
		m_Frame.m_Checksum += ReceiveRaw(m_Frame.m_RSSI);
		m_Frame.m_Checksum += ReceiveRaw(m_Frame.m_Options);
		m_LengthRemaining -= 2 + 1 + 1;

		m_NextSection = ENextSection::FrameData;
	}

	if (m_NextSection == ENextSection::FrameData)
	{
		if (m_LengthRemaining > sizeof(m_Frame.m_Payload))
		{
#ifdef PACKET_DEBUGGING
			serprintf(Serial, "%lu: XTendAPI: Going to ENextSection::Failure because there's too much data (%u bytes) in the packet\n", millis(), m_LengthRemaining);
#endif
			m_NextSection = ENextSection::Failure;
		}
		else if ((uint16_t)m_pStream->available() < m_LengthRemaining)
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
		if (m_pStream->available() < 1)
			return false;
			
		uint8_t computedChecksum = m_Frame.m_Checksum;
		m_Frame.m_Checksum = m_pStream->read();

		bool checksumPassed = (m_Frame.m_Checksum == 0xFF - computedChecksum);
		m_NextSection = ENextSection::PacketStart;

		if (checksumPassed)
			(*ppFrame) = &m_Frame;
#ifdef PACKET_DEBUGGING
		else
		{
			serprintf(Serial, "%lu: XTendAPI: Failed checksum\n", millis());
		}
#endif

		return true;
	}
	
	if (m_NextSection == ENextSection::Failure)
	{
#ifdef PACKET_DEBUGGING
		serprintf(Serial, "%lu: XTendAPI: In failure state with %u bytes remaining\n", millis(), m_LengthRemaining);
#endif
		
		m_NextSection = ENextSection::PacketStart;
		return true;
	}
	
	return false;
}

uint8_t XTendAPI::TransmitRaw(uint8_t data)
{
	m_pStream->write(data);
	return data;
}

uint8_t XTendAPI::TransmitRaw(const uint8_t* data, uint16_t size)
{
	uint8_t sum = 0;
	for (uint16_t i=0; i<size; ++i)
		sum += TransmitRaw(data[i]);
	return sum;
}

uint8_t XTendAPI::TransmitRawSwapped(const uint8_t* data, uint16_t size)
{
	uint8_t sum = 0;
	for (uint16_t i=0; i<size; ++i)
		sum += TransmitRaw(data[size - 1 - i]);
	return sum;
}

uint8_t XTendAPI::ReceiveRaw(uint8_t& data)
{
	data = m_pStream->read();
	return data;
}

uint8_t XTendAPI::ReceiveRaw(uint8_t* data, uint16_t size)
{
	uint8_t sum = 0;
	for (uint16_t i=0; i<size; ++i)
		sum += ReceiveRaw(data[i]);
	return sum;
}

uint8_t XTendAPI::ReceiveRawSwapped(uint8_t* data, uint16_t size)
{
	uint8_t sum = 0;
	for (uint16_t i=0; i<size; ++i)
		sum += ReceiveRaw(data[size - 1 - i]);
	return sum;
}

