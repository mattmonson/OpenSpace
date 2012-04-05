#include "AX25.h"
#include <Sinewave.h>

const uint8_t FLAG_BYTE = 0x7E;

AX25Packet::AX25Packet() :
	m_PTTPin(-1),
	m_TransmitBitIndex(0),
	m_Transmitting(false)
{
}

void AX25Packet::MicECompress(AX25Address* dest, char* info, float lat, float lon, int32_t altMeters, float speedMetersPerSecond, uint32_t courseDeg, char symbol, char table) const
{
	//TODO: this doesn't support position ambiguity

	sprintf(dest->m_CallSign, "%.2ld%.2ld%.2ld",
		(int32_t)fabs(lat),
		(int32_t)fabs(lat *  60) % 60,
		(int32_t)fabs(lat * 60 * 100) % 100
	);

	const bool messageIsStandard = 1;
	const bool messageTypeBitA = 1;
	const bool messageTypeBitB = 1;
	const bool messageTypeBitC = 0;

	if (messageTypeBitA)
		dest->m_CallSign[0] += (messageIsStandard ? 'P' : 'A') - '0';
	if (messageTypeBitB)
		dest->m_CallSign[1] += (messageIsStandard ? 'P' : 'A') - '0';
	if (messageTypeBitC)
		dest->m_CallSign[2] += (messageIsStandard ? 'P' : 'A') - '0';
	if (lat >= 0)
		dest->m_CallSign[3] += 'P' - '0';
	if (labs(lon) >= 100 || labs(lon) <= 9) // 0-9 are a special case (!)
		dest->m_CallSign[4] += 'P' - '0';
	if (lon < 0)
		dest->m_CallSign[5] += 'P' - '0';

	dest->m_SSID = 0;

	const int32_t lonDeg = (int32_t)fabs(lon);
	const int32_t lonMin = (int32_t)fabs(lon * 60) % 60;
	const int32_t lonCentiMin = (int32_t)fabs(lon * 60 * 100) % 100;

	const uint32_t speedKnots = (uint32_t)METERS_PER_SECOND_TO_KNOTS(speedMetersPerSecond);

	const bool gpsDataCurrent = 1;
	sprintf(info, "%c%c%c%c%c%c%c%c%c%c%c%c}",
		gpsDataCurrent ? '`' : '\'',
		(char)(lonDeg >= 110 ? lonDeg - 110 + '&' : lonDeg >= 100 ? lonDeg - 100 + 'l' : lonDeg >= 10 ? lonDeg - 10 + '&' : lonDeg + 'v'),
		(char)(lonMin >= 10 ? lonMin - 10 + '&' : lonMin + 'X'),
		(char)(lonCentiMin + 28),
		(char)(speedKnots < 200 ? speedKnots / 10 + 'l' : speedKnots / 10 - 20 + '0'),
		(char)(speedKnots % 10 * 10 + courseDeg / 100 + 32),
		(char)(courseDeg % 100 + 28),
		symbol,
		table,
		(char)((altMeters + 10000l) / 91 / 91 + 33),
		(char)((altMeters + 10000l) / 91 % 91 + 33),
		(char)((altMeters + 10000l) % 91 + 33)
	);
}

void AX25Packet::build(const AX25Address& src, const AX25Address& dest, const AX25Address* path, const uint8_t pathCount, const char* message)
{
	m_BitStream.clear();
	
	m_CRC = 0xFFFF,
	m_ConsecutiveOnes = 0;

	for (uint8_t i=0; i<c_PrefixZeroesCount; ++i)
		addFlagByte(0x00);

	for (uint8_t i=0; i<c_PrefixFlagsCount; ++i)
		addFlagByte(FLAG_BYTE);

	addAddress(dest, false);
	addAddress(src, pathCount == 0);
	
	for (uint8_t i=0; i<pathCount; ++i)
		addAddress(path[i], i+1 == pathCount);
	
	addByte(0x03); // UI frame
	addByte(0xF0); // No layer-3 protocol
	
	while (*message)
		addByte(*message++);
	
	uint16_t finalCRC = m_CRC;
	addByte(~finalCRC & 0xFF);
	addByte(~finalCRC >> 8);
	
	for (uint8_t i=0; i<c_SuffixFlagsCount; ++i)
		addFlagByte(FLAG_BYTE);
}

const AX25Packet::AX25BitStream& AX25Packet::getBitStream() const
{
	return m_BitStream;
}

void AX25Packet::addFlagByte(uint8_t byte)
{
	for (uint8_t i=0; i<8; ++i)
	{
		uint8_t bit = (byte >> i) & 0x1;
		m_BitStream.push_back(bit);
	}
}

void AX25Packet::addByte(uint8_t byte)
{
	for (uint8_t i=0; i<8; ++i)
	{
		uint8_t bit = (byte >> i) & 0x1;

		m_BitStream.push_back(bit);
		crcBit(bit);

		if (!bit)
		{
			m_ConsecutiveOnes = 0;
		}
		else if (++m_ConsecutiveOnes >= 5)
		{
			m_BitStream.push_back(0);
			m_ConsecutiveOnes = 0;
		}
	}
}

void AX25Packet::addAddress(const AX25Address& address, bool isLast)
{
	const char* callSign = address.m_CallSign;
	for (uint8_t i=0; i<6; ++i)
		addByte((*callSign ? *callSign++ : ' ') << 1);
	addByte(((address.m_SSID + '0') << 1) | (isLast ? 0x01 : 0x00));
}

void AX25Packet::crcBit(uint8_t bit)
{
	m_CRC ^= bit;
	if (m_CRC & 0x1)
		m_CRC = (m_CRC >> 1) ^ 0x8408;
	else
		m_CRC = m_CRC >> 1;
}

///// transmitter bits /////

const uint32_t MARK_FREQUENCY = 1200; // Hz
const uint32_t SPACE_FREQUENCY = 2200; // Hz
const uint32_t BAUD = 1200; // bits per second

const uint32_t MARK_SAMPLING_PERIOD  = Sinewave::computeSamplingPeriod(MARK_FREQUENCY);
const uint32_t SPACE_SAMPLING_PERIOD = Sinewave::computeSamplingPeriod(SPACE_FREQUENCY);
const uint32_t SWITCH_SAMPLING_PERIOD = MARK_SAMPLING_PERIOD ^ SPACE_SAMPLING_PERIOD;

void AX25Packet::setPTTPin(int16_t PTTPin)
{
	m_PTTPin = PTTPin;
	if (m_PTTPin != -1)
	{
		pinMode(m_PTTPin, OUTPUT);
		digitalWrite(m_PTTPin, LOW);
	}
}

void AX25Packet::transmit(Sinewave* pSinewave)
{
	if (m_PTTPin != -1)
		digitalWrite(m_PTTPin, HIGH);

	m_Transmitting = true;
	pSinewave->setSamplingPeriod(MARK_SAMPLING_PERIOD);
	pSinewave->setBaudCallback(staticBaudCallback, this, BAUD);
	pSinewave->start();
}

uint32_t AX25Packet::getTransmissionTime() const
{
	return m_BitStream.size() * 1000 / BAUD;
}

bool AX25Packet::transmitting() const
{
	return m_Transmitting;
}

void AX25Packet::staticBaudCallback(void* pContext, Sinewave* pSinewave)
{
	reinterpret_cast<AX25Packet*>(pContext)->baudCallback(pSinewave);
}

void AX25Packet::baudCallback(Sinewave* pSinewave)
{
	// consume the next bit
	++m_TransmitBitIndex;
	if (m_TransmitBitIndex >= m_BitStream.size())
	{
		pSinewave->stop();
		m_TransmitBitIndex = 0;
		m_Transmitting = false;
		
		if (m_PTTPin != -1)
			digitalWrite(m_PTTPin, LOW);
	}
	else if (!m_BitStream[m_TransmitBitIndex])
	{
		pSinewave->setSamplingPeriod(pSinewave->getSamplingPeriod() ^ SWITCH_SAMPLING_PERIOD);
	}
}
