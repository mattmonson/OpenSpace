#include "BMP085.h"
#include <Wire.h>

namespace
{
	const uint8_t I2C_ADDRESS                = 0x77;

	const uint8_t REGISTER_CALIBRATION_START = 0xAA;
	const uint8_t REGISTER_CONTROL           = 0xF4;
	const uint8_t REGISTER_OUTPUT            = 0xF6;

	// possible values for the control register
	const uint8_t CONTROL_MEASURE_TEMP       = 0x2E;
	const uint8_t CONTROL_MEASURE_PRESSURE   = 0x34;
}

BMP085::BMP085() :
	m_State(EState::Start),
	m_StateStart(0),
	m_OSS(0),
	m_ReferencePressureInPa(101325),
	m_TempInDeciC(0),
	m_PressureInPa(0)
{
}

void BMP085::setup()
{
	// read the configuration data
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(REGISTER_CALIBRATION_START);
	Wire.endTransmission();

	Wire.requestFrom(I2C_ADDRESS, (uint8_t)22);
	m_AC1 = WireReceiveBigEndian<int16_t>();
	m_AC2 = WireReceiveBigEndian<int16_t>();
	m_AC3 = WireReceiveBigEndian<int16_t>();
	m_AC4 = WireReceiveBigEndian<uint16_t>();
	m_AC5 = WireReceiveBigEndian<uint16_t>();
	m_AC6 = WireReceiveBigEndian<uint16_t>();
	m_B1  = WireReceiveBigEndian<int16_t>();
	m_B2  = WireReceiveBigEndian<int16_t>();
	m_MB  = WireReceiveBigEndian<int16_t>();
	m_MC  = WireReceiveBigEndian<int16_t>();
	m_MD  = WireReceiveBigEndian<int16_t>();
}

void BMP085::loop()
{
	RequestTemp();
	delay(5);
	m_UT = GetRawTemp();
	
	RequestPressure();
	delay(2 + (3 << m_OSS));
	m_UP = GetRawPressure();

	ProcessRawReadings();
	
	// fix up the state stuff so that things don't go horribly wrong if we go back into async mode
	m_State = EState::Start;
	m_StateStart = millis();
}

void BMP085::loopAsync()
{
	switch(m_State)
	{
	case EState::WaitForTemp:
		if (millis() - m_StateStart < 5)
			break;
		
		m_UT = GetRawTemp();
		RequestPressure();

		m_State = EState::WaitForPressure;
		m_StateStart = millis();
		break;
		
	case EState::WaitForPressure:
		if (millis() - m_StateStart < (2 + ((uint32_t)3 << m_OSS)))
			break;
		
		m_UP = GetRawPressure();
		ProcessRawReadings();
		
	case EState::Start:
		RequestTemp();
		
		m_State = EState::WaitForTemp;
		m_StateStart = millis();
		break;
	}
}

void BMP085::SetOversamplingSetting(uint8_t oss)
{
	m_OSS = Clamp<uint8_t>(oss, 0, 3);
}

void BMP085::SetReferencePressure(int32_t referencePressureInPa)
{
	m_ReferencePressureInPa = referencePressureInPa;
}

int32_t BMP085::GetReferencePressureInPa() const
{
	return m_ReferencePressureInPa;
}

int32_t BMP085::GetTempInDeciC() const
{
	return m_TempInDeciC;
}

float BMP085::GetTempInC() const
{
	return m_TempInDeciC * 0.1f;
}

int32_t BMP085::GetPressureInPa() const
{
	return m_PressureInPa;
}

float BMP085::GetAltitudeInM() const
{
	return 44330.0f * (1.0f - pow(m_PressureInPa / (float)m_ReferencePressureInPa, 1 / 5.255f));
}

void BMP085::RequestTemp()
{
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(REGISTER_CONTROL);
	Wire.write(CONTROL_MEASURE_TEMP);
	Wire.endTransmission();
}

int32_t BMP085::GetRawTemp()
{
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(REGISTER_OUTPUT);
	Wire.endTransmission();

	Wire.requestFrom(I2C_ADDRESS, (uint8_t)2);
	return (int32_t)WireReceiveBigEndian<uint16_t>();
}

void BMP085::RequestPressure()
{
	uint8_t ossPressure = CONTROL_MEASURE_PRESSURE + (m_OSS << 6);
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(REGISTER_CONTROL);
	Wire.write(ossPressure);
	Wire.endTransmission();
}

int32_t BMP085::GetRawPressure()
{
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(REGISTER_OUTPUT);
	Wire.endTransmission();

	Wire.requestFrom(I2C_ADDRESS, (uint8_t)3);
	return ((int32_t)WireReceiveBigEndian<uint16_t>() << 8 | (int32_t)WireReceiveBigEndian<uint8_t>()) >> (8 - m_OSS);
}

void BMP085::ProcessRawReadings()
{
	int32_t x1 = ((m_UT - m_AC6) * m_AC5) >> 15;
	int32_t x2 = ((int32_t)m_MC << 11) / (x1 + m_MD);
	int32_t b5 = x1 + x2;
	m_TempInDeciC = (b5 + 8) >> 4;

	int32_t b6 = b5 - 4000;
	x1 = (m_B2 * (b6 * b6 >> 12)) >> 11;
	x2 = m_AC2 * b6 >> 11;
	int32_t x3 = x1 + x2;
	int32_t b3 = (((m_AC1 * 4 + x3) << m_OSS) + 2) >> 2;
	x1 = m_AC3 * b6 >> 13;
	x2 = (m_B1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	uint32_t b4 = m_AC4 * (uint32_t)(x3 + 32768) >> 15;
	uint32_t b7 = ((uint32_t)m_UP - b3) * (50000 >> m_OSS);
	int32_t p = (b7 < 0x80000000 ? b7 * 2 / b4 : b7 / b4 * 2);
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	m_PressureInPa = p + ((x1 + x2 + 3791) >> 4);
}
