#include "BMP085.h"
#include <Wire.h>

namespace
{
	const u8 I2C_ADDRESS                = 0x77;

	const u8 REGISTER_CALIBRATION_START = 0xAA;
	const u8 REGISTER_CONTROL           = 0xF4;
	const u8 REGISTER_OUTPUT            = 0xF6;

	// possible values for the control register
	const u8 CONTROL_MEASURE_TEMP       = 0x2E;
	const u8 CONTROL_MEASURE_PRESSURE   = 0x34;
}

BMP085::BMP085() :
	m_OSS(0),
	m_ReferencePressureInPa(101325),
	m_TempInDeciC(0),
	m_PressureInPa(0),
	m_State(EState::Start),
	m_StateStart(0)
{
}

void BMP085::setup()
{
	// read the configuration data
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.send(REGISTER_CALIBRATION_START);
	Wire.endTransmission();

	Wire.requestFrom(I2C_ADDRESS, (u8)22);
	m_AC1 = WireReceiveBigEndian<s16>();
	m_AC2 = WireReceiveBigEndian<s16>();
	m_AC3 = WireReceiveBigEndian<s16>();
	m_AC4 = WireReceiveBigEndian<u16>();
	m_AC5 = WireReceiveBigEndian<u16>();
	m_AC6 = WireReceiveBigEndian<u16>();
	m_B1  = WireReceiveBigEndian<s16>();
	m_B2  = WireReceiveBigEndian<s16>();
	m_MB  = WireReceiveBigEndian<s16>();
	m_MC  = WireReceiveBigEndian<s16>();
	m_MD  = WireReceiveBigEndian<s16>();
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
		if (millis() - m_StateStart < (2 + (3 << m_OSS)))
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

void BMP085::SetOversamplingSetting(u8 oss)
{
	m_OSS = Clamp<u8>(oss, 0, 3);
}

void BMP085::SetReferencePressure(s32 referencePressureInPa)
{
	m_ReferencePressureInPa = referencePressureInPa;
}

s32 BMP085::GetReferencePressureInPa() const
{
	return m_ReferencePressureInPa;
}

s32 BMP085::GetTempInDeciC() const
{
	return m_TempInDeciC;
}

f32 BMP085::GetTempInC() const
{
	return m_TempInDeciC * 0.1f;
}

s32 BMP085::GetPressureInPa() const
{
	return m_PressureInPa;
}

f32 BMP085::GetAltitudeInM() const
{
	return 44330.0f * (1.0f - pow(m_PressureInPa / (f32)m_ReferencePressureInPa, 1 / 5.255f));
}

void BMP085::RequestTemp()
{
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.send(REGISTER_CONTROL);
	Wire.send(CONTROL_MEASURE_TEMP);
	Wire.endTransmission();
}

s32 BMP085::GetRawTemp()
{
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.send(REGISTER_OUTPUT);
	Wire.endTransmission();

	Wire.requestFrom(I2C_ADDRESS, (u8)2);
	return (s32)WireReceiveBigEndian<u16>();
}

void BMP085::RequestPressure()
{
	u8 ossPressure = CONTROL_MEASURE_PRESSURE + (m_OSS << 6);
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.send(REGISTER_CONTROL);
	Wire.send(ossPressure);
	Wire.endTransmission();
}

s32 BMP085::GetRawPressure()
{
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.send(REGISTER_OUTPUT);
	Wire.endTransmission();

	Wire.requestFrom(I2C_ADDRESS, (u8)3);
	return ((s32)WireReceiveBigEndian<u16>() << 8 | (s32)WireReceiveBigEndian<u8>()) >> (8 - m_OSS);
}

void BMP085::ProcessRawReadings()
{
	s32 x1 = ((m_UT - m_AC6) * m_AC5) >> 15;
	s32 x2 = ((s32)m_MC << 11) / (x1 + m_MD);
	s32 b5 = x1 + x2;
	m_TempInDeciC = (b5 + 8) >> 4;

	s32 b6 = b5 - 4000;
	x1 = (m_B2 * (b6 * b6 >> 12)) >> 11;
	x2 = m_AC2 * b6 >> 11;
	s32 x3 = x1 + x2;
	s32 b3 = (((m_AC1 * 4 + x3) << m_OSS) + 2) >> 2;
	x1 = m_AC3 * b6 >> 13;
	x2 = (m_B1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	u32 b4 = m_AC4 * (u32)(x3 + 32768) >> 15;
	u32 b7 = ((u32)m_UP - b3) * (50000 >> m_OSS);
	s32 p = (b7 < 0x80000000 ? b7 * 2 / b4 : b7 / b4 * 2);
	s32 p1 = p;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	m_PressureInPa = p + ((x1 + x2 + 3791) >> 4);
}