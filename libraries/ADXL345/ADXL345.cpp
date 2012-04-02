#include "ADXL345.h"
#include <Wire.h>

namespace
{
	const u8 I2C_ADDRESS          = 0x1D; // alternate = 0x53;

	const u8 REGISTER_POWER_CTL   = 0x2D;
	const u8 REGISTER_DATA_FORMAT = 0x31;
	const u8 REGISTER_DATAX0      = 0x32;
	const u8 REGISTER_DATAX1      = 0x33;
	const u8 REGISTER_DATAY0      = 0x34;
	const u8 REGISTER_DATAY1      = 0x35;
	const u8 REGISTER_DATAZ0      = 0x36;
	const u8 REGISTER_DATAZ1      = 0x37;
}

ADXL345::ADXL345() :
	m_FullResolution(false),
	m_Range(0),
	m_OutputRaw((OutputRaw){0, 0, 0})
{
}

void ADXL345::setup()
{
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(REGISTER_POWER_CTL);
	Wire.write(0x08);
	Wire.endTransmission();

	SetDataFormat(true, 3);
}

void ADXL345::loop()
{
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(REGISTER_DATAX0);
	Wire.endTransmission();

	Wire.requestFrom(I2C_ADDRESS, (u8)sizeof(m_OutputRaw));
	m_OutputRaw.x = WireReceiveLittleEndian<s16>();
	m_OutputRaw.y = WireReceiveLittleEndian<s16>();
	m_OutputRaw.z = WireReceiveLittleEndian<s16>();
}

void ADXL345::SetDataFormat(bool fullResolution, u8 range)
{
	m_FullResolution = fullResolution;
	m_Range = (range & 0x03);
	u8 value = (m_FullResolution ? 0x08 : 0x00) | m_Range;

	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(REGISTER_DATA_FORMAT);
	Wire.write(value);
	Wire.endTransmission();
}

ADXL345::OutputRaw ADXL345::GetOutputRaw() const
{
	return m_OutputRaw;
}

vec3 ADXL345::GetOutput() const
{
	f32 lsbsPerG = 256.0f;
	if (!m_FullResolution)
		lsbsPerG = (f32)(256 >> m_Range);

	// convert to m/s^2
	return vec3(m_OutputRaw.x, m_OutputRaw.y, m_OutputRaw.z) / lsbsPerG * 9.8f;
}
