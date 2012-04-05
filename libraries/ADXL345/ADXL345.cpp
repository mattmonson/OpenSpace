#include "ADXL345.h"
#include <Wire.h>

namespace
{
	const uint8_t I2C_ADDRESS          = 0x1D; // alternate = 0x53;

	const uint8_t REGISTER_POWER_CTL   = 0x2D;
	const uint8_t REGISTER_DATA_FORMAT = 0x31;
	const uint8_t REGISTER_DATAX0      = 0x32;
	const uint8_t REGISTER_DATAX1      = 0x33;
	const uint8_t REGISTER_DATAY0      = 0x34;
	const uint8_t REGISTER_DATAY1      = 0x35;
	const uint8_t REGISTER_DATAZ0      = 0x36;
	const uint8_t REGISTER_DATAZ1      = 0x37;
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

	Wire.requestFrom(I2C_ADDRESS, (uint8_t)sizeof(m_OutputRaw));
	m_OutputRaw.x = WireReceiveLittleEndian<int16_t>();
	m_OutputRaw.y = WireReceiveLittleEndian<int16_t>();
	m_OutputRaw.z = WireReceiveLittleEndian<int16_t>();
}

void ADXL345::SetDataFormat(bool fullResolution, uint8_t range)
{
	m_FullResolution = fullResolution;
	m_Range = (range & 0x03);
	uint8_t value = (m_FullResolution ? 0x08 : 0x00) | m_Range;

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
	float lsbsPerG = 256.0f;
	if (!m_FullResolution)
		lsbsPerG = (float)(256 >> m_Range);

	// convert to m/s^2
	return vec3(m_OutputRaw.x, m_OutputRaw.y, m_OutputRaw.z) / lsbsPerG * 9.8f;
}
