#include "HMC5843.h"
#include <Wire.h>

namespace
{
	const uint8_t I2C_ADDRESS = 0x1E;

	const uint8_t REGISTER_CONFIG_A  = 0x00;
	const uint8_t REGISTER_CONFIG_B  = 0x01;
	const uint8_t REGISTER_MODE      = 0x02;

	const uint16_t RANGE_COUNTS_PER_GAUSS[] = { 1620, 1300, 970, 780, 530, 460, 390, 280 };
}

HMC5843::HMC5843() :
	m_Range(ERange::PlusMinus1_0Ga),
	m_OutputRaw((OutputRaw){0, 0, 0, 0})
{
}

void HMC5843::setup(EOutputRate::Enum outputRate, EBias::Enum bias, EConversionMode::Enum conversionMode, ERange::Enum range)
{
	outputRate =     (EOutputRate::Enum)    Clamp((int)outputRate,     0, EOutputRate::EnumCount-1);
	bias =           (EBias::Enum)          Clamp((int)bias,           0, EBias::EnumCount-1);
	conversionMode = (EConversionMode::Enum)Clamp((int)conversionMode, 0, EConversionMode::EnumCount-1);
	m_Range =        (ERange::Enum)         Clamp((int)range,          0, ERange::EnumCount-1);

	uint8_t configA = (outputRate << 2) | bias;
	uint8_t configB = (m_Range << 5);
	uint8_t mode = conversionMode;

	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write((uint8_t)0x00);
	Wire.write(configA);
	Wire.write(configB);
	Wire.write(mode);
	Wire.endTransmission();
}

void HMC5843::loop()
{
	Wire.requestFrom(I2C_ADDRESS, (uint8_t)sizeof(m_OutputRaw));
	m_OutputRaw.x = WireReceiveBigEndian<int16_t>();
	m_OutputRaw.y = WireReceiveBigEndian<int16_t>();
	m_OutputRaw.z = WireReceiveBigEndian<int16_t>();
	m_OutputRaw.status = WireReceiveBigEndian<uint8_t>();
}

HMC5843::OutputRaw HMC5843::GetOutputRaw() const
{
	return m_OutputRaw;
}

vec3 HMC5843::GetOutput() const
{
	// convert to real units
	return vec3(m_OutputRaw.x, m_OutputRaw.y, m_OutputRaw.z) / (float)RANGE_COUNTS_PER_GAUSS[m_Range];
}
