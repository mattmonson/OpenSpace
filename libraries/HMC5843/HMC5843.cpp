#include "HMC5843.h"
#include <Wire.h>

namespace
{
	const u8 I2C_ADDRESS = 0x1E;

	const u8 REGISTER_CONFIG_A  = 0x00;
	const u8 REGISTER_CONFIG_B  = 0x01;
	const u8 REGISTER_MODE      = 0x02;

	const u16 RANGE_COUNTS_PER_GAUSS[] = { 1620, 1300, 970, 780, 530, 460, 390, 280 };
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

	u8 configA = (outputRate << 2) | bias;
	u8 configB = (m_Range << 5);
	u8 mode = conversionMode;

	Wire.beginTransmission(I2C_ADDRESS);
	Wire.send(0x00);
	Wire.send(configA);
	Wire.send(configB);
	Wire.send(mode);
	Wire.endTransmission();
}

void HMC5843::loop()
{
	Wire.requestFrom(I2C_ADDRESS, (u8)sizeof(m_OutputRaw));
	m_OutputRaw.x = WireReceiveBigEndian<s16>();
	m_OutputRaw.y = WireReceiveBigEndian<s16>();
	m_OutputRaw.z = WireReceiveBigEndian<s16>();
	m_OutputRaw.status = WireReceiveBigEndian<u8>();
}

HMC5843::OutputRaw HMC5843::GetOutputRaw() const
{
	return m_OutputRaw;
}

vec3 HMC5843::GetOutput() const
{
	// convert to real units
	return vec3(m_OutputRaw.x, m_OutputRaw.y, m_OutputRaw.z) / (f32)RANGE_COUNTS_PER_GAUSS[m_Range];
}
