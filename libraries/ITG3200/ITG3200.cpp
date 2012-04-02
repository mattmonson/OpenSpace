#include "ITG3200.h"
#include <Wire.h>

namespace
{
	//const u8 I2C_ADDRESS          = 0x68; // normal
	const u8 I2C_ADDRESS          = 0x69; // alternat

	const u8 REGISTER_SMPLRT_DIV  = 0x15;
	const u8 REGISTER_DPLF_FS     = 0x16;

	const u8 REGISTER_TEMP1       = 0x1B;
	const u8 REGISTER_TEMP0       = 0x1C;
	const u8 REGISTER_XOUT1       = 0x1D;
	const u8 REGISTER_XOUT0       = 0x1E;
	const u8 REGISTER_YOUT1       = 0x1F;
	const u8 REGISTER_YOUT0       = 0x20;
	const u8 REGISTER_ZOUT1       = 0x21;
	const u8 REGISTER_ZOUT0       = 0x22;
}

ITG3200::ITG3200() :
	m_OutputRaw((OutputRaw){0, 0, 0, 0})
{
}

void ITG3200::setup()
{
	SetSampleRateDivisor(0);
	SetLowPassFilterConfig(ELowPassFilterConfig::Filter98Hz_Sample1kHz);
}

void ITG3200::loop()
{
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.send(REGISTER_TEMP1);
	Wire.endTransmission();

	Wire.requestFrom(I2C_ADDRESS, (u8)sizeof(m_OutputRaw));
	m_OutputRaw.temp = WireReceiveBigEndian<s16>();
	m_OutputRaw.x = WireReceiveBigEndian<s16>();
	m_OutputRaw.y = WireReceiveBigEndian<s16>();
	m_OutputRaw.z = WireReceiveBigEndian<s16>();
}

void ITG3200::Prime()
{
	vec3 biasAccum;
	
	const u32 count = 128;
	for (u32 i=0; i<count; ++i)
	{
		loop();
		biasAccum += GetBiasedAngVel();
		delay(10);
	}
	
	m_Bias = biasAccum / count;
}

void ITG3200::UpdateBias(f32 dt)
{
	m_Bias = LowPassFilter(GetBiasedAngVel(), m_Bias, dt, 30.0f);
}

void ITG3200::SetSampleRateDivisor(u8 divisor)
{
	// sample rate is F_internal / (divisor + 1), where F_internal is either 1 or 8 kHz
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.send(REGISTER_SMPLRT_DIV);
	Wire.send(divisor);
	Wire.endTransmission();
}

void ITG3200::SetLowPassFilterConfig(ELowPassFilterConfig::Enum lpfConfig)
{
	u8 value = (EFullScale::PlusMinus2000 << 3) | lpfConfig;

	Wire.beginTransmission(I2C_ADDRESS);
	Wire.send(REGISTER_DPLF_FS);
	Wire.send(value);
	Wire.endTransmission();
}

ITG3200::OutputRaw ITG3200::GetOutputRaw() const
{
	return m_OutputRaw;
}

f32 ITG3200::GetTemp() const
{
	return 35.0f + (m_OutputRaw.temp + 13200) / 280.0f; // 280 LSB/C, -13,200 LSB @ 35 C
}

vec3 ITG3200::GetBiasedAngVel() const
{
	return vec3(m_OutputRaw.x,m_OutputRaw.y, m_OutputRaw.z) / 14.375f; // 14.375 LSB/(deg/S)
}

vec3 ITG3200::GetAngVel() const
{
	return GetBiasedAngVel() - m_Bias;
}