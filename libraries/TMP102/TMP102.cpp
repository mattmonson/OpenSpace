#include "TMP102.h"
#include <Wire.h>

namespace
{
	const u8 I2C_ADDRESS_GND = 0x48; // ADD0 = Ground
	const u8 I2C_ADDRESS_V   = 0x49; // ADD0 = V+
	const u8 I2C_ADDRESS_SDA = 0x50; // ADD0 = SDA
	const u8 I2C_ADDRESS_SCL = 0x51; // ADD0 = SCL
	
	const u8 REGISTER_TEMP 		= 0x00;
	const u8 REGISTER_CONFIG	= 0x01;
	const u8 REGISTER_TLOW		= 0x02;
	const u8 REGISTER_THIGH		= 0x03;
}

TMP102::TMP102(EAddress::Enum addr) :
	m_RawTemp(0)
{
	switch(addr)
	{
	case EAddress::GND: 	m_Address = I2C_ADDRESS_GND; 	break;
	case EAddress::V:   	m_Address = I2C_ADDRESS_V;   	break;
	case EAddress::SDA: 	m_Address = I2C_ADDRESS_SDA; 	break;
	case EAddress::SCL: 	m_Address = I2C_ADDRESS_SCL; 	break;
	default:            	m_Address = I2C_ADDRESS_GND; 	break;
	}
}

struct ConfigRegister
{
	ConfigRegister() : 
		OneShotConversionReady(0),
		ConverterResolution(3),
		FaultQueue(0),
		Polarity(0),
		Thermostat(0),
		ShutdownMode(0),
		ConversionRate(TMP102::EConversionRate::Hz4),
		Alert(1),
		ExtendedMode(0),
		Reserved(0)
	{
	}
	 
	u16 Reserved : 4;
	u16 ExtendedMode : 1;
	u16 Alert : 1;
	u16 ConversionRate : 2;
	u16 ShutdownMode : 1;
	u16 Thermostat : 1;
	u16 Polarity : 1;
	u16 FaultQueue : 2;
	u16 ConverterResolution : 2;
	u16 OneShotConversionReady : 1;
};

void TMP102::setup(bool extendedMode, EConversionRate::Enum conversionRate)
{
	ConfigRegister config;
	config.ExtendedMode = extendedMode;
	config.ConversionRate = conversionRate;

	Wire.beginTransmission(m_Address);
	Wire.send(REGISTER_CONFIG);
	WireSendBigEndian((u8*)&config, sizeof(config));
	Wire.endTransmission();
	
	Wire.beginTransmission(m_Address);
	Wire.send(REGISTER_TEMP);
	Wire.endTransmission();
}

void TMP102::loop()
{
	Wire.requestFrom(m_Address, (u8)2);
	s16 reg = WireReceiveBigEndian<s16>();
	
	if (reg & 0x0001)
		m_RawTemp = reg >> 3; // Extended Mode, 13 bits of temp
	else
		m_RawTemp = reg >> 4; // Normal Mode, 12 bits of temp
}

s16 TMP102::GetRawTemp() const
{
	return m_RawTemp;
}

f32 TMP102::GetTemp() const
{
	return m_RawTemp * 0.0625f;
}
