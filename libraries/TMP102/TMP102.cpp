#include "TMP102.h"
#include <Wire.h>

namespace
{
	const uint8_t I2C_ADDRESS_GND = 0x48; // ADD0 = Ground
	const uint8_t I2C_ADDRESS_V   = 0x49; // ADD0 = V+
	const uint8_t I2C_ADDRESS_SDA = 0x50; // ADD0 = SDA
	const uint8_t I2C_ADDRESS_SCL = 0x51; // ADD0 = SCL
	
	const uint8_t REGISTER_TEMP 		= 0x00;
	const uint8_t REGISTER_CONFIG	= 0x01;
	const uint8_t REGISTER_TLOW		= 0x02;
	const uint8_t REGISTER_THIGH		= 0x03;
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
		Reserved(0),
		ExtendedMode(0),
		Alert(1),
		ConversionRate(TMP102::EConversionRate::Hz4),
		ShutdownMode(0),
		Thermostat(0),
		Polarity(0),
		FaultQueue(0),
		ConverterResolution(3),
		OneShotConversionReady(0)
	{
	}
	 
	uint16_t Reserved : 4;
	uint16_t ExtendedMode : 1;
	uint16_t Alert : 1;
	uint16_t ConversionRate : 2;
	uint16_t ShutdownMode : 1;
	uint16_t Thermostat : 1;
	uint16_t Polarity : 1;
	uint16_t FaultQueue : 2;
	uint16_t ConverterResolution : 2;
	uint16_t OneShotConversionReady : 1;
};

void TMP102::setup(bool extendedMode, EConversionRate::Enum conversionRate)
{
	ConfigRegister config;
	config.ExtendedMode = extendedMode;
	config.ConversionRate = conversionRate;

	Wire.beginTransmission(m_Address);
	Wire.write(REGISTER_CONFIG);
	WireSendBigEndian((uint8_t*)&config, sizeof(config));
	Wire.endTransmission();
	
	Wire.beginTransmission(m_Address);
	Wire.write(REGISTER_TEMP);
	Wire.endTransmission();
}

void TMP102::loop()
{
	Wire.requestFrom(m_Address, (uint8_t)2);
	int16_t reg = WireReceiveBigEndian<int16_t>();
	
	if (reg & 0x0001)
		m_RawTemp = reg >> 3; // Extended Mode, 13 bits of temp
	else
		m_RawTemp = reg >> 4; // Normal Mode, 12 bits of temp
}

int16_t TMP102::GetRawTemp() const
{
	return m_RawTemp;
}

float TMP102::GetTemp() const
{
	return m_RawTemp * 0.0625f;
}
