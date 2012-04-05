#ifndef _TMP102_H
#define _TMP102_H

#include <Core.h>

class TMP102
{
public:
	struct EAddress { enum Enum { GND, V, SDA, SCL }; };
	struct EConversionRate { enum Enum { Hz0_25, Hz1, Hz4, Hz8 }; };

	TMP102(EAddress::Enum addr = EAddress::GND);
	void setup(bool extendedMode = false, EConversionRate::Enum conversionRate = EConversionRate::Hz4);
	void loop();
	
	int16_t GetRawTemp() const; // in LSB
	float GetTemp() const; // in degrees C

private:
	uint8_t m_Address;
	int16_t m_RawTemp;
};

#endif
