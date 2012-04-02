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
	
	s16 GetRawTemp() const; // in LSB
	f32 GetTemp() const; // in degrees C

private:
	u8 m_Address;
	s16 m_RawTemp;
};

#endif