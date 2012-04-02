#ifndef _BMP085_H
#define _BMP085_H

#include <Core.h>

class BMP085
{
public:
	BMP085();
	void setup();
	void loop();
	void loopAsync();

	void SetOversamplingSetting(u8 oss);
	void SetReferencePressure(s32 referencePressureInPa);
	s32 GetReferencePressureInPa() const;

	s32 GetTempInDeciC() const;  // in deci-degrees-C
	f32 GetTempInC() const;      // in degrees C
	s32 GetPressureInPa() const; // Pa
	f32 GetAltitudeInM() const;  // meters

private:
	void RequestTemp();
	s32 GetRawTemp();
	void RequestPressure();
	s32 GetRawPressure();
	void ProcessRawReadings();

	// Calibration coefficients -- read from EEPROM
	s16 m_AC1, m_AC2, m_AC3;
	u16 m_AC4, m_AC5, m_AC6;
	s16 m_B1, m_B2;
	s16 m_MB, m_MC, m_MD;
	
	// Raw readings for temp & pressure
	s32 m_UT, m_UP;
	
	// Non-blocking update state
	struct EState { enum Enum { Start, WaitForTemp, WaitForPressure }; };
	EState::Enum m_State;
	u32 m_StateStart;

	// Configuration
	u8 m_OSS;
	s32 m_ReferencePressureInPa;

	// Data
	s32 m_TempInDeciC;
	s32 m_PressureInPa;
};

#endif
