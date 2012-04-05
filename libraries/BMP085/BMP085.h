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

	void SetOversamplingSetting(uint8_t oss);
	void SetReferencePressure(int32_t referencePressureInPa);
	int32_t GetReferencePressureInPa() const;

	int32_t GetTempInDeciC() const;  // in deci-degrees-C
	float GetTempInC() const;      // in degrees C
	int32_t GetPressureInPa() const; // Pa
	float GetAltitudeInM() const;  // meters

private:
	void RequestTemp();
	int32_t GetRawTemp();
	void RequestPressure();
	int32_t GetRawPressure();
	void ProcessRawReadings();

	// Calibration coefficients -- read from EEPROM
	int16_t m_AC1, m_AC2, m_AC3;
	uint16_t m_AC4, m_AC5, m_AC6;
	int16_t m_B1, m_B2;
	int16_t m_MB, m_MC, m_MD;
	
	// Raw readings for temp & pressure
	int32_t m_UT, m_UP;
	
	// Non-blocking update state
	struct EState { enum Enum { Start, WaitForTemp, WaitForPressure }; };
	EState::Enum m_State;
	uint32_t m_StateStart;

	// Configuration
	uint8_t m_OSS;
	int32_t m_ReferencePressureInPa;

	// Data
	int32_t m_TempInDeciC;
	int32_t m_PressureInPa;
};

#endif
