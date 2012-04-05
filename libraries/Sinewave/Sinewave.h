#ifndef _SINEWAVE_H
#define _SINEWAVE_H

#include <Core.h>

class Sinewave
{
public:
	Sinewave(volatile uint8_t* pOutput, uint16_t outputResolution, uint8_t outputMask);

	void setFrequency(uint32_t frequency);
	void setSamplingPeriod(uint32_t samplingPeriod);
	uint32_t getSamplingPeriod() const;
	static uint32_t computeSamplingPeriod(uint32_t sinewaveFrequency);
	
	void start();
	void stop();
	
	typedef void (*BaudCallback)(void*, Sinewave*);
	void setBaudCallback(BaudCallback baudCallback, void* pContext, uint32_t baud);
	void clearBaudCallback();

private:
	void createSinTable(uint16_t outputResolution);
	void set(uint8_t value);
	void callback();
	static void staticCallback();
	
private:
	static const uint8_t SAMPLES_PER_WAVELENGTH = 32;
	uint8_t m_SinTable[SAMPLES_PER_WAVELENGTH];
	
	volatile uint8_t* m_pOutput;                            // where are we outputting to? Ex: PORTB
	uint8_t m_OutputMask;                                   // mask for writing to pOutput, Ex: 0x0F to write to digital pins [8..11] on PORTB
	
	BaudCallback m_BaudCallback;
	void* m_pBaudCallbackContext;
	uint32_t m_BaudCallbackPeriod;                          // how often (in microseconds) should the BaudCallback be called
	uint32_t m_TimeSinceLastBaudCallback;                   // in microseconds
	
	uint8_t m_Sample;                                       // the current sample index, [0..SAMPLES_PER_WAVELENGTH)
	uint32_t m_SamplingPeriod;                              // in microseconds
};

#endif
