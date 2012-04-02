#ifndef _SINEWAVE_H
#define _SINEWAVE_H

#include <Core.h>
#include <TimerOne.h>

class Sinewave
{
public:
	Sinewave(volatile u8* pOutput, u16 outputResolution, u8 outputMask);

	void setFrequency(u32 frequency);
	void setSamplingPeriod(u32 samplingPeriod);
	u32 getSamplingPeriod() const;
	static u32 computeSamplingPeriod(u32 sinewaveFrequency);
	
	void start();
	void stop();
	
	typedef void (*BaudCallback)(void*, Sinewave*);
	void setBaudCallback(BaudCallback baudCallback, void* pContext, u32 baud);
	void clearBaudCallback();

private:
	void createSinTable(u16 outputResolution);
	void set(u8 value);
	void callback();
	static void staticCallback();
	
private:
	static const u8 SAMPLES_PER_WAVELENGTH = 32;
	u8 m_SinTable[SAMPLES_PER_WAVELENGTH];
	
	volatile u8* m_pOutput;				// where are we outputting to? Ex: PORTB
	u8 m_OutputMask;					// mask for writing to pOutput, Ex: 0x0F to write to digital pins [8..11] on PORTB
	
	BaudCallback m_BaudCallback;
	void* m_pBaudCallbackContext;
	u32 m_BaudCallbackPeriod;			// how often (in microseconds) should the BaudCallback be called
	u32 m_TimeSinceLastBaudCallback;	// in microseconds
	
	u8 m_Sample;						// the current sample index, [0..SAMPLES_PER_WAVELENGTH)
	u32 m_SamplingPeriod;				// in microseconds
};

#endif