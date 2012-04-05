#include "Sinewave.h"
#include <TimerOne.h>

Sinewave* g_pSinewave = NULL;

Sinewave::Sinewave(volatile uint8_t* pOutput, uint16_t outputResolution, uint8_t outputMask) :
	m_pOutput(pOutput),
	m_OutputMask(outputMask),
	m_BaudCallback(NULL),
	m_pBaudCallbackContext(NULL),
	m_BaudCallbackPeriod(-1),
	m_TimeSinceLastBaudCallback(0),
	m_Sample(0),
	m_SamplingPeriod(0)
{
	createSinTable(outputResolution);
}

void Sinewave::setFrequency(uint32_t frequency)
{
	setSamplingPeriod(computeSamplingPeriod(frequency));
}

void Sinewave::setSamplingPeriod(uint32_t samplingPeriod)
{
	m_SamplingPeriod = samplingPeriod;
	
	if (g_pSinewave == this)
	{
		Timer1.setPeriod(m_SamplingPeriod);
	}
}

uint32_t Sinewave::getSamplingPeriod() const
{
	return m_SamplingPeriod;
}

uint32_t Sinewave::computeSamplingPeriod(uint32_t sinewaveFrequency)
{
	return 1000000 / (sinewaveFrequency * SAMPLES_PER_WAVELENGTH);
}

void Sinewave::start()
{
	if (m_SamplingPeriod != 0)
	{
		g_pSinewave = this;
		Timer1.initialize(m_SamplingPeriod);
		Timer1.attachInterrupt(staticCallback);
	}
}

void Sinewave::stop()
{
	if (g_pSinewave == this)
	{
		Timer1.detachInterrupt();
		set(0);
		
		g_pSinewave = NULL;
		m_Sample = 0;
		m_TimeSinceLastBaudCallback = 0;
	}
}

void Sinewave::setBaudCallback(BaudCallback baudCallback, void* pContext, uint32_t baud)
{
	m_BaudCallback = baudCallback;
	m_pBaudCallbackContext = pContext;
	m_BaudCallbackPeriod = 1000000 / baud;
}

void Sinewave::clearBaudCallback()
{
	m_BaudCallback = NULL;
	m_pBaudCallbackContext = NULL;
	m_BaudCallbackPeriod = -1;
}

void Sinewave::createSinTable(uint16_t outputResolution)
{
	for (uint32_t i=0; i<SAMPLES_PER_WAVELENGTH; ++i)
	{
		m_SinTable[i] = uint8_t(LerpClamp(sin(2 * PI * i / SAMPLES_PER_WAVELENGTH), -1.0f, 1.0f, 0, outputResolution - 1) + 0.5f);
	}
}

void Sinewave::set(uint8_t value)
{
	// PORTB = (PORTB & 0xF0) | (value & 0x0F);
	// OCR2A = value;
	*m_pOutput = (*m_pOutput & ~m_OutputMask) | (value & m_OutputMask);
}

void Sinewave::callback()
{
	set(m_SinTable[m_Sample]);
	m_Sample = (m_Sample + 1) % SAMPLES_PER_WAVELENGTH;
	
	m_TimeSinceLastBaudCallback += m_SamplingPeriod;
	if (m_TimeSinceLastBaudCallback >= m_BaudCallbackPeriod)
	{
		m_TimeSinceLastBaudCallback -= m_BaudCallbackPeriod;
		if (m_BaudCallback)
		{
			m_BaudCallback(m_pBaudCallbackContext, this);
		}
	}
}

void Sinewave::staticCallback()
{
	g_pSinewave->callback();
}
