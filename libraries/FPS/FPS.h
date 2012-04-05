#ifndef _FPS_H
#define _FPS_H

#include <Core.h>

class FPS
{
public:
	FPS(uint32_t targetFrameTime = 0) :
		m_TargetFrameTime(targetFrameTime),
		m_FrameCounter(0),
		m_LastFrameTime(millis()),
		m_Framerate(0)
	{
	}
	
	void increment()
	{
		++m_FrameCounter;
	}

	void loop()
	{
		// wait out the rest of the frame...
		uint32_t now = millis();
		if (now - m_LastFrameTime + 1 < m_TargetFrameTime)
			delay(m_TargetFrameTime - (now - m_LastFrameTime + 1));

		// update the frame counters      
		now = millis();
		if (now / 1000 != m_LastFrameTime / 1000)
		{
			m_Framerate = m_FrameCounter;
			m_FrameCounter = 0;
		}
		m_LastFrameTime = now;
	}

	uint32_t GetFramerate() const
	{
		return m_Framerate;
	}

protected:
	uint32_t m_TargetFrameTime;
	uint32_t m_FrameCounter;
	uint32_t m_LastFrameTime;
	uint32_t m_Framerate;
};

#endif
