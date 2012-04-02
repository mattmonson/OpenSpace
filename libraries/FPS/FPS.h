#ifndef _FPS_H
#define _FPS_H

#include <Core.h>

class FPS
{
public:
	FPS(u32 targetFrameTime = 0) :
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
		u32 now = millis();
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

	u32 GetFramerate() const
	{
		return m_Framerate;
	}

protected:
	u32 m_TargetFrameTime;
	u32 m_FrameCounter;
	u32 m_LastFrameTime;
	u32 m_Framerate;
};

#endif
