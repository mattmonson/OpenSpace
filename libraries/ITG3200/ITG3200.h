#ifndef _ITG3200_H
#define _ITG3200_H

#include <Core.h>
#include <VectorMath.h>

class ITG3200
{
public:
	struct EFullScale { enum Enum { Reserved0, Reserved1, Reserved2, PlusMinus2000 }; };
	struct ELowPassFilterConfig { enum Enum { Filter256Hz_Sample8kHz, Filter188Hz_Sample1kHz, Filter98Hz_Sample1kHz, Filter42Hz_Sample1kHz, Filter20Hz_Sample1kHz, Filter10Hz_Sample1kHz, Filter5Hz_Sample1kHz, Reserved }; };

	struct OutputRaw
	{
		int16_t temp;
		int16_t x, y, z;
	};

public:
	ITG3200();
	void setup();
	void loop();

	void Prime();
	void UpdateBias(float dt);
	
	void SetSampleRateDivisor(uint8_t divisor);  // sample rate is F_internal / (divisor + 1), where F_internal is either 1 or 8 kHz
	void SetLowPassFilterConfig(ELowPassFilterConfig::Enum lpfConfig);

	OutputRaw GetOutputRaw() const;
	float GetTemp() const;                    // in C
	vec3 GetBiasedAngVel() const;           // in deg/S
	vec3 GetAngVel() const;                 // in deg/S

private:
	OutputRaw m_OutputRaw;
	vec3 m_Bias;
};

#endif
