#ifndef _ADXL345_H
#define _ADXL345_H

#include <Core.h>
#include <VectorMath.h>

class ADXL345
{
public:
	struct OutputRaw
	{
		int16_t x, y, z;
	};

public:
	ADXL345();
	void setup();
	void loop();
	
	void SetDataFormat(bool fullResolution, uint8_t range); // range: -/+2^(n+1)g

	OutputRaw GetOutputRaw() const;
	vec3 GetOutput() const;				// in m/s^2

private:
	bool m_FullResolution;
	uint8_t m_Range;
	
	OutputRaw m_OutputRaw;
};

#endif
