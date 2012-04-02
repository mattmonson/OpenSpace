#ifndef _ADXL345_H
#define _ADXL345_H

#include <Core.h>
#include <VectorMath.h>

class ADXL345
{
public:
	struct OutputRaw
	{
		s16 x, y, z;
	};

public:
	ADXL345();
	void setup();
	void loop();
	
	void SetDataFormat(bool fullResolution, u8 range); // range: -/+2^(n+1)g

	OutputRaw GetOutputRaw() const;
	vec3 GetOutput() const;				// in m/s^2

private:
	bool m_FullResolution;
	u8 m_Range;
	
	OutputRaw m_OutputRaw;
};

#endif
