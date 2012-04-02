#ifndef _HMC5843_H
#define _HMC5843_H

#include <Core.h>
#include <VectorMath.h>

class HMC5843
{
public:
	struct EOutputRate { enum Enum { HalfHz, OneHz, TwoHz, FiveHz, TenHz, TwentyHz, FiftyHz, EnumCount }; };
	struct EBias { enum Enum { None, Positive, Negative, EnumCount }; };
	struct EConversionMode { enum Enum { Continuous, Single, Idle, Sleep, EnumCount }; };
	struct ERange { enum Enum { PlusMinus0_7Ga, PlusMinus1_0Ga, PlusMinus1_5Ga, PlusMinus2_0Ga, PlusMinus3_2Ga, PlusMinus3_8Ga, PlusMinus4_5Ga, PlusMinus6_5Ga, EnumCount }; };

	struct OutputRaw
	{
		s16 x, y, z;
		u8 status;
	};

public:
	HMC5843();
	void setup
	(
		EOutputRate::Enum outputRate = EOutputRate::TenHz,
		EBias::Enum bias = EBias::None,
		EConversionMode::Enum conversionMode = EConversionMode::Continuous,
		ERange::Enum range = ERange::PlusMinus1_0Ga
	);
	void loop();

	OutputRaw GetOutputRaw() const;
	vec3 GetOutput() const;

private:
	ERange::Enum m_Range;
	OutputRaw m_OutputRaw;
};

#endif
