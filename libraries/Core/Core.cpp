#include "Core.h"

float sign(float x)
{
	return (x >= 0.0f ? 1.0f : -1.0f);
}

float LerpClamp(float in, float in0, float in1, float out0, float out1)
{
	if (in <= in0)
		return out0;
	if (in >= in1)
		return out1;
	return out0 + (in - in0) / (in1 - in0) * (out1 - out0);
}

float LerpClamp(float in, float in0, float in1, float in2, float out0, float out1, float out2)
{
	if (in <= in0)
		return out0;
	if (in >= in2)
		return out2;
	if (in <= in1)
		return out0 + (in - in0) / (in1 - in0) * (out1 - out0);
	return out1 + (in - in1) / (in2 - in1) * (out2 - out1);
}

float ModInto(float in, float min, float max)
{
	float diff = max - min;
	while (in > max) in -= diff;
	while (in < min) in += diff;
	return in;
}

void WireSendBigEndian(const uint8_t* buffer, uint8_t size)
{
	for (uint8_t i=0; i<size; ++i)
		Wire.write(buffer[size - i - 1]);
}

void WireSendLittleEndian(const uint8_t* buffer, uint8_t size)
{
	Wire.write(const_cast<uint8_t*>(buffer), size);
}

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

uintptr_t GetFreeMemory()
{
	uintptr_t free_memory;

	if (__brkval == 0)
		free_memory = ((uintptr_t)&free_memory) - ((uintptr_t)&__bss_end);
	else
		free_memory = ((uintptr_t)&free_memory) - ((uintptr_t)__brkval);

	return free_memory;
}

