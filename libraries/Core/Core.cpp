#include "Core.h"

f32 sign(f32 x)
{
	return (x >= 0.0f ? 1.0f : -1.0f);
}

f32 LerpClamp(f32 in, f32 in0, f32 in1, f32 out0, f32 out1)
{
	if (in <= in0)
		return out0;
	if (in >= in1)
		return out1;
	return out0 + (in - in0) / (in1 - in0) * (out1 - out0);
}

f32 LerpClamp(f32 in, f32 in0, f32 in1, f32 in2, f32 out0, f32 out1, f32 out2)
{
	if (in <= in0)
		return out0;
	if (in >= in2)
		return out2;
	if (in <= in1)
		return out0 + (in - in0) / (in1 - in0) * (out1 - out0);
	return out1 + (in - in1) / (in2 - in1) * (out2 - out1);
}

f32 ModInto(f32 in, f32 min, f32 max)
{
	f32 diff = max - min;
	while (in > max) in -= diff;
	while (in < min) in += diff;
	return in;
}

void WireSendBigEndian(const u8* buffer, u8 size)
{
	for (u8 i=0; i<size; ++i)
		Wire.write(buffer[size - i - 1]);
}

void WireSendLittleEndian(const u8* buffer, u8 size)
{
	Wire.write(const_cast<u8*>(buffer), size);
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

