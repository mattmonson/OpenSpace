#ifndef _CORE_H
#define _CORE_H

#include <WProgram.h>
#include <Wire.h>

#define u8 uint8_t
#define s8 char
#define u16 unsigned int
#define s16 int
#define u32 unsigned long
#define s32 long
#define f32 float

#define METERS_PER_SECOND_TO_MILES_PER_HOUR(x) ((x) * 2.23693629f)
#define METERS_PER_SECOND_TO_KNOTS(x) ((x) * 1.94384449f)
#define METERS_TO_FEET(x) ((x) * 3.2808399f)

#define _countof(x) (sizeof(x) / sizeof((x)[0]))

f32 sign(f32 x);
f32 LerpClamp(f32 in, f32 in0, f32 in1, f32 out0, f32 out1);
f32 LerpClamp(f32 in, f32 in0, f32 in1, f32 in2, f32 out0, f32 out1, f32 out2);
f32 ModInto(f32 in, f32 min, f32 max);
uintptr_t GetFreeMemory();

template <typename T>
T ToRadians(const T& degrees)
{
	return degrees * 0.0174532925f;
}

template <typename T>
T ToDegrees(const T& radians)
{
	return radians * 57.2957795f;
}

template <typename T>
T Clamp(const T& in, const T& min, const T& max)
{
	if (in < min)
		return min;
	if (in > max)
		return max;
	return in;
}

template <typename T>
T ClampBetween(const T& in, const T& extent0, const T& extent1)
{
	if (extent0 <= extent1)
		return Clamp(in, extent0, extent1);
	return Clamp(in, extent1, extent0);
}

template <typename T>
T WireReceiveBigEndian()
{
	T t;
	for (u8 i=0; i<sizeof(T); ++i)
		reinterpret_cast<u8*>(&t)[sizeof(T) - i - 1] = Wire.receive();
	return t;
}

template <typename T>
T WireReceiveLittleEndian()
{
	T t;
	for (u8 i=0; i<sizeof(T); ++i)
		reinterpret_cast<u8*>(&t)[i] = Wire.receive();
	return t;
}

void WireSendBigEndian(const u8* buffer, u8 size);
void WireSendLittleEndian(const u8* buffer, u8 size);

template <typename T>
T pow2(const T& t)
{
	return t * t;
}

template <typename T>
T LowPassFilter(const T& input, const T& filtered, f32 dt, f32 timeConstant)
{
	f32 alpha = (dt > 0.0f ? dt / (dt + timeConstant) : 0.0f);
	//return (1.0f - alpha) * filtered + alpha * input;
	return filtered + alpha * (input - filtered);
}

template <typename T>
T LowPassFilter(const T& input, const T& filtered, f32 alpha)
{
	//return (1.0f - alpha) * filtered + alpha * input;
	return filtered + alpha * (input - filtered);
}

template <typename T>
T HighPassFilter(const T& prevInput, const T& input, const T& filtered, f32 dt, f32 timeConstant)
{
	f32 alpha = timeConstant / (timeConstant + dt);
	//return alpha * filtered + alpha * (input - prevInput);
	return alpha * (filtered + input - prevInput);
}

template <typename T>
T PID(T& prevError, T& integral, const T& target, const T& current, f32 dt, f32 Kp, f32 Ki, f32 Kd, f32 expectedEffectiveness=1.0f)
{
	const T error = target - current;
	integral += error * dt * expectedEffectiveness;
	const T derivative = (error - prevError) / dt;
	prevError = error;
	return Kp * error + Ki * integral + Kd * derivative;
}

template <typename TSerial>
void serprintf(TSerial& serial, char *fmt, ...)
{
	char buffer[128];
	va_list args;
	
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end (args);

	serial.print(buffer);
}

template <u32 BUFFER_SIZE>
struct BitStream
{
public:
	BitStream() :
		m_Size(0)
	{
	}
	
	void clear()
	{
		m_Size = 0;
	}
	
	u32 size() const
	{
		return m_Size;
	}
	
	void push_back(u8 bit)
	{
		u8& byte = m_Buffer[m_Size / 8];
		if (bit)
			byte |= (1 << (m_Size % 8));
		else
			byte &= ~(1 << (m_Size % 8));
		++m_Size;
	}
	
	u8 operator [] (u32 bitIndex) const
	{
		return (m_Buffer[bitIndex / 8] >> (bitIndex % 8)) & 0x01;
	}
	
	void print() const
	{
		for (u32 i=0; i<m_Size; ++i)
		{
			serprintf(Serial, "%hu", (u8)((m_Buffer[i / 8] >> (7 - i % 8)) & 0x1));
			// serprintf(Serial, "%hu", (*this)[i]);

			if (i % 8 == 7 || i + 1 == m_Size)
				serprintf(Serial, " 0x%.2hX '%c'\n", m_Buffer[i / 8], m_Buffer[i / 8] ? m_Buffer[i / 8] : ' ');
		}
	}
	
private:
	u8 m_Buffer[BUFFER_SIZE];
	u32 m_Size;
};

#endif
