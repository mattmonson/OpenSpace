#ifndef _CORE_H
#define _CORE_H

#include <Arduino.h>
#include <Wire.h>

#define METERS_PER_SECOND_TO_MILES_PER_HOUR(x) ((x) * 2.23693629f)
#define METERS_PER_SECOND_TO_KNOTS(x) ((x) * 1.94384449f)
#define METERS_TO_FEET(x) ((x) * 3.2808399f)

#define _countof(x) (sizeof(x) / sizeof((x)[0]))

float sign(float x);
float LerpClamp(float in, float in0, float in1, float out0, float out1);
float LerpClamp(float in, float in0, float in1, float in2, float out0, float out1, float out2);
float ModInto(float in, float min, float max);
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
	for (uint8_t i=0; i<sizeof(T); ++i)
		reinterpret_cast<uint8_t*>(&t)[sizeof(T) - i - 1] = Wire.read();
	return t;
}

template <typename T>
T WireReceiveLittleEndian()
{
	T t;
	for (uint8_t i=0; i<sizeof(T); ++i)
		reinterpret_cast<uint8_t*>(&t)[i] = Wire.read();
	return t;
}

void WireSendBigEndian(const uint8_t* buffer, uint8_t size);
void WireSendLittleEndian(const uint8_t* buffer, uint8_t size);

template <typename T>
T pow2(const T& t)
{
	return t * t;
}

template <typename T>
T LowPassFilter(const T& input, const T& filtered, float dt, float timeConstant)
{
	float alpha = (dt > 0.0f ? dt / (dt + timeConstant) : 0.0f);
	//return (1.0f - alpha) * filtered + alpha * input;
	return filtered + alpha * (input - filtered);
}

template <typename T>
T LowPassFilter(const T& input, const T& filtered, float alpha)
{
	//return (1.0f - alpha) * filtered + alpha * input;
	return filtered + alpha * (input - filtered);
}

template <typename T>
T HighPassFilter(const T& prevInput, const T& input, const T& filtered, float dt, float timeConstant)
{
	float alpha = timeConstant / (timeConstant + dt);
	//return alpha * filtered + alpha * (input - prevInput);
	return alpha * (filtered + input - prevInput);
}

template <typename T>
T PID(T& prevError, T& integral, const T& target, const T& current, float dt, float Kp, float Ki, float Kd, float expectedEffectiveness=1.0f)
{
	const T error = target - current;
	integral += error * dt * expectedEffectiveness;
	const T derivative = (error - prevError) / dt;
	prevError = error;
	return Kp * error + Ki * integral + Kd * derivative;
}

template <typename TSerial>
void serprintf(TSerial& serial, const char *fmt, ...)
{
	char buffer[128];
	va_list args;
	
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end (args);

	serial.print(buffer);
}

template <uint32_t BUFFER_SIZE>
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
	
	uint32_t size() const
	{
		return m_Size;
	}
	
	void push_back(uint8_t bit)
	{
		uint8_t& byte = m_Buffer[m_Size / 8];
		if (bit)
			byte |= (1 << (m_Size % 8));
		else
			byte &= ~(1 << (m_Size % 8));
		++m_Size;
	}
	
	uint8_t operator [] (uint32_t bitIndex) const
	{
		return (m_Buffer[bitIndex / 8] >> (bitIndex % 8)) & 0x01;
	}
	
	void print() const
	{
		for (uint32_t i=0; i<m_Size; ++i)
		{
			serprintf(Serial, "%hu", (uint8_t)((m_Buffer[i / 8] >> (7 - i % 8)) & 0x1));
			// serprintf(Serial, "%hu", (*this)[i]);

			if (i % 8 == 7 || i + 1 == m_Size)
				serprintf(Serial, " 0x%.2hX '%c'\n", m_Buffer[i / 8], m_Buffer[i / 8] ? m_Buffer[i / 8] : ' ');
		}
	}
	
private:
	uint8_t m_Buffer[BUFFER_SIZE];
	uint32_t m_Size;
};

#endif
