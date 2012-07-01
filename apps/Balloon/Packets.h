#pragma once

#include <Core.h>

struct EPacketType
{
	enum Enum
	{
		None,
		Ping,
		Pong,
		Telemetry
	};
};

struct PingPacket
{
	PingPacket() : packetType(EPacketType::Ping) {}
	
	uint8_t packetType;
	uint32_t time;
};

struct PongPacket
{
	PongPacket() : packetType(EPacketType::Pong) {}
	
	uint8_t packetType;
	uint32_t time;
};

struct TelemetryPacket
{
	TelemetryPacket() : packetType(EPacketType::Telemetry) {}

	uint8_t packetType;
	uint16_t time;               // in s

	float gpsLat, gpsLon;        // in degrees
	int32_t gpsAlt : 18;         // in m
	int32_t gpsCourse : 10;      // in degrees
	uint32_t gpsSpeed : 8;       // in m/s
	uint32_t bmpPressure : 24;   // in Pa
	
	int8_t tmpInternal;          // in deg C
	int8_t tmpExternal;          // in deg C

	uint16_t batteryVoltage;     // in mV
};

