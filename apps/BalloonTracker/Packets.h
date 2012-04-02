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
	
	u8 packetType;
	u32 time;
};

struct PongPacket
{
	PongPacket() : packetType(EPacketType::Pong) {}
	
	u8 packetType;
	u32 time;
};

struct TelemetryPacket
{
	TelemetryPacket() : packetType(EPacketType::Telemetry) {}

	u8 packetType;
	u16 time;           // in s

	f32 gpsLat, gpsLon; // in degrees
	s32 gpsAlt : 18;    // in m
	s32 gpsCourse : 10; // in degrees
	u8 gpsSpeed;        // in m/s
	
	s8 tmpInternal;     // in deg C
	s8 tmpExternal;     // in deg C

	u16 batteryVoltage; // in mV
};

