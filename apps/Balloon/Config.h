#pragma once

#include <Core.h>
//#include <AX25.h>
#include <XTendAPI.h>

const u32 TargetFrameTime           = 0ul;
const u32 LoggingInterval           = 100ul;
const u32 LoggingStagger            = 0ul;
const u32 TelemetryTransmitInterval = 500ul;
const u32 TelemetryTransmitStagger  = 250ul;

#define LoggingBaud 115200

#define GPSSerial Serial
#define GPSBaud 115200

#define BatteryMonitorPin A3

struct EThermistors
{
	enum Enum
	{
		Internal,
		External1,
		External2,
		
		EnumCount
	};
};

#define ThermistorPins {A0, A1, A2}

struct ETMPs
{
	enum Enum
	{
		Internal,
		External,
		
		EnumCount
	};
};

#define TMPAddresses { TMP102::EAddress::GND, TMP102::EAddress::V }



//#define XTendSerial Serial3
#define XTendSerialTXPin 5
#define XTendSerialRXPin 4
#define XTendBaud 9600
//#define XTendPTTPin 13
const XTendAPI::Address XTendDest = 0x5854;

