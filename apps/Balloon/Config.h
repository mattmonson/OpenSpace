#pragma once

#include <Core.h>
#include <XTendAPI.h>

const uint32_t TargetFrameTime           = 0ul;
const uint32_t LoggingInterval           = 100ul;
const uint32_t LoggingStagger            = 0ul;
const uint32_t TelemetryTransmitInterval = 500ul;
const uint32_t TelemetryTransmitStagger  = 250ul;

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



#define XTendSerialTXPin 5
#define XTendSerialRXPin 4
#define XTendBaud 9600
const XTendAPI::Address XTendDest = 0x5854;

