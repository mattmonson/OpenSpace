#pragma once

#include <Core.h>
#include <XTendAPI.h>

const u32 LoggingInterval = 1000ul;
const u32 LoggingStagger  = 250ul;
const u32 LCDInterval     = 1000ul;
const u32 LCDStagger      = 0ul;
const u32 PingInterval    = 30000ul;
const u32 PingStagger     = 500ul;

#define LoggingBaud 115200

#define LCDSerial Serial2
#define LCDBaud 9600
#define LCDPagePin 2
#define LCDPageCount 6
#define LCDPageTime 3000

#define GPSSerial Serial1
#define GPSBaud 115200

#define XTendSerial Serial3
#define XTendBaud 115200
#define XTendPTTPin 52
const XTendAPI::Address XTendDest = 0x6905;

const u32 AscentTrackingIntervals[] = {5000, 30000};

