#pragma once

#include <Core.h>
#include <XTendAPI.h>

const uint32_t LoggingInterval = 1000ul;
const uint32_t LoggingStagger  = 250ul;
const uint32_t LCDInterval     = 1000ul;
const uint32_t LCDStagger      = 0ul;
const uint32_t PingInterval    = 30000ul;
const uint32_t PingStagger     = 500ul;

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

const uint32_t AscentTrackingIntervals[] = {5000, 30000};

