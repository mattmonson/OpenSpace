#include <Core.h>
#include <Thermistor.h>
#include <FPS.h>
#include <TinyGPS.h>
#include <BMP085.h>
#include <Flash.h>
#include "Jonah.h"
#include <CRC.h>
#include <AX25.h>
#include <Sinewave.h>
#include <TimerOne.h>

#include <Wire.h>
#include <SoftwareSerial.h>

#define LoggingInterval 1000ul
uint32_t loggingLastSend = 0;

uint32_t lastFrameTime = 0;
FPS fps(0);

#define BatteryMonitorPin A3
float batteryVoltage = 0.0f;
float batteryVoltageSmooth = 0.0f;

#define ThermistorPins {A0, A1, A2}
Thermistor therms[] = ThermistorPins;
float thermTempsFiltered[_countof(therms)];

TinyGPS gps;

#define I2CEnablePin 12
BMP085 pressure;
float pressureFiltered;

#define JonahBaud 4800
SoftwareSerial JonahSerial(11, 9);
JonahRX jonahRX;

bool jonahListening = false;
uint32_t lastJonahListenStart = 0;
uint32_t jonahReceiveCount = 0;
uint32_t jonahListenCount = 0;
const uint32_t c_JonahListenPeriod = 3000ul; // how often we'll turn on the JonahSerial and look for a JonahPacket
const uint32_t c_JonahListenTimeout = 250ul; // how long we'll listen before giving up

struct JonahPacket
{
	uint32_t now;
	uint32_t batteryVoltage : 14; // in milli-Volts
	uint32_t bmpPressure    : 18; // in Pa
	int32_t bmpTemp         : 12; // in deci-C
	int32_t thermTemp       : 20; // in milli-C
};

JonahPacket lastJonahPacket;
uint32_t lastJonahPacketReceiveTime = 0;

// data for tracking the ascent rate
const uint32_t c_AscentRateInterval = 15000ul;
uint32_t ascentRateTime = 0;
float ascentRateAlt = 0.0f;
float ascentRate = 0.0f;

#define APRSPTTPin 4
#define APRSTXPin 3
AX25Packet packet;
Sinewave sinewave(&OCR2B, 256, 0xFF);
uint32_t msgNum = 0;

const AX25Address c_SrcAddress = {"KF7OCC", 0};
const AX25Address c_FullPath[] = {
	{"WIDE1", 1},
	{"WIDE2", 1},
};
const uint8_t c_FullPathCount = sizeof(c_FullPath) / sizeof(c_FullPath[0]);

const float c_HighAltitudeCutoff = 1500.0f;

const uint32_t TransmitPeriod = 60000ul;
uint32_t lastTransmit = 0;


void jonahUpdate(uint32_t now);
void jonahListen(uint32_t now);
void jonahIgnore();
void onJonahReceive(const uint8_t* data, size_t size);
void transmitLoggingHeadings();
void transmitLogging(uint32_t now);
void transmitAPRS(uint32_t now);

void setup()
{
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	Serial.begin(115200);

	pinMode(I2CEnablePin, OUTPUT);
	digitalWrite(I2CEnablePin, HIGH);
	Wire.begin();

	delay(1000);

	pinMode(BatteryMonitorPin, INPUT);
	batteryVoltage = analogRead(BatteryMonitorPin) / 1023.0f * 5.0f;
	batteryVoltageSmooth = batteryVoltage;

	for (uint8_t i=0; i<_countof(therms); ++i)
		thermTempsFiltered[i] = Clamp(therms[i].getTemp(), -99.0, 99.0);

	pressure.setup();
	pressure.SetOversamplingSetting(3);
	pressure.SetReferencePressure(101325);
	pressure.loop();
	pressureFiltered = pressure.GetPressureInPa();

	packet.setPTTPin(APRSPTTPin);
	pinMode(APRSTXPin, OUTPUT);

	TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS20);
	OCR2B = 0;

	uint32_t now = millis();
	lastFrameTime = now;
	loggingLastSend = now;
	lastJonahListenStart = now - c_JonahListenPeriod;
	lastTransmit = now - TransmitPeriod + 5000;

	transmitLoggingHeadings();

	digitalWrite(13, LOW);
}

void loop()
{
	const uint32_t now = millis();
	const float dt = (now - lastFrameTime) * 0.001f;
	lastFrameTime = now;

	fps.increment();

	bool ledStatus =
		(now % 1000) < 100 ||
		(now - lastJonahPacketReceiveTime) < 500ul;
	digitalWrite(13, ledStatus);
	
	batteryVoltage = analogRead(BatteryMonitorPin) / 1023.0f * 5.0f;
	batteryVoltageSmooth = LowPassFilter(batteryVoltage, batteryVoltageSmooth, dt, 2.5f);

	for (uint8_t i=0; i<_countof(therms); ++i)
		thermTempsFiltered[i] = LowPassFilter((float)Clamp(therms[i].getTemp(), -99.0, 99.0), thermTempsFiltered[i], dt, 2.5f);

	while (Serial.available())
		gps.encode((char)Serial.read());

	pressure.loopAsync();
	pressureFiltered = LowPassFilter((float)pressure.GetPressureInPa(), pressureFiltered, dt, 2.5f);

	jonahUpdate(now);

	// ascent rate
	if (gps.get_position(NULL, NULL))
	{
		if (ascentRateTime == 0)
		{
			ascentRateTime = now;
			ascentRateAlt = gps.f_altitude();
		}
		else if (now - ascentRateTime > c_AscentRateInterval)
		{
			const float alt = gps.f_altitude();
			ascentRate = (alt - ascentRateAlt) * 1000 / (now - ascentRateTime);
			ascentRateTime = now;
			ascentRateAlt = alt;
		}
	}

	// time to transmit?
	if (now - loggingLastSend >= LoggingInterval)
		transmitLogging(now);
	if (now - lastTransmit >= TransmitPeriod && gps.get_position(NULL, NULL) && gps.get_datetime(NULL, NULL))
		transmitAPRS(now);
	
	fps.loop();
}

void jonahUpdate(uint32_t now)
{
	// handle jonah
	if (jonahListening && now - lastJonahListenStart >= c_JonahListenTimeout)
	{
		jonahIgnore();
	}
	else if (jonahListening)
	{
		while (JonahSerial.available() && millis() - lastJonahListenStart < c_JonahListenTimeout)
		{
			if (jonahRX.onReceive(JonahSerial.read()))
			{
				onJonahReceive(jonahRX.getData(), jonahRX.getDataSize());
				jonahIgnore();
				break;
			}
		}
	}
	else if (now - lastJonahListenStart >= c_JonahListenPeriod)
	{
		jonahListen(now);
	}
}

void jonahListen(uint32_t now)
{
    ++jonahListenCount;
	lastJonahListenStart = now;
	jonahListening = true;
	JonahSerial.begin(JonahBaud);
}

void jonahIgnore()
{
	jonahListening = false;
	JonahSerial.end();
}

void onJonahReceive(const uint8_t* data, size_t size)
{
	if (size != sizeof(JonahPacket))
	{
		return;
	}

    ++jonahReceiveCount;
	lastJonahPacket = *reinterpret_cast<const JonahPacket*>(data);
	lastJonahPacketReceiveTime = millis();

#if 0
	Serial << F("Jonah,");
	Serial.print(millis());
	Serial.print(',');
	Serial.print(lastJonahPacket.now);
	Serial.print(',');
	Serial.print(lastJonahPacket.batteryVoltage * 0.001, 3);
	Serial.print(',');
	Serial.print(lastJonahPacket.bmpPressure);
	Serial.print(',');
	Serial.print(lastJonahPacket.bmpTemp * 0.1, 1);
	Serial.print(',');
	Serial.print(lastJonahPacket.thermTemp * 0.001, 3);
	Serial.println();
#endif
}

void transmitLoggingHeadings()
{
	Serial << F("now (ms),");
	Serial << F("battery (V),");

	Serial << F("gpsTime,");
	Serial << F("gpsLat (deg),");
	Serial << F("gpsLon (deg),");
	Serial << F("gpsAlt (m),");
	Serial << F("gpsAscentRate (m/s),");
	Serial << F("gpsCourse (deg),");
	Serial << F("gpsCourse (cardinal),");
	Serial << F("gpsSpeed (m/s),");
	Serial << F("gpsSats,");
	
	for (uint8_t i=0; i<_countof(therms); ++i)
	{
		Serial << F("thermistor");
		Serial.print(i);
		Serial << F(" (deg C),");
	}

	Serial << F("bmpTemp (deg C),");
	Serial << F("bmpPressure (Pa),");

    Serial << F("jonah receive count,");
    Serial << F("jonah listen count,");
	Serial << F("jonah now (ms),");
	Serial << F("jonah battery (V),");
	Serial << F("balloon pressure (Pa),");
	Serial << F("balloon bmpTemp (deg C),");
	Serial << F("balloon thermistor (deg C),");

	Serial.println();
}

void transmitLogging(uint32_t now)
{
	loggingLastSend = now;

	Serial.print(now);
	Serial.print(',');

	Serial.print(batteryVoltageSmooth, 3);
	Serial.print(',');

	uint8_t hours, minutes, seconds, hundredths;
	if (gps.crack_datetime(NULL, NULL, NULL, &hours, &minutes, &seconds, &hundredths))
	{
		Serial.print((int)hours);
		Serial.print(':');
		Serial.print((int)minutes);
		Serial.print(':');
		Serial.print(seconds + 0.01f * hundredths, 2);
	}
	Serial.print(',');

	float lat, lon;
	if (gps.f_get_position(&lat, &lon))
	{
		Serial.print(lat, 6);
		Serial.print(',');
		Serial.print(lon, 6);
		Serial.print(',');
		Serial.print(gps.f_altitude(), 3);
		Serial.print(',');
		Serial.print(ascentRate, 3);
		Serial.print(',');
		Serial.print(gps.f_course(), 3);
		Serial.print(',');
		Serial.print(TinyGPS::cardinal(gps.f_course()));
		Serial.print(',');
		Serial.print(gps.f_speed_mps(), 3);
		Serial.print(',');
		Serial.print(gps.satellites());
		Serial.print(',');
	}
	else
	{
		Serial.print(',');
		Serial.print(',');
		Serial.print(',');
		Serial.print(',');
		Serial.print(',');
		Serial.print(',');
		Serial.print(',');
		Serial.print(',');
	}

	for (uint8_t i=0; i<_countof(therms); ++i)
	{
		Serial.print(thermTempsFiltered[i], 2);
		Serial.print(',');
	}

	Serial.print(pressure.GetTempInC(), 1);
	Serial.print(',');
	Serial.print(pressureFiltered, 0);
	Serial.print(',');

    Serial.print(jonahReceiveCount);
	Serial.print(',');
    Serial.print(jonahListenCount);
	Serial.print(',');
	Serial.print(lastJonahPacket.now);
	Serial.print(',');
	Serial.print(lastJonahPacket.batteryVoltage * 0.001, 3);
	Serial.print(',');
	Serial.print(lastJonahPacket.bmpPressure);
	Serial.print(',');
	Serial.print(lastJonahPacket.bmpTemp * 0.1, 1);
	Serial.print(',');
	Serial.print(lastJonahPacket.thermTemp * 0.001, 3);
	Serial.print(',');

	Serial.println();
}

void transmitAPRS(uint32_t now)
{
	lastTransmit = now;
	++msgNum;

	// ignore Jonah because we don't want it's interruptions here
	jonahIgnore();

	AX25Address dest = {"APRS", 0};
	uint8_t pathCount = c_FullPathCount;

	char msg[128];

	{
		uint8_t hours, minutes, seconds;
		float lat, lon;
		float course, speed, alt;

		if (!gps.crack_datetime(NULL, NULL, NULL, &hours, &minutes, &seconds, NULL, NULL))
		{
			hours = minutes = seconds = 0;
		}

		if (gps.f_get_position(&lat, &lon, NULL))
		{
			course = gps.f_course();
			speed = gps.f_speed_mps();
			alt = gps.f_altitude();
		}
		else
		{
			lat = lon = 0;
			course = speed = alt = 0;
		}

		if (alt >= c_HighAltitudeCutoff)
		{
			pathCount = 0;
		}

#if 1
		char miceInfo[32];
		packet.MicECompress(&dest, miceInfo, 
			lat,
			lon,
			alt,
			speed,
			course,
			'O',
			'/'
		);

		sprintf(msg,
			"%s"
			"Ti=%+ld"
			"/Te=%+ld"
			"/Tb=%+ld"
			"/TE=%+ld"
			"/TB=%+ld"
			"/V=%ld.%.2ld"
			"/Vb=%ld.%.2ld"
			"/Pe=%.6ld"
			"/Pb=%.6ld"
			"/A'=%+.2ld"
			"/#%lu", 
			miceInfo,
			(int32_t)thermTempsFiltered[0],
			(int32_t)(0.5f * (thermTempsFiltered[1] + thermTempsFiltered[2])),
			(int32_t)(lastJonahPacket.thermTemp / 1000),
			(int32_t)(pressure.GetTempInDeciC() / 10),
			(int32_t)(lastJonahPacket.bmpTemp / 10),
			(int32_t)fabs(batteryVoltageSmooth),
			(int32_t)fabs(batteryVoltageSmooth * 100) % 100,
			(int32_t)(lastJonahPacket.batteryVoltage / 1000),
			(int32_t)(lastJonahPacket.batteryVoltage / 10) % 100,
			(int32_t)pressureFiltered,
			(int32_t)lastJonahPacket.bmpPressure,
			(int32_t)(ascentRate * 10),
			msgNum
		);
#else
		sprintf(msg, 
			";CXXISAT00*"
			"%.2hu%.2hu%.2huh"
			"%.2d%.2ld.%.2ld%c"
			"%c"
			"%.3d%.2ld.%.2ld%c"
			"%c"
			"%.3ld/%.3ld"
			"/A=%.6ld"
			"/Ti=%+.2ld"
			"/Te=%+.2ld"
			"/V=%ld.%.2ld"
			"/#%.4lu",
			hours, minutes, seconds,
			(int16_t)fabs(lat), (int32_t)fabs(lat * 60) % 60, (int32_t)fabs(lat * 60 * 100) % 100, lat > 0 ? 'N' : 'S',
			'/',
			(int16_t)fabs(lon), (int32_t)fabs(lon * 60) % 60, (int32_t)fabs(lon * 60 * 100) % 100, lon > 0 ? 'E' : 'W',
			'O',
			(int32_t)course, (int32_t)METERS_PER_SECOND_TO_KNOTS(speed),
			(int32_t)METERS_TO_FEET(max(alt, 0.0f)),
			(int32_t)thermTempsFiltered[0],
			(int32_t)thermTempsFiltered[1],
			(int32_t)fabs(batteryVoltageSmooth),
			(int32_t)fabs(batteryVoltageSmooth * 100) % 100,
			msgNum
		);
#endif

	}

#if 1
	Serial.print(now);
	Serial.print(',');
	Serial.print(c_SrcAddress.m_CallSign);
	Serial.print('-');
	Serial.print((int)c_SrcAddress.m_SSID);

	for (int i=0; i<pathCount; ++i)
	{
		Serial.print('/');
		Serial.print(c_FullPath[i].m_CallSign);
		Serial.print('-');
		Serial.print((int)c_FullPath[i].m_SSID);
	}

	Serial.print('>');
	Serial.print(dest.m_CallSign);
	Serial.print('-');
	Serial.print((int)dest.m_SSID);
	Serial.print(msg);
	Serial.print('(');
	Serial.print(strlen(msg));
	Serial.print(')');
	Serial.println();
#endif

	packet.build(c_SrcAddress, dest, c_FullPath, pathCount, msg);
	packet.transmit(&sinewave);

	// wait for the packet to be done transmitting
	while (packet.transmitting())
	{
	}

	//Serial.println(GetFreeMemory());
}

