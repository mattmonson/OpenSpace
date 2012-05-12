#include <Core.h>
#include <Thermistor.h>
#include <FPS.h>
#include <TinyGPS.h>
#include <Flash.h>
#include "Jonah.h"
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

SoftwareSerial JonahSerial(11, 9);
JonahRX jonahRX;

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


void onJonahReceive(const uint8_t* data, size_t size);
void transmitLoggingHeadings();
void transmitLogging(uint32_t now);
void transmitAPRS(uint32_t now);

void setup()
{
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	Serial.begin(115200);
	JonahSerial.begin(4800);

	delay(3000);

	pinMode(BatteryMonitorPin, INPUT);
	batteryVoltage = analogRead(BatteryMonitorPin) / 1023.0f * 5.0f;
	batteryVoltageSmooth = batteryVoltage;

	for (uint8_t i=0; i<_countof(therms); ++i)
		thermTempsFiltered[i] = therms[i].getTemp();

	packet.setPTTPin(APRSPTTPin);
	pinMode(APRSTXPin, OUTPUT);

	TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS20);
	OCR2B = 0;

	uint32_t now = millis();
	lastFrameTime = now;
	loggingLastSend = now;
	lastTransmit = now - TransmitPeriod + 1000;

	transmitLoggingHeadings();

	digitalWrite(13, LOW);
}

void loop()
{
	const uint32_t now = millis();
	const float dt = (now - lastFrameTime) * 0.001f;
	lastFrameTime = now;

	fps.increment();

	digitalWrite(13, (now / 250) % 2);
	
	batteryVoltage = analogRead(BatteryMonitorPin) / 1023.0f * 5.0f;
	batteryVoltageSmooth = LowPassFilter(batteryVoltage, batteryVoltageSmooth, dt, 2.5f);

	for (uint8_t i=0; i<_countof(therms); ++i)
		thermTempsFiltered[i] = LowPassFilter((float)therms[i].getTemp(), thermTempsFiltered[i], dt, 2.5f);

	while (Serial.available())
		gps.encode((char)Serial.read());

	while (JonahSerial.available())
		if (jonahRX.onReceive(JonahSerial.read()))
			onJonahReceive(jonahRX.getData(), jonahRX.getDataSize());

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

void onJonahReceive(const uint8_t* data, size_t size)
{
	struct Packet
	{
		uint32_t now;
		uint16_t batteryVoltage; // in centi-Volts
		int16_t thermTemp;       // in deci-C
		int16_t bmpTemp;         // in deci-C
		uint32_t bmpPressure;    // in Pa
	};

	if (size != sizeof(Packet))
	{
		//Serial.print("Unexpected size for Jonah packet: ");
		//Serial.println(size);
		return;
	}

	const Packet* p = reinterpret_cast<const Packet*>(data);
	Serial.print(p->now);
	Serial.print(',');
	Serial.print(p->batteryVoltage);
	Serial.print(',');
	Serial.print(p->thermTemp);
	Serial.print(',');
	Serial.print(p->bmpTemp);
	Serial.print(',');
	Serial.print(p->bmpPressure);
	Serial.println();
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
	
	Serial.println();
}

void transmitAPRS(uint32_t now)
{
	lastTransmit = now;
	++msgNum;

	AX25Address dest = {"APRS", 0};
	uint8_t pathCount = c_FullPathCount;

	char msg[128];

	{
		uint8_t hours, minutes, seconds;
		float lat, lon;

		gps.crack_datetime(NULL, NULL, NULL, &hours, &minutes, &seconds, NULL, NULL);
		gps.f_get_position(&lat, &lon, NULL);
		float course = gps.f_course();
		float speed = gps.f_speed_mps();
		float alt = gps.f_altitude();

		if (alt >= c_HighAltitudeCutoff)
		{
			pathCount = 0;
		}

#if 0
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

		sprintf(msg, "%sTi=%ld/Te=%ld/V=%ld.%.2ld/A'=%.2ld/#%lu", 
			miceInfo,
			(int32_t)thermTempsFiltered[0],
			(int32_t)thermTempsFiltered[1],
			(int32_t)fabs(batteryVoltageSmooth),
			(int32_t)fabs(batteryVoltageSmooth * 100) % 100,
			(int32_t)(ascentRate * 10),
			msgNum
		);
#else
		sprintf(msg, ";CXXISAT00*%.2hu%.2hu%.2huh%.2d%.2ld.%.2ld%c/%.3d%.2ld.%.2ld%c>%.3ld/%.3ld/A=%.6ld/Ti=%ld/Te=%ld/V=%ld.%.2ld/#%lu", 
			hours,
			minutes,
			seconds,
			(int16_t)fabs(lat),
			(int32_t)fabs(lat * 60) % 60,
			(int32_t)fabs(lat * 60 * 100) % 100,
			lat > 0 ? 'N' : 'S',
			(int16_t)fabs(lon),
			(int32_t)fabs(lon * 60) % 60,
			(int32_t)fabs(lon * 60 * 100) % 100,
			lon > 0 ? 'E' : 'W',
			(int32_t)course,
			(int32_t)METERS_PER_SECOND_TO_KNOTS(speed),
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
	Serial.println();
#endif

	packet.build(c_SrcAddress, dest, c_FullPath, pathCount, msg);
	packet.transmit(&sinewave);

	//Serial.println(GetFreeMemory());
}

