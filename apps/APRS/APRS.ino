#include <Core.h>
#include <Thermistor.h>
#include <FPS.h>
#include <TinyGPS.h>
#include <AX25.h>
#include <Sinewave.h>
#include <TimerOne.h>

#include <Wire.h>

#define LoggingInterval 1000ul
u32 loggingLastSend = 0;

u32 lastFrameTime = 0;
FPS fps(0);

#define BatteryMonitorPin A3
f32 batteryVoltage = 0.0f;
f32 batteryVoltageSmooth = 0.0f;

#define ThermistorPins {A1, A2}
Thermistor therms[2] = ThermistorPins;
f32 thermTempsFiltered[_countof(therms)];

TinyGPS gps;

// data for tracking the ascent rate
const u32 c_AscentRateInterval = 15000ul;
u32 ascentRateTime = 0;
f32 ascentRateAlt = 0.0f;
f32 ascentRate = 0.0f;

#define APRSPTTPin 2
#define APRSTXPin 3
AX25Packet packet;
Sinewave sinewave(&OCR2B, 256, 0xFF);
u32 msgNum = 0;

const AX25Address c_SrcAddress = {"KF7OCC", 11};
const AX25Address c_FullPath[] = {
	{"WIDE1", 1},
	{"WIDE2", 1},
};
const u8 c_FullPathCount = sizeof(c_FullPath) / sizeof(c_FullPath[0]);

const f32 c_HighAltitudeCutoff = 1500.0f;

const u32 TransmitPeriod = 81000ul;
u32 lastTransmit = 0;


void transmitLoggingHeadings();
void transmitLogging(u32 now);
void transmitAPRS(u32 now);

void setup()
{
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	Serial.begin(115200);

	delay(3000);

	pinMode(BatteryMonitorPin, INPUT);
	batteryVoltage = analogRead(BatteryMonitorPin) / 1023.0f * 5.0f;
	batteryVoltageSmooth = batteryVoltage;

	for (u8 i=0; i<_countof(therms); ++i)
		thermTempsFiltered[i] = therms[i].getTemp();

	packet.setPTTPin(APRSPTTPin);
	pinMode(APRSTXPin, OUTPUT);

	TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS20);
	OCR2B = 0;

	u32 now = millis();
	lastFrameTime = now;
	loggingLastSend = now;
	lastTransmit = now - TransmitPeriod + 1000;

	transmitLoggingHeadings();

	digitalWrite(13, LOW);
}

void loop()
{
	const u32 now = millis();
	const f32 dt = (now - lastFrameTime) * 0.001f;
	lastFrameTime = now;

	fps.increment();

	digitalWrite(13, (now / 250) % 2);
	
	batteryVoltage = analogRead(BatteryMonitorPin) / 1023.0f * 5.0f;
	batteryVoltageSmooth = LowPassFilter(batteryVoltage, batteryVoltageSmooth, dt, 2.5f);

	for (u8 i=0; i<_countof(therms); ++i)
		thermTempsFiltered[i] = LowPassFilter((f32)therms[i].getTemp(), thermTempsFiltered[i], dt, 2.5f);

	while (Serial.available())
		gps.encode((char)Serial.read());

	// ascent rate
	if (gps.get_has_fix())
	{
		if (ascentRateTime == 0)
		{
			ascentRateTime = now;
			ascentRateAlt = gps.f_altitude();
		}
		else if (now - ascentRateTime > c_AscentRateInterval)
		{
			const f32 alt = gps.f_altitude();
			ascentRate = (alt - ascentRateAlt) * 1000 / (now - ascentRateTime);
			ascentRateTime = now;
			ascentRateAlt = alt;
		}
	}

	// time to transmit?
	if (now - loggingLastSend >= LoggingInterval)
		transmitLogging(now);
	if (now - lastTransmit >= TransmitPeriod && gps.get_has_fix())
		transmitAPRS(now);
	
	fps.loop();
}

void transmitLoggingHeadings()
{
	Serial.print("now (ms),");
	Serial.print("battery (V),");

	Serial.print("gpsTime,");
	Serial.print("gpsLat (deg),");
	Serial.print("gpsLon (deg),");
	Serial.print("gpsAlt (m),");
	Serial.print("gpsAscentRate (m/s),");
	Serial.print("gpsCourse (deg),");
	Serial.print("gpsCourse (cardinal),");
	Serial.print("gpsSpeed (m/s),");
	Serial.print("gpsSats,");
	
	for (u8 i=0; i<_countof(therms); ++i)
		serprintf(Serial, "thermistor%hu (deg C),", i);
	
	Serial.println();
}

void transmitLogging(u32 now)
{
	loggingLastSend = now;

	Serial.print(now);
	Serial.print(',');

	Serial.print(batteryVoltageSmooth, 3);
	Serial.print(',');

	u8 hours, minutes, seconds, hundredths;
	f32 lat, lon;
	gps.crack_datetime(NULL, NULL, NULL, &hours, &minutes, &seconds, &hundredths);
	gps.f_get_position(&lat, &lon);

	Serial.print((int)hours);
	Serial.print(':');
	Serial.print((int)minutes);
	Serial.print(':');
	Serial.print(seconds + 0.01f * hundredths, 2);
	Serial.print(',');
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

	for (u8 i=0; i<_countof(therms); ++i)
	{
		Serial.print(thermTempsFiltered[i], 2);
		Serial.print(',');
	}
	
	Serial.println();
}

void transmitAPRS(u32 now)
{
	lastTransmit = now;
	++msgNum;

	s16 year;
	u8 hours, minutes, seconds;
	f32 lat, lon;

	gps.crack_datetime(NULL, NULL, NULL, &hours, &minutes, &seconds, NULL, NULL);
	gps.f_get_position(&lat, &lon, NULL);
	f32 course = gps.f_course();
	f32 speed = gps.f_speed_mps();
	f32 alt = gps.f_altitude();

	AX25Address dest = {"APRS", 0};
	char msg[256];

	if (true)
	{
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
			(s32)thermTempsFiltered[0],
			(s32)thermTempsFiltered[1],
			(s32)fabs(batteryVoltageSmooth),
			(s32)fabs(batteryVoltageSmooth * 100) % 100,
			(s32)(ascentRate * 10),
			msgNum
		);
	}
	else
	{
		sprintf(msg, ";MYBALLOON*%.2hu%.2hu%.2huh%.2d%.2ld.%.2ld%c/%.3d%.2ld.%.2ld%c>%.3ld/%.3ld/A=%.6ld/Ti=%ld/Te=%ld/V=%ld.%.2ld/#%lu", 
			hours,
			minutes,
			seconds,
			(s16)fabs(lat),
			(s32)fabs(lat * 60) % 60,
			(s32)fabs(lat * 60 * 100) % 100,
			lat > 0 ? 'N' : 'S',
			(s16)fabs(lon),
			(s32)fabs(lon * 60) % 60,
			(s32)fabs(lon * 60 * 100) % 100,
			lon > 0 ? 'E' : 'W',
			(s32)course,
			(s32)METERS_PER_SECOND_TO_KNOTS(speed),
			(s32)METERS_TO_FEET(max(alt, 0.0f)),
			(s32)thermTempsFiltered[0],
			(s32)thermTempsFiltered[1],
			(s32)fabs(batteryVoltageSmooth),
			(s32)fabs(batteryVoltageSmooth * 100) % 100,
			msgNum
		);
	}

	const u8 pathCount = alt >= c_HighAltitudeCutoff ? 0 : c_FullPathCount;

#if 1
	serprintf(Serial, "%lu,%s-%d",
		now,
		c_SrcAddress.m_CallSign,
		(int)c_SrcAddress.m_SSID
	);

	for (int i=0; i<pathCount; ++i)
		serprintf(Serial, "/%s-%d", c_FullPath[i].m_CallSign, (int)c_FullPath[i].m_SSID);

	serprintf(Serial, ">%s-%d:%s\n",
		dest.m_CallSign,
		(int)dest.m_SSID,
		msg
	);
#endif

	packet.build(c_SrcAddress, dest, c_FullPath, pathCount, msg);
	packet.transmit(&sinewave);
}

