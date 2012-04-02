#include <Core.h>
#include <Wire.h>
#include <VectorMath.h>
#include <MatrixMath.h>
#include <Quaternion.h>
#include <TinyGPS.h>

#include <XTendAPI.h>

#include "Config.h"
#include "Packets.h"

u32 lastFrameTime = 0;

TinyGPS gps;

XTendAPI xtend(&XTendSerial);

u32 loggingLastSend = 0;
u32 lcdLastSend = 0;
bool lcdPageButtonPressed = false;
u32 lcdPageButtonLastChange = 0;
u32 lcdPage = LCDPageCount;

u32 pingLastSend = 0;
u32 pingSendCount = 0;
u32 pingReceiveCount = 0;
u32 pingRTT = 0;
u32 pingRTTMin = 0;
u32 pingRTTMax = 0;

u32 telemetryReceiveCount = 0;
u32 latestTelemetryReceiveTime = 0;
TelemetryPacket latestTelemetryPacket;

u8 latestSignalStrength = 0;

// data for tracking the ascent rate
struct AscentRateData
{
	AscentRateData() { m_Time = 0; }

	u32 m_Time;
	f32 m_Alt;
	f32 m_AscentRate;
};

AscentRateData ascentRateData[_countof(AscentTrackingIntervals)];



void xtendReceive(u32 now);
void handlePong(u32 now, const PongPacket& packet);
void handleTelemetry(u32 now, const TelemetryPacket& packet);
void transmitHeadings();
void transmitLogging(u32 now);
void transmitLCD(u32 now);
void transmitPing(u32 now);

void setup()
{
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	pinMode(12, OUTPUT);
	digitalWrite(12, LOW);

	pinMode(LCDPagePin, INPUT);
	digitalWrite(LCDPagePin, HIGH);

	Serial.begin(LoggingBaud);

	LCDSerial.begin(LCDBaud);
	GPSSerial.begin(GPSBaud);

	// xtend setup
	XTendSerial.begin(XTendBaud);
	pinMode(XTendPTTPin, OUTPUT);
	digitalWrite(XTendPTTPin, HIGH);
	
	delay(1000);

	u32 now = millis();
	lastFrameTime = now;
	loggingLastSend = now - LoggingStagger;
	lcdLastSend = now - LCDStagger;
	pingLastSend = now - PingStagger;
	
	transmitHeadings();

	digitalWrite(13, LOW);
}

void loop()
{
	u32 now = millis();
	f32 dt = (now - lastFrameTime) * 0.001f;
	lastFrameTime = now;

	//digitalWrite(13, (now / 250) % 2);
	digitalWrite(12, (now - latestTelemetryReceiveTime) < 550);

	// update sensors
	while (GPSSerial.available())
		gps.encode(GPSSerial.read());

	// update LCD page button
	if (now - lcdPageButtonLastChange >= 150 && (digitalRead(LCDPagePin) == LOW) != lcdPageButtonPressed)
	{
		lcdPageButtonPressed = !lcdPageButtonPressed;
		lcdPageButtonLastChange = now;
		if (lcdPageButtonPressed)
		{
			lcdPage = (lcdPage + 1) % (LCDPageCount + 1);
			transmitLCD(now);
		}
	}

	// network receive
	xtendReceive(now);

	// time to transmit?
	if (now - loggingLastSend >= LoggingInterval)
		transmitLogging(now);
	if (now - lcdLastSend >= LCDInterval)
		transmitLCD(now);
	//if (now - pingLastSend >= PingInterval)
	//	transmitPing(now);
}

void xtendReceive(u32 now)
{
	const XTendAPI::Frame* pFrame;
	while (xtend.Receive(&pFrame))
	{
		if (pFrame && pFrame->m_PayloadLength >= 1)
		{
			latestSignalStrength = pFrame->m_RSSI;

			switch (pFrame->m_Payload[0])
			{
			case EPacketType::Pong:
				if (pFrame->m_PayloadLength == sizeof(PingPacket))
					handlePong(now, *reinterpret_cast<const PongPacket*>(pFrame->m_Payload));
				break;
				
			case EPacketType::Telemetry:
				if (pFrame->m_PayloadLength == sizeof(TelemetryPacket))
					handleTelemetry(now, *reinterpret_cast<const TelemetryPacket*>(pFrame->m_Payload));
				break;
			}
		}
	}
}

void handlePong(u32 now, const PongPacket& packet)
{
	pingRTT = millis() - packet.time;
	
	if (!pingReceiveCount)
	{
		pingRTTMin = pingRTT;
		pingRTTMax = pingRTT;
	}
	else
	{
		pingRTTMin = min(pingRTTMin, pingRTT);
		pingRTTMax = max(pingRTTMax, pingRTT);
	}
	
	++pingReceiveCount;

	// make sure the LCD is up to date
	transmitLCD(now);
}

void handleTelemetry(u32 now, const TelemetryPacket& packet)
{
	++telemetryReceiveCount;
	latestTelemetryReceiveTime = now;
	latestTelemetryPacket = packet;

	for (u32 i=0; i<_countof(AscentTrackingIntervals); ++i)
	{
		if (ascentRateData[i].m_Time == 0)
		{
			ascentRateData[i].m_Time = now;
			ascentRateData[i].m_Alt = packet.gpsAlt;
		}
		else if (now >= ascentRateData[i].m_Time + AscentTrackingIntervals[i])
		{
			f32 interval = (now - ascentRateData[i].m_Time) / 1000.0f;
			ascentRateData[i].m_AscentRate = (packet.gpsAlt - ascentRateData[i].m_Alt) / interval;

			ascentRateData[i].m_Time = now;
			ascentRateData[i].m_Alt = packet.gpsAlt;
		}
	}

	Serial.print("Telemetry,");
	Serial.print(now);
	Serial.print(',');
	Serial.print(-(int)latestSignalStrength);
	Serial.print(',');
	Serial.print(telemetryReceiveCount);
	Serial.print(',');
	Serial.print(packet.time);
	Serial.print(',');
	Serial.print(packet.gpsLat, 6);
	Serial.print(',');
	Serial.print(packet.gpsLon, 6);
	Serial.print(',');
	Serial.print(packet.gpsAlt);
	Serial.print(',');

	for (u32 i=0; i<_countof(AscentTrackingIntervals); ++i)
	{
		Serial.print(ascentRateData[i].m_AscentRate, 3);
		Serial.print(',');
	}

	Serial.print(packet.gpsCourse);
	Serial.print(',');
	Serial.print(TinyGPS::f_cardinal(packet.gpsCourse));
	Serial.print(',');
	Serial.print((int)packet.gpsSpeed);
	Serial.print(',');
	Serial.print((int)packet.tmpInternal);
	Serial.print(',');
	Serial.print((int)packet.tmpExternal);
	Serial.print(',');
	Serial.print(packet.batteryVoltage / 1000.0f, 3);
	Serial.print(',');

	Serial.println();

	// make sure the LCD is up to date
	transmitLCD(now);
}

void transmitHeadings()
{
	// for Logging rows:
	Serial.print("Logging,");
	Serial.print("now (ms),");
	
	Serial.print("gpsTime,");
	Serial.print("gpsLat (deg),");
	Serial.print("gpsLon (deg),");
	Serial.print("gpsAlt (m),");
	Serial.print("gpsCourse (deg),");
	Serial.print("gpsCourse (cardinal),");
	Serial.print("gpsSpeed (m/s),");
	Serial.print("gpsSats,");
	Serial.print("range (m),");
	Serial.print("bearing (deg),");
	Serial.print("bearing (cardinal),");
	
	Serial.println();

	// for Telemetry rows:
	Serial.print("Telemetry,");
	Serial.print("now (ms),");
	Serial.print("signal strength (-dBm),");
	Serial.print("recvNum,");
	Serial.print("uptime (s),");
	Serial.print("gpsLat (deg),");
	Serial.print("gpsLon (deg),");
	Serial.print("gpsAlt (m),");

	for (u32 i=0; i<_countof(AscentTrackingIntervals); ++i)
	{
		Serial.print("gpsAsc (m/s@");
		Serial.print(AscentTrackingIntervals[i] / 1000);
		Serial.print("s),");
	}

	Serial.print("gpsCourse (deg),");
	Serial.print("gpsCourse (cardinal),");
	Serial.print("gpsSpeed (m/s),");
	Serial.print("tmpInt (C),");
	Serial.print("tmpExt (C),");
	Serial.print("battery (V),");

	Serial.println();
}

void transmitLogging(u32 now)
{
	loggingLastSend = now;

	Serial.print("Logging,");
	Serial.print(now);	
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
	Serial.print(gps.f_course(), 3);
	Serial.print(',');
	Serial.print(TinyGPS::f_cardinal(gps.f_course()));
	Serial.print(',');
	Serial.print(gps.f_speed_mps(), 3);
	Serial.print(',');
	Serial.print(gps.satellites());
	Serial.print(',');

	if (gps.get_has_fix() && telemetryReceiveCount > 0)
	{
		float lat, lon;
		gps.f_get_position(&lat, &lon);

		Serial.print(TinyGPS::distance_between(lat, lon, latestTelemetryPacket.gpsLat, latestTelemetryPacket.gpsLon), 3);
		Serial.print(',');
		Serial.print(TinyGPS::course_to(lat, lon, latestTelemetryPacket.gpsLat, latestTelemetryPacket.gpsLon), 0);
		Serial.print(',');
		Serial.print(TinyGPS::f_cardinal(TinyGPS::course_to(lat, lon, latestTelemetryPacket.gpsLat, latestTelemetryPacket.gpsLon)));
		Serial.print(',');
	}
	else
	{
		Serial.print(',');
		Serial.print(',');
		Serial.print(',');
	}

	Serial.println();
}

void transmitLCD(u32 now)
{
	lcdLastSend = now;

	const char* c_Clear = "\xFE\x01";
	const char* c_GoToLine2 = "\xFE\xC0";

	// clear
	LCDSerial.print(c_Clear);

	switch (lcdPage < LCDPageCount ? lcdPage : now / LCDPageTime % 6)
	{
	case 0:
		LCDSerial.print("Pos ");
		LCDSerial.print(fabs(latestTelemetryPacket.gpsLat), 6);
		LCDSerial.print(' ');
		LCDSerial.print(latestTelemetryPacket.gpsLat >= 0 ? 'N' : 'S');

		LCDSerial.print(c_GoToLine2);
		LCDSerial.print("    ");
		LCDSerial.print(fabs(latestTelemetryPacket.gpsLon), 6);
		LCDSerial.print(' ');
		LCDSerial.print(latestTelemetryPacket.gpsLon >= 0 ? 'E' : 'W');
		
		break;

	case 1:
		LCDSerial.print("Alt ");
		LCDSerial.print(latestTelemetryPacket.gpsAlt);
		LCDSerial.print('m');

		LCDSerial.print(c_GoToLine2);
		LCDSerial.print("Asc ");

		for (u32 i=0; i<_countof(AscentTrackingIntervals); ++i)
		{
			if (i)
			{
				LCDSerial.print('/');
			}

			LCDSerial.print(ascentRateData[i].m_AscentRate, 1);
		}

		LCDSerial.print("m/s");

		break;

	case 2:
		LCDSerial.print("Spd ");
		LCDSerial.print((int)latestTelemetryPacket.gpsSpeed);
		LCDSerial.print("m/s ");
		LCDSerial.print(TinyGPS::f_cardinal(latestTelemetryPacket.gpsCourse));

		LCDSerial.print(c_GoToLine2);
		LCDSerial.print("Car ");
		if (gps.get_has_fix())
		{
			LCDSerial.print(gps.f_speed_mps(), 0);
			LCDSerial.print("m/s ");
			LCDSerial.print(TinyGPS::f_cardinal(gps.f_course()));
		}
		else
		{
			LCDSerial.print("---");
		}

		break;
	
	case 3:
		LCDSerial.print("Rng ");
		if (gps.get_has_fix() && telemetryReceiveCount > 0)
		{
			float lat, lon;
			gps.f_get_position(&lat, &lon);
			LCDSerial.print(TinyGPS::distance_between(lat, lon, latestTelemetryPacket.gpsLat, latestTelemetryPacket.gpsLon), 0);
			LCDSerial.print("m ");
			LCDSerial.print(TinyGPS::f_cardinal(TinyGPS::course_to(lat, lon, latestTelemetryPacket.gpsLat, latestTelemetryPacket.gpsLon)));
		}
		else
		{
			LCDSerial.print("---");
		}

		LCDSerial.print(c_GoToLine2);
		LCDSerial.print("Sig ");
		LCDSerial.print(-(int)latestSignalStrength);
		LCDSerial.print("dBm");

		break;

	case 4:
		LCDSerial.print("Bat ");
		LCDSerial.print(latestTelemetryPacket.batteryVoltage / 1000.0f, 3);
		LCDSerial.print('v');

		LCDSerial.print(c_GoToLine2);
		LCDSerial.print("Tmp ");
		LCDSerial.print((int)latestTelemetryPacket.tmpInternal);
		LCDSerial.print("c ");
		LCDSerial.print((int)latestTelemetryPacket.tmpExternal);
		LCDSerial.print('c');

		break;

	case 5:
		LCDSerial.print("Upt ");
		LCDSerial.print(latestTelemetryPacket.time);
		LCDSerial.print('+');

		LCDSerial.print((now - latestTelemetryReceiveTime) / 1000);
		LCDSerial.print('s');

		LCDSerial.print(c_GoToLine2);
		LCDSerial.print("Pkt #");
		LCDSerial.print(telemetryReceiveCount);

		break;

	case LCDPageCount:
		break;
	}
}

void transmitPing(u32 now)
{
	pingLastSend = now;
	
	PingPacket packet;
	packet.time = millis();

	++pingSendCount;

	xtend.SendTo(XTendDest, (u8*)&packet, sizeof(packet));
}

