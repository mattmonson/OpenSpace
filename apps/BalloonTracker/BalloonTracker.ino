#include <Core.h>
#include <Wire.h>
#include <VectorMath.h>
#include <MatrixMath.h>
#include <Quaternion.h>
#include <TinyGPS.h>

#include <XTendAPI.h>

#include "Config.h"
#include "Packets.h"

uint32_t lastFrameTime = 0;

TinyGPS gps;

XTendAPI xtend(&XTendSerial);

uint32_t loggingLastSend = 0;
uint32_t lcdLastSend = 0;
bool lcdPageButtonPressed = false;
uint32_t lcdPageButtonLastChange = 0;
uint32_t lcdPage = LCDPageCount;

uint32_t pingLastSend = 0;
uint32_t pingSendCount = 0;
uint32_t pingReceiveCount = 0;
uint32_t pingRTT = 0;
uint32_t pingRTTMin = 0;
uint32_t pingRTTMax = 0;

uint32_t telemetryReceiveCount = 0;
uint32_t latestTelemetryReceiveTime = 0;
TelemetryPacket latestTelemetryPacket;

uint8_t latestSignalStrength = 0;

// data for tracking the ascent rate
struct AscentRateData
{
	AscentRateData() { m_Time = 0; }

	uint32_t m_Time;
	float m_Alt;
	float m_AscentRate;
};

AscentRateData ascentRateData[_countof(AscentTrackingIntervals)];



void xtendReceive(uint32_t now);
void handlePong(uint32_t now, const PongPacket& packet);
void handleTelemetry(uint32_t now, const TelemetryPacket& packet);
void transmitHeadings();
void transmitLogging(uint32_t now);
void transmitLCD(uint32_t now);
void transmitPing(uint32_t now);

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

	uint32_t now = millis();
	lastFrameTime = now;
	loggingLastSend = now - LoggingStagger;
	lcdLastSend = now - LCDStagger;
	pingLastSend = now - PingStagger;
	
	transmitHeadings();

	digitalWrite(13, LOW);
}

void loop()
{
	uint32_t now = millis();
	float dt = (now - lastFrameTime) * 0.001f;
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

void xtendReceive(uint32_t now)
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

void handlePong(uint32_t now, const PongPacket& packet)
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

void handleTelemetry(uint32_t now, const TelemetryPacket& packet)
{
	++telemetryReceiveCount;
	latestTelemetryReceiveTime = now;
	latestTelemetryPacket = packet;

	for (uint32_t i=0; i<_countof(AscentTrackingIntervals); ++i)
	{
		if (ascentRateData[i].m_Time == 0)
		{
			ascentRateData[i].m_Time = now;
			ascentRateData[i].m_Alt = packet.gpsAlt;
		}
		else if (now >= ascentRateData[i].m_Time + AscentTrackingIntervals[i])
		{
			float interval = (now - ascentRateData[i].m_Time) / 1000.0f;
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

	for (uint32_t i=0; i<_countof(AscentTrackingIntervals); ++i)
	{
		Serial.print(ascentRateData[i].m_AscentRate, 3);
		Serial.print(',');
	}

	Serial.print(packet.gpsCourse);
	Serial.print(',');
	Serial.print(TinyGPS::cardinal(packet.gpsCourse));
	Serial.print(',');
	Serial.print(packet.gpsSpeed);
	Serial.print(',');
	Serial.print(packet.bmpPressure);
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

	for (uint32_t i=0; i<_countof(AscentTrackingIntervals); ++i)
	{
		Serial.print("gpsAsc (m/s@");
		Serial.print(AscentTrackingIntervals[i] / 1000);
		Serial.print("s),");
	}

	Serial.print("gpsCourse (deg),");
	Serial.print("gpsCourse (cardinal),");
	Serial.print("gpsSpeed (m/s),");
	Serial.print("bmpPressure (Pa),");
	Serial.print("tmpInt (C),");
	Serial.print("tmpExt (C),");
	Serial.print("battery (V),");

	Serial.println();
}

void transmitLogging(uint32_t now)
{
	loggingLastSend = now;

	Serial.print("Logging,");
	Serial.print(now);	
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
	const bool hasPosition = gps.f_get_position(&lat, &lon);

	if (hasPosition)
	{
		Serial.print(lat, 6);
		Serial.print(',');
		Serial.print(lon, 6);
		Serial.print(',');
		Serial.print(gps.f_altitude(), 3);
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
	}


	if (hasPosition && 
	    telemetryReceiveCount > 0 &&
	    latestTelemetryPacket.gpsLat != 0.0f &&
	    latestTelemetryPacket.gpsLon != 0.0f)
	{
		Serial.print(TinyGPS::distance_between(lat, lon, latestTelemetryPacket.gpsLat, latestTelemetryPacket.gpsLon), 3);
		Serial.print(',');
		Serial.print(TinyGPS::course_to(lat, lon, latestTelemetryPacket.gpsLat, latestTelemetryPacket.gpsLon), 0);
		Serial.print(',');
		Serial.print(TinyGPS::cardinal(TinyGPS::course_to(lat, lon, latestTelemetryPacket.gpsLat, latestTelemetryPacket.gpsLon)));
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

void transmitLCD(uint32_t now)
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
		LCDSerial.print(latestTelemetryPacket.bmpPressure);
		LCDSerial.print('P');

		LCDSerial.print(c_GoToLine2);
		LCDSerial.print("Asc ");

		for (uint32_t i=0; i<_countof(AscentTrackingIntervals); ++i)
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
		LCDSerial.print(latestTelemetryPacket.gpsSpeed);
		LCDSerial.print("m/s ");
		LCDSerial.print(TinyGPS::cardinal(latestTelemetryPacket.gpsCourse));

		LCDSerial.print(c_GoToLine2);
		LCDSerial.print("Car ");
		if (gps.get_position(NULL, NULL))
		{
			LCDSerial.print(gps.f_speed_mps(), 0);
			LCDSerial.print("m/s ");
			LCDSerial.print(TinyGPS::cardinal(gps.f_course()));
		}
		else
		{
			LCDSerial.print("---");
		}

		break;
	
	case 3:
		LCDSerial.print("Rng ");
		float lat, lon;
		if (gps.f_get_position(&lat, &lon) && 
		    telemetryReceiveCount > 0 &&
		    latestTelemetryPacket.gpsLat != 0.0f &&
		    latestTelemetryPacket.gpsLon != 0.0f)
		{
			LCDSerial.print(TinyGPS::distance_between(lat, lon, latestTelemetryPacket.gpsLat, latestTelemetryPacket.gpsLon), 0);
			LCDSerial.print("m ");
			LCDSerial.print(TinyGPS::cardinal(TinyGPS::course_to(lat, lon, latestTelemetryPacket.gpsLat, latestTelemetryPacket.gpsLon)));
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

void transmitPing(uint32_t now)
{
	pingLastSend = now;
	
	PingPacket packet;
	packet.time = millis();

	++pingSendCount;

	xtend.SendTo(XTendDest, (uint8_t*)&packet, sizeof(packet));
}

