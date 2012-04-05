#include <Core.h>
#include <Wire.h>
#include <VectorMath.h>
#include <MatrixMath.h>
#include <Quaternion.h>
#include <FPS.h>
#include <TinyGPS.h>
#include <HMC5843.h>
#include <ADXL345.h>
#include <ITG3200.h>
#include <BMP085.h>
#include <TMP102.h>
#include <Thermistor.h>
#include <SoftwareSerial.h>

#include <XTendAPI.h>

#include "Config.h"
#include "Packets.h"

uint32_t lastFrameTime = 0;
FPS fps(TargetFrameTime);

const float c_BatteryVoltageScale = 3.3f / 1024.0f / (3.9 / (3.9 + 1.2));
float batteryVoltage = 0.0f;
float batteryVoltageSmooth = 0.0f;

TinyGPS gps;
HMC5843 magneto;
ADXL345 accel;
ITG3200 gyro;
BMP085 pressure;
TMP102 tmps[ETMPs::EnumCount] = TMPAddresses;
Thermistor therms[EThermistors::EnumCount] = ThermistorPins;
float thermTempsFiltered[_countof(therms)];

vec3 accelFiltered(0.0f, 0.0f, -9.8f);
vec3 angVelFiltered(0.0f, 0.0f, 0.0f);

SoftwareSerial XTendSerial(XTendSerialRXPin, XTendSerialTXPin);
XTendAPI xtend(&XTendSerial);
uint32_t packetNum = 0;

uint32_t loggingLastSend = 0;
uint32_t telemetryLastSend = 0;


void xtendReceive();
void transmitLoggingHeadings();
void transmitLogging(uint32_t now);
void transmitTelemetry(uint32_t now);

void setup()
{
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	Serial.begin(LoggingBaud);

	Wire.begin();

	// xtend setup
	XTendSerial.begin(XTendBaud);
	//pinMode(XTendPTTPin, OUTPUT);
	//digitalWrite(XTendPTTPin, HIGH);
	
	delay(3000);

	// setup the sensors
	pinMode(BatteryMonitorPin, INPUT);
	batteryVoltage = analogRead(BatteryMonitorPin) * c_BatteryVoltageScale;
	batteryVoltageSmooth = batteryVoltage;

	GPSSerial.begin(GPSBaud);

	magneto.setup(HMC5843::EOutputRate::FiftyHz);
	accel.setup();
	gyro.setup();
	pressure.setup();
	pressure.SetOversamplingSetting(3);
	for (uint8_t i=0; i<ETMPs::EnumCount; ++i)
		tmps[i].setup(true, TMP102::EConversionRate::Hz8);

	for (uint8_t i=0; i<_countof(therms); ++i)
		thermTempsFiltered[i] = therms[i].getTemp();

	pressure.SetReferencePressure(101325);
	pressure.loop();

	uint32_t now = millis();
	lastFrameTime = now;
	loggingLastSend = now - LoggingStagger;
	telemetryLastSend = now - TelemetryTransmitStagger;
	
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
	
	// update sensors
	batteryVoltage = analogRead(BatteryMonitorPin) * c_BatteryVoltageScale;
	batteryVoltageSmooth = LowPassFilter(batteryVoltage, batteryVoltageSmooth, dt, 2.5f);

	for (uint8_t i=0; i<_countof(therms); ++i)
		thermTempsFiltered[i] = LowPassFilter((float)therms[i].getTemp(), thermTempsFiltered[i], dt, 2.5f);

	while (GPSSerial.available())
		gps.encode(GPSSerial.read());
	
	magneto.loop();
	accel.loop();
	gyro.loop();
	pressure.loopAsync();
	for (uint8_t i=0; i<ETMPs::EnumCount; ++i)
		tmps[i].loop();

	accelFiltered = accel.GetOutput();//LowPassFilter(accel.GetOutput(), accelFiltered, dt, 0.25f);
	angVelFiltered = gyro.GetAngVel();//LowPassFilter(gyro.GetBiasedAngVel(), angVelFiltered, dt, 0.25f);

	// network receive
	xtendReceive();

	// time to transmit?
	if (now - loggingLastSend >= LoggingInterval)
		transmitLogging(now);
	if (now - telemetryLastSend >= TelemetryTransmitInterval)// && gps.get_has_fix())
		transmitTelemetry(now);
	
	fps.loop();
}



void xtendReceive()
{
/*
	const XTendAPI::Frame* pFrame;
	while (xtend.Receive(&pFrame))
	{
		if (pFrame && pFrame->m_PayloadLength >= 1)
		{
			switch (pFrame->m_Payload[0])
			{
			case EPacketType::Ping:
				if (pFrame->m_PayloadLength == sizeof(PingPacket))
				{
					const PingPacket* pPacket = reinterpret_cast<const PingPacket*>(pFrame->m_Payload);
					PongPacket pong;
					pong.time = pPacket->time;
					xtend.SendTo(XTendDest, (uint8_t*)&pong, sizeof(pong));
				}
				break;
			}
		}
	}
*/
}



void transmitLoggingHeadings()
{
	Serial.print("now (ms),");
	Serial.print("fps,");
	Serial.print("battery (V),");

	Serial.print("gpsTime,");
	Serial.print("gpsLat (deg),");
	Serial.print("gpsLon (deg),");
	Serial.print("gpsAlt (m),");
	Serial.print("gpsCourse (deg),");
	Serial.print("gpsCourse (cardinal),");
	Serial.print("gpsSpeed (m/s),");
	Serial.print("gpsSats,");
	
	Serial.print("bmpTemp (deg C),");
	Serial.print("bmpPressure (Pa),");
	Serial.print("bmpAlt (m),");

	for (uint8_t i=0; i<EThermistors::EnumCount; ++i)
		serprintf(Serial, "thermistor%hu (deg C),", i);
	
	for (uint8_t i=0; i<ETMPs::EnumCount; ++i)
		serprintf(Serial, "tmp%hu (deg C),", i);
	
	Serial.print("gyroTemp (deg C),");
	
	Serial.print("accelX (m/s^2),");
	Serial.print("accelY (m/s^2),");
	Serial.print("accelZ (m/s^2),");
	
	Serial.print("angVelX (deg/s),");
	Serial.print("angVelY (deg/s),");
	Serial.print("angVelZ (deg/s),");
	
	Serial.print("magX (Gauss),");
	Serial.print("magY (Gauss),");
	Serial.print("magZ (Gauss),");
	
	Serial.print("\n");
}

void transmitLogging(uint32_t now)
{
	loggingLastSend = now;

	Serial.print(now);
	Serial.print(',');
	Serial.print(fps.GetFramerate());
	Serial.print(',');

	Serial.print(batteryVoltageSmooth, 3);
	Serial.print(',');

	uint8_t hours, minutes, seconds, hundredths;
	float lat, lon;
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
	Serial.print(TinyGPS::cardinal(gps.f_course()));
	Serial.print(',');
	Serial.print(gps.f_speed_mps(), 3);
	Serial.print(',');
	Serial.print(gps.satellites());
	Serial.print(',');

	Serial.print(pressure.GetTempInC(), 3);
	Serial.print(',');
	Serial.print(pressure.GetPressureInPa());
	Serial.print(',');
	Serial.print(pressure.GetAltitudeInM(), 3);
	Serial.print(',');
	
	for (uint8_t i=0; i<EThermistors::EnumCount; ++i)
	{
		Serial.print(thermTempsFiltered[i], 2);
		Serial.print(',');
	}
	
	for (uint8_t i=0; i<ETMPs::EnumCount; ++i)
	{
		Serial.print(tmps[i].GetTemp(), 3);
		Serial.print(',');
	}
	
	Serial.print(gyro.GetTemp(), 3);
	Serial.print(',');
	
	Serial.print(accelFiltered.x, 6);
	Serial.print(',');
	Serial.print(accelFiltered.y, 6);
	Serial.print(',');
	Serial.print(accelFiltered.z, 6);
	Serial.print(',');
	
	Serial.print(angVelFiltered.x, 6);
	Serial.print(',');
	Serial.print(angVelFiltered.y, 6);
	Serial.print(',');
	Serial.print(angVelFiltered.z, 6);
	Serial.print(',');
	
	const vec3 mag = magneto.GetOutput();
	Serial.print(mag.x, 6);
	Serial.print(',');
	Serial.print(mag.y, 6);
	Serial.print(',');
	Serial.print(mag.z, 6);
	Serial.print(',');
	
	Serial.println();
}

void transmitTelemetry(uint32_t now)
{
	telemetryLastSend = now;
	
	TelemetryPacket packet;
	packet.time = now / 1000;

	gps.f_get_position(&packet.gpsLat, &packet.gpsLon);
	packet.gpsAlt = gps.altitude() / 100;
	packet.gpsCourse = gps.course() / 100;
	packet.gpsSpeed = gps.f_speed_mps();

	//packet.tmpInternal = (int8_t)(tmps[ETMPs::Internal].GetTemp() + 0.5f);
	//packet.tmpExternal = (int8_t)(tmps[ETMPs::External].GetTemp() + 0.5f);
	packet.tmpInternal = (int8_t)(thermTempsFiltered[EThermistors::Internal] + 0.5f);
	packet.tmpExternal = (int8_t)(thermTempsFiltered[EThermistors::External1] + 0.5f);

	packet.batteryVoltage = (uint16_t)fabs(batteryVoltageSmooth * 1000.0f + 0.5f);
	
	xtend.SendTo(XTendDest, (uint8_t*)&packet, sizeof(packet));
}


