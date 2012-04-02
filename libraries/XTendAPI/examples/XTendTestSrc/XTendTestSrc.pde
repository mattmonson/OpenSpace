#include <Core.h>
#include <FPS.h>
#include <GPS.h>
#include <XTendAPI.h>

#include <Wire.h>

#define GPSSerial Serial1
#define XTendSerial Serial2
#define XTendPTTPin 23

u32 lastFrameTime = 0;
FPS fps(0);

XTendAPI xtend(&XTendSerial);
const XTendAPI::Address xtendDest = 0x5854;
u32 xtendLastSend = 0;

GPS gps(&GPSSerial);

void setup()
{
	Serial.begin(115200);
	serprintf(Serial, "%lu: Serial up and running...\n", millis());

	delay(1000);

	xtend.setup(115200);
	pinMode(XTendPTTPin, OUTPUT);
	digitalWrite(XTendPTTPin, HIGH);
	
	// setup the sensors
	gps.setup(115200);

	u32 now = millis();
	lastFrameTime = now;
	xtendLastSend = now;
}

void loop()
{
	u32 now = millis();
	f32 dt = (now - lastFrameTime) * 0.001f;
	lastFrameTime = now;

	fps.increment();
	
	// update sensors
	gps.loop();
	
	if (now - xtendLastSend > 1000)
	{
		serprintf(Serial, "Sending: %lu\n", now);
		xtend.SendTo(xtendDest, (u8*)&now, sizeof(now));
		
		xtendLastSend = now;
	}

	fps.loop();
}
