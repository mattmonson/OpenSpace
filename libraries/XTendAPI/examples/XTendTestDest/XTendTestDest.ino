#include <Core.h>
#include <FPS.h>
#include <GPS.h>
#include <XTendAPI.h>

#include <Wire.h>

#define GPSSerial Serial2
#define XTendSerial Serial1
#define XTendPTTPin 2

uint32_t lastFrameTime = 0;
FPS fps(0);

XTendAPI xtend(&XTendSerial);

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

	lastFrameTime = millis();
}

void loop()
{
	uint32_t now = millis();
	float dt = (now - lastFrameTime) * 0.001f;
	lastFrameTime = now;

	fps.increment();
	
	// update sensors
	gps.loop();
	
	const XTendAPI::Frame* pFrame;
	while (xtend.Receive(&pFrame))
	{
serprintf(Serial, "SomeKindOfFrame\n");
		if (pFrame)
		{
			serprintf(Serial, "Frame: \n");
			serprintf(Serial, "- Length:  %ud\n", pFrame->m_Length);
			serprintf(Serial, "- APIID:   %hX\n", pFrame->m_APIIdentifier);
			serprintf(Serial, "- SrcAddr: %X\n",  pFrame->m_SrcAddress);
			serprintf(Serial, "- RSSI:    %hd\n", pFrame->m_RSSI);
			serprintf(Serial, "- Options: %hX\n", pFrame->m_Options);
			serprintf(Serial, "- PayLen:  %hu\n", pFrame->m_PayloadLength);
			serprintf(Serial, "- Payload: ");
			for (uint16_t i=0; i<pFrame->m_PayloadLength; ++i)
				serprintf(Serial, "%.2hX", pFrame->m_Payload[i]);
			serprintf(Serial, "\n");
		}
	}

	fps.loop();
}
