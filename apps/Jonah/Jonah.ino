#include <Core.h>
#include <Wire.h>
#include <FPS.h>
#include <BMP085.h>
#include <Thermistor.h>
#include <CRC.h>

uint32_t lastFrameTime = 0;
FPS fps(0);

const int c_BatteryMonitorPin = A3;
const float c_BatteryVoltageScale = 3.3f / 1024.0f / (3.9 / (3.9 + 1.2));
float batteryVoltage = 0.0f;
float batteryVoltageSmooth = 0.0f;

Thermistor therm(A0);
float thermTempFiltered;

BMP085 pressure;
float pressureFiltered;

uint32_t lastSend = 0;

void transmit(uint32_t now);

void setup()
{
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	Serial.begin(4800);

	Wire.begin();

	delay(1000);

	// setup the sensors
	pinMode(c_BatteryMonitorPin, INPUT);
	batteryVoltage = analogRead(c_BatteryMonitorPin) * c_BatteryVoltageScale;
	batteryVoltageSmooth = batteryVoltage;

	thermTempFiltered = therm.getTemp();

	pressure.setup();
	pressure.SetOversamplingSetting(3);
	pressure.SetReferencePressure(101325);
	pressure.loop();
	pressureFiltered = pressure.GetPressureInPa();

	uint32_t now = millis();
	lastFrameTime = now;
	lastSend = now;
	
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
	batteryVoltage = analogRead(c_BatteryMonitorPin) * c_BatteryVoltageScale;
	batteryVoltageSmooth = LowPassFilter(batteryVoltage, batteryVoltageSmooth, dt, 2.5f);

	thermTempFiltered = LowPassFilter((float)therm.getTemp(), thermTempFiltered, dt, 2.5f);

	pressure.loopAsync();
	pressureFiltered = LowPassFilter((float)pressure.GetPressureInPa(), pressureFiltered, dt, 2.5f);

	// transmit
	transmit(now);
	
	fps.loop();
}

void transmit(uint32_t now)
{
	lastSend = now;

	struct JonahPacket
	{
		uint32_t now;
		uint32_t batteryVoltage : 14; // in milli-Volts
		uint32_t bmpPressure    : 18; // in Pa
		int32_t bmpTemp         : 12; // in deci-C
		int32_t thermTemp       : 20; // in milli-C
	};

	JonahPacket p;
	p.now = now;
	p.batteryVoltage = floor(batteryVoltageSmooth * 1000.0f + 0.5f);
	p.bmpPressure = floor(pressureFiltered + 0.5f);
	p.bmpTemp = pressure.GetTempInDeciC();
	p.thermTemp = floor(thermTempFiltered * 1000.0f + 0.5f);

	for (int i=0; i<10; ++i)
		Serial.write((uint8_t)0);

	uint32_t crc = crc32_init();

	Serial.write((uint8_t)sizeof(p));
	crc = crc32_update(crc, (uint8_t)sizeof(p));

	for (size_t i=0; i<sizeof(p); ++i)
	{
		const uint8_t byte = reinterpret_cast<uint8_t*>(&p)[i];

		Serial.write(byte);
		crc = crc32_update(crc, byte);
	}

	crc = crc32_finish(crc);

	Serial.write((uint8_t*)&crc, sizeof(crc));
}
