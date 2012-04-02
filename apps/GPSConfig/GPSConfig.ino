#define GPSSerial Serial1
//#define GPSBaud 115200
//#define GPSBaud 57600  // default for 10Hz
//#define GPSBaud 9600   // default for 5Hz

// PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*           // set report frequency, in /update (yaesu vx8dr, TinyGPS)
// PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*           // set report frequency, in /update (plane, deprecated)
// PMTK314,-1*                                          // set report frequency to default
// PMTK220,200*                                         // set update rate, in ms (5 Hz)
// PMTK220,100*                                         // set update rate, in ms (10 Hz)
// PMTK251,115200*                                      // set baud, in hz

//$PMTK251,9600*17
//$PMTK251,115200*1F
//$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28
//$PMTK220,100*2F
//$PMTK220,200*2C
//$PMTK220,1000*1F

void setup()
{
	Serial.begin(115200);
	GPSSerial.begin(GPSBaud);
}

char commandStr[256];
byte commandLen = 0;

void loop()
{
	char str[256];

	while (Serial.available())
	{
		commandStr[commandLen++] = Serial.read();
		if (commandStr[commandLen-1] == '*')
		{
			commandStr[commandLen] = 0;

			byte checksum = 0;
			for (byte i=0; i<commandLen-1; ++i)
				checksum ^= commandStr[i];

			sprintf(str, "$%s%.2X\r\n", commandStr, checksum);
			Serial.print(str);

			GPSSerial.print(str);

			commandLen = 0;
		}
	}

	while(GPSSerial.available())
	{
		char c = GPSSerial.read();
		Serial.print(c);
	}
}

//  $GPGGA,065605.000,4739.8881,N,12212.2641,W,1,5,2.15,21.2,M,-17.2,M,,*56
//  $GPGLL,4739.8881,N,12212.2641,W,065605.000,A,A*48
//  $GPGSA,A,3,07,16,13,08,23,,,,,,,,2.33,2.15,0.89*0C
//  $GPGSV,4,1,14,13,82,339,26,23,60,115,28,07,54,261,36,16,31,049,26*7E
//  $GPGSV,4,2,14,08,19,251,26,03,18,110,,06,17,096,,10,16,320,*7A
//  $GPGSV,4,3,14,20,15,178,,47,11,246,,02,06,297,24,04,03,260,16*7A
//  $GPGSV,4,4,14,05,03,329,21,19,02,130,*79
//  $GPRMC,065605.000,A,4739.8881,N,12212.2641,W,0.04,120.35,150110,,,A*7A
//  $GPVTG,120.35,T,,M,0.04,N,0.07,K,A*3B
