#include <Thermistor.h>

Thermistor temp(0);

void setup() {
  Serial.begin(9600);
}

void loop() {
  int temperature = int(temp.getTemp());
  
  Serial.print(temperature);
  delay(1000);
}
