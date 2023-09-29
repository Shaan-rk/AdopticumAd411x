#include <Adafruit_SPIDevice.h>
#include "Adopticum_AD411x.h"
#include "Adopticum_AD411x_Definitions.h"

namespace config {
  const char *Program = "hello_ad4116";
  const char *Version = "2023-09-26";

	// Only REQUIRE serial connection to computer in debugging scenario. 
	const bool SerialDebug = false;
  const long SerialSpeed = 115200;

  // Pin on Arduino connected to chip select on AD411x.
  const int CS_PIN = 10;
}


void setup() 
{
	// Generic initialisation.
	if (config::SerialDebug) { while (!Serial) { delay(10); } }
	Serial.begin(config::SerialSpeed);
  Serial.println(config::Program);
	Serial.println(config::Version);
	pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Setup communication with AD411x.
  ad4116.setup(config::CS_PIN);
  if (!ad4116.begin() || !ad4116.check_id()) {
    Serial.println("Could not initialize AD411x device.");
    while (1);
  }
  ad4116.reset();
  Serial.println("AD411x initialized.");
}


void loop() 
{
  Serial.println("---");
  ad4116.read_many_things();
  delay(10000);
}
