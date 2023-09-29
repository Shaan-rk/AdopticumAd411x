#include <Adafruit_SPIDevice.h>
#include "Adopticum_AD411x.h"
#include "Adopticum_AD411x_Definitions.h"

namespace config {
  const char *Program = "sample_one_channel";
  const char *Version = "2023-09-26";

	// Only REQUIRE serial connection to computer in debugging scenario. 
	const bool SerialDebug = false;
  const long SerialSpeed = 115200;

  // Pin on Arduino connected to chip select on AD411x.
  const int CS_PIN = 10;
  // Pin on Arduino to trigger interrupt when data is ready.
  // Connect DRDY pin to CIPO (a.k.a. MISO) pin.
  const int DRDY_PIN = 3;  
}


void hardcoded_ad4116_setup()
{
  const uint16_t CH_ENABLE = 0x8000;
  const uint16_t VIN0_VIN1   = 0b0000000001;
  const uint16_t VIN0_VINCOM = 0b0000010000;
  const uint16_t VIN1_VIN0   = 0b0000100000;
  uint16_t setup_no = 0 << 12;
  // Configure Channel 0
  uint16_t ch_cfg = CH_ENABLE | setup_no | VIN0_VIN1;
  ad4116.write_register(Registers::CH0, ch_cfg);

  // Configure Setup 0. Default: 0x1000
  const uint16_t BIPOLAR = 0x1000;
  const uint16_t REFBUF  = 0x0C00;
  const uint16_t INPUT_BUFFER = 0x0300;
  const uint16_t REFSEL = 0x0000; // 0x0=Ext ref, 0x2= Int ref (enable via ADCMODE), 0x3 = AVDD-AVSS
  // Keep default? (0x1000)
  uint16_t setup_cfg = BIPOLAR | REFBUF | INPUT_BUFFER | REFSEL;
  ad4116.write_register(Registers::SETUP0, setup_cfg);

  // Configure filter. Default: 0x0500
  //TODO: Sinc3, Enhanced 50/60Hz rejection, post filters, order of digital filter, ...
  uint16_t filter_cfg = 0x0500 | OutputDataRate::SPS_5194;
  ad4116.write_register(Registers::FILTER0, filter_cfg);

  // Configure ADC mode to continuous.
  uint16_t adc_mode = 0x0000;
  ad4116.write_register(Registers::ADCMOD, adc_mode);
  
  // Set data_stat to get channel no with each sample.
  ad4116.set_data_stat(true);

  // Use one pin for data ready interrupt.
  ad4116.enable_interrupt(config::DRDY_PIN);
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
  Serial.println("AD411x initialized.");

  // Configure AD411x.
  ad4116.reset();
  hardcoded_ad4116_setup();

  // Setup analog outputs as sample signals.
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  digitalWrite(A0, LOW);  // keep A0 as "GND" always.
  Serial.println("setup done.");
}


byte channel;
double volt;
uint32_t t0 = 0;

void loop() 
{
  // AD411x is set to continuous mode.
  // We do nothing until there is data ready to read.
  if (!ad4116.is_data_ready()) { return; }

  // Read the latest sample from the AD-converter.
  ad4116.read_volt(&channel, &volt);

  // Limit the rest of the loop to only once per second.
  auto now = millis();
  if ((now - t0) < 1000) { return; }
  t0 = now;

  // Print the latest sample.
  Serial.print("Channel: ");
  Serial.print(channel);
  Serial.print(", ");
  Serial.print(volt);
  Serial.println(" V");

  // Toggle some outputs to use as example analog signals  
  byte toggle = ((now / 1000) % 4) > 0;
  Serial.print("Toggle: ");
  Serial.println(toggle);
  // Use digitalWrite to get 0V or 5V, instead of PWM.
  digitalWrite(A1, toggle);   //analogWrite(A1, 255 * toggle);
  digitalWrite(A2, toggle);   //analogWrite(A2, 255 * toggle);
  digitalWrite(A3, 1-toggle); //analogWrite(A3, 255 * (1-toggle));

  // Debug the interrupt handler using some counters.
  Serial.print("Interrupt / lap): ");
  Serial.println(ad4116.get_interrupt_lap());
  Serial.print("Skips / lap: ");
  Serial.println(ad4116.get_skip_lap());
  Serial.print("Data ready / lap: ");
  Serial.println(ad4116.get_drdy_lap());
  Serial.print("Reads / lap: ");
  Serial.println(ad4116.get_read_lap());
}
