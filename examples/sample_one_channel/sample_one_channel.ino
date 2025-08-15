/*
sample_one_channel.ino
Example code for Adopticum_AD411x Analog to Digital converter Arduino library.

This example connects to an AD4116 device connected to the Arduino's native SPI bus.
The AD4116 is configured to measure voltage between VIN0 and VIN1
and to measure continuously and output data at the configured data rate.

Note: 
For interrupt handling to work, connect DRDY pin to CIPO (a.k.a. MISO) pin.
With a default Arduino setup that would be pin 3 for DRDY and pin 12 CIPO.

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#include <Adopticum_AD411x.h>

namespace config {
  const char *Program = "sample_one_channel";
  const char *Version = "2025-08-08";

	// Only REQUIRE serial connection to computer in debugging scenario. 
	const bool SerialDebug = false;
  const long SerialSpeed = 115200;

  // Pin on Arduino connected to chip select on AD411x.
  const int CS_PIN = 5;
  const uint8_t pin_sync=17;
  const uint8_t pin_vaux=32;
  const uint8_t pin_24v_en=33;
  // Pin on Arduino to trigger interrupt when data is ready.
  // Connect DRDY pin to CIPO (a.k.a. MISO) pin.
  const int DRDY_PIN = 19;  
  // Select an output data rate for the AD4116.
  const uint16_t ODR = (uint16_t)AD4116::OutputDataRate::SPS_500;
}

void setup_ad4116() {
  ad4116.reset();

  for (byte ch = 0; ch < 16; ch++) {
    ad4116.disable_channel(ch);
  }
  // ad4116.enable_channel(2);

  {
    uint8_t buf[2];
    int i = 8;
    ad4116.configure_channel(i, AD4116::Input::IN0P_IN0N, 0, true);
    ad4116.read_register(0x10+i, buf, 2);
    Serial.printf("-- CH%d register (0x", i);
    print_bytes(buf, 2);
    Serial.println(").");
  }

  uint16_t setup_cfg = AD411x::Setup::BIPOLAR | AD411x::Setup::INTERNAL_REF
    | AD411x::Setup::REFBUF_P | AD411x::Setup::REFBUF_N; //| AD411x::Setup::INPUT_BUFFERS 
  ad4116.configure_setup(0, setup_cfg);

  // byte status_setup[2] = { 0x00, 0x00 };
  // ad4116.write_register(0x00, status_setup, 1);
  

  ad4116.write_filter_register(0, config::ODR);

  uint16_t adc_mode = 
      (uint16_t)AD411x::ADCMode::Mode::CONTINUOUS |
      (uint16_t)AD411x::ADCMode::Clock::EXTERNAL_CRYSTAL |
      (uint16_t)AD411x::ADCMode::Delay::DELAY_0 |
      (uint16_t)AD411x::ADCMode::REF_EN;
  ad4116.write_adc_mode(adc_mode);
}

void setup() 
{
	// Generic initialisation.
	if (config::SerialDebug) { while (!Serial) { delay(10); } }
	Serial.begin(config::SerialSpeed);
  Serial.println(config::Program);
	Serial.println(config::Version);

	pinMode(config::pin_vaux, OUTPUT);
  pinMode(config::pin_24v_en, OUTPUT);
  pinMode(config::pin_sync, OUTPUT);

  digitalWrite(config::pin_sync, HIGH);
  digitalWrite(config::pin_vaux, HIGH);
  digitalWrite(config::pin_24v_en, HIGH);

  // Setup communication with AD411x.
  ad4116.setup(config::CS_PIN);
  if (!ad4116.begin()){ //|| !ad4116.check_id()) {
    Serial.println("Could not initialize AD411x device.");
    
    while (1);
  }
  Serial.println("AD411x initialized.");

  // Configure AD4116.
  setup_ad4116();

  Serial.println("------");

  // ad4116.set_data_stat(true);

  byte iface[2];
  ad4116.read_register(0x02, iface, 2);  // Read 2 bytes (16 bits)
  Serial.print("Interface mode register: 0x");
  Serial.println(((uint16_t)iface[0] << 8) | iface[1], HEX);
  Serial.print("IFMODE LSB (should have bit 6 = DATA_STAT): 0x");
  Serial.println(iface[1], BIN);
  bool data_stat_enabled = (iface[1] & (1 << 6)) != 0;
  Serial.print("DATA_STAT bit is ");
  Serial.println(data_stat_enabled ? "enabled" : "disabled");

  byte buf[4];
  ad4116.read_register(0x04, buf, 4);  // try to force 4-byte read
  Serial.print("Raw data + status: ");
  for (int i = 0; i < 4; i++) {
    Serial.printf("0x%02X ", buf[i]);
  }
  Serial.println();

  byte stat;
  ad4116.read_register(0x00, &stat, 1);
  Serial.printf("Status reg (manual): 0x%02X\n", stat);

  uint8_t adc[2];
  ad4116.read_register(0x01, adc, 2);
  Serial.printf("adc: %x, %x\n", adc[0], adc[1]);

  ad4116.read_many_things();

  Serial.println("setup done.");
  Serial.println("------");
}

void print_bytes(byte *data, byte data_len)
		{
			char sz[4];
			for (auto i = 0; i < data_len; i++)
			{
				snprintf(sz, 4, " %02X", data[i]);
				Serial.print(sz);
			}
		}

byte channel;
double volt;
double current_A;
uint32_t t0 = 0;

void loop() 
{
  byte buf[4];
  auto now = millis();
  if ((now - t0) < 500) { return; }
  t0 = now;

  // ad4116.read_volt(&channel, &volt);

  // Serial.print("Channel: ");
  // Serial.print(channel);
  // Serial.print(", ");
  // Serial.print(volt, 6);
  // Serial.println(" V");

  ad4116.read_current(&channel, &current_A);
  Serial.printf("Channel %d: %.6f A (%.3f mA)\n", channel, current_A, current_A * 1000.0);
}