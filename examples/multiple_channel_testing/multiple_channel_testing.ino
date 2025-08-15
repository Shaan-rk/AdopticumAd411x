#include <Adopticum_AD411x.h>

namespace config {
  const char *Program = "sample_multiple_channel";
  const char *Version = "13/08/2025";

  const bool SerialDebug = false;
  const long SerialSpeed = 115200;

  const int CS_PIN = 5;
  const uint8_t pin_sync = 17;
  const uint8_t pin_vaux = 32;
  const uint8_t pin_24v_en = 33;
  const int DRDY_PIN = 19;  // Not used now

  const uint16_t ODR = (uint16_t)AD4116::OutputDataRate::SPS_500; // 500 SPS
}

void setup_ad4116() {
  ad4116.reset();

  // Disable all channels and mark not configured in library array
  for (byte ch = 0; ch < 16; ch++) {
    ad4116.disable_channel(ch);
  }

  // Configure channel 0
  {
    uint8_t buf[2];
    int i = 4;
    ad4116.configure_channel(i, AD4116::Input::VIN4_VINCOM, 0, true);
    ad4116.read_register(0x10 + i, buf, 2);
    Serial.printf("-- CH%d register (0x", i);
    print_bytes(buf, 2);
    Serial.println(").");
  }

  // Configure channel 1
  {
    uint8_t buf[2];
    int i = 5;
    ad4116.configure_channel(i, AD4116::Input::VIN5_VINCOM, 0, true);
    ad4116.read_register(0x10 + i, buf, 2);
    Serial.printf("-- CH%d register (0x", i);
    print_bytes(buf, 2);
    Serial.println(").");
  }

  uint16_t setup_cfg = AD411x::Setup::BIPOLAR | AD411x::Setup::INTERNAL_REF
    | AD411x::Setup::INPUT_BUFFERS | AD411x::Setup::REFBUF_P | AD411x::Setup::REFBUF_N;
  ad4116.configure_setup(1, setup_cfg);

  ad4116.write_filter_register(1, config::ODR);

  uint16_t adc_mode =
    (uint16_t)AD411x::ADCMode::Mode::CONTINUOUS |
    (uint16_t)AD411x::ADCMode::Clock::EXTERNAL_CRYSTAL |
    (uint16_t)AD411x::ADCMode::Delay::DELAY_0 |
    (uint16_t)AD411x::ADCMode::REF_EN;
  ad4116.write_adc_mode(adc_mode);

  // ad4116.set_data_stat(true);

  delay(10);
}

void setup() {
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

  ad4116.setup(config::CS_PIN);
  if (!ad4116.begin()){
    Serial.println("Could not initialize AD411x device.");
    while (1);
  }

  Serial.println("AD411x initialized.");

  setup_ad4116();

  Serial.println("------");

  byte iface[2];
  ad4116.read_register(0x02, iface, 2);  // Read 2 bytes (16 bits)

  bool data_stat_enabled = (iface[1] & (1 << 6)) != 0;
  Serial.print("DATA_STAT bit is ");
  Serial.println(data_stat_enabled ? "enabled" : "disabled");

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

void print_bytes(byte *data, byte data_len) {
  char sz[4];
  for (auto i = 0; i < data_len; i++) {
    snprintf(sz, 4, " %02X", data[i]);
    Serial.print(sz);
  }
}

byte channel;
double volt;
double samples[8] = {0.0};
uint32_t t0 = 0;

void loop() {
  auto now = millis();
  if ((now - t0) < 500) return;
  t0 = now;

  ad4116.read_volt(&channel, &volt);

  if (channel < 8) samples[channel] = volt;

  Serial.print("Latest samples:");
  for (auto i = 0; i < 8; i++) {
    Serial.print(" ch ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(samples[i]);
    Serial.print(" V ");
  }
  Serial.println();

  ad4116.read_register(0x00, &channel, 1);
  Serial.print("Status register (0x");
  print_bytes(&channel, 1);
  Serial.println(").");
}