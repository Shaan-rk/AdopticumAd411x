#include <Adopticum_AD411x.h>

namespace config {
  const char *Program = "AD4111_SINGLE_CONV_MultiChannel";
  const char *Version = "2025-08-14";
  const long SerialSpeed = 115200;

  const int CS_PIN     = 5;
  const int DRDY_PIN   = 19;  // not used here
  const uint8_t pin_sync = 17;
  const uint8_t pin_vaux = 32;
  const uint8_t pin_24v_en = 33;

  const uint16_t ODR = (uint16_t)AD4116::OutputDataRate::SPS_500;
}

// VINx_VINCOM mapping for AD4111 (only channels 0–7 exist)
AD4116::Input channel_inputs[] = {
    AD4116::Input::VIN0_VINCOM,
    AD4116::Input::VIN1_VINCOM,
    AD4116::Input::VIN2_VINCOM,
    AD4116::Input::VIN3_VINCOM,
    AD4116::Input::VIN4_VINCOM,
    AD4116::Input::VIN5_VINCOM,
    AD4116::Input::VIN6_VINCOM,
    AD4116::Input::VIN7_VINCOM
};

void setup_ad4111_single()
{
    ad4116.reset();

    // Disable all channels initially
    for (byte ch = 0; ch < 8; ch++) {
        ad4116.disable_channel(ch);
    }

    // Configure setup 0 for bipolar voltage, internal ref, and buffers
    uint16_t setup_cfg = AD411x::Setup::BIPOLAR | AD411x::Setup::INTERNAL_REF
        | AD411x::Setup::INPUT_BUFFERS | AD411x::Setup::REFBUF_P | AD411x::Setup::REFBUF_N;
    ad4116.configure_setup(0, setup_cfg);

    // Configure ODR for setup 0
    ad4116.write_filter_register(0, config::ODR);

    // Set ADC mode to SINGLE_CONV
    uint16_t adc_mode =
        (uint16_t)AD411x::ADCMode::Mode::CONTINUOUS |
        (uint16_t)AD411x::ADCMode::Clock::EXTERNAL_CRYSTAL |
        (uint16_t)AD411x::ADCMode::Delay::DELAY_0 |
        (uint16_t)AD411x::ADCMode::REF_EN;
    ad4116.write_adc_mode(adc_mode);
}

void setup()
{
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
    if (!ad4116.begin()) {
        Serial.println("Could not initialize AD4111 device.");
        while (1);
    }
    Serial.println("AD4111 initialized.");

    setup_ad4111_single();
}

void loop()
{
    static uint32_t t0 = 0;
    if (millis() - t0 < 500) return; // sample every 500ms
    t0 = millis();

    byte channels_to_read[] = {0, 1, 2, 3, 4, 5, 6, 7};

    Serial.println("----- Channel Readings -----");
    for (byte i = 0; i < sizeof(channels_to_read); i++) {
        byte ch = channels_to_read[i];

        // Disable all channels
        for (byte c = 0; c < 8; c++) ad4116.disable_channel(c);

        // Enable only the current VINx_VINCOM channel
        ad4116.configure_channel(ch, channel_inputs[ch], 0, true);

        delay(5); // allow mux to settle

        // Read voltage only (ignore returned channel number)
        double voltage;
        ad4116.read_volt(nullptr, &voltage);

        Serial.print("Channel ");
        Serial.print(ch);
        Serial.print(": ");
        Serial.print(voltage, 2);
        Serial.println(" V");
    }
    Serial.println("----------------------------");
}